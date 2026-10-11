#!/usr/bin/env python3
# claude: 2026-10-11 新設 — 「ある時刻のスキャンを、基準姿勢と推定姿勢それぞれで地図に置いたらどれだけ壁に乗るか」を数えて絵にする。
#
#   見失いの瞬間の診断用: 基準姿勢でも乗らない → 地図とスキャンの帯/座標が合っていない (手法のせいではない)
#                         基準姿勢では乗るが推定姿勢で乗らない → 手法が外した
#   hit 率 = スキャン終点のうち、地図の占有セルから hit_dist [m] 以内にあるものの割合
#
# 使い方 (コンテナ内):
#   python3 scan_fit.py <bag_dir> <ref_dir> <map.yaml> <out_dir> --times 30 60 90 [--run <run_dir>] [--scan /rfans/scan]
#     --times は initialpose (run があれば events.txt、無ければ基準の先頭) からの秒
import argparse
import json
import os

import cv2
import numpy as np
import yaml
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from scipy.spatial import cKDTree
from sensor_msgs.msg import LaserScan


def quat_yaw(q):
    x, y, z, w = q
    return np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def pose_at(tum, t):
    """TUM 配列を時刻 t で補間 → (x, y, yaw)"""
    i = np.searchsorted(tum[:, 0], t).clip(1, len(tum) - 1)
    a, b = tum[i - 1], tum[i]
    f = (t - a[0]) / max(1e-6, b[0] - a[0])
    ya, yb = quat_yaw(a[4:8]), quat_yaw(b[4:8])
    yaw = ya + f * np.arctan2(np.sin(yb - ya), np.cos(yb - ya))
    return a[1] + f * (b[1] - a[1]), a[2] + f * (b[2] - a[2]), yaw


def read_scans(bag, topic, wanted, tol=0.15):
    """wanted の各時刻に最も近いスキャンを返す {t: LaserScan}"""
    r = SequentialReader()
    r.open(StorageOptions(uri=bag, storage_id="mcap"), ConverterOptions("", ""))
    out = {}
    best = {t: (tol, None) for t in wanted}
    tmax = max(wanted) + tol
    while r.has_next():
        tp, data, _ = r.read_next()
        if tp != topic:
            continue
        m = deserialize_message(data, LaserScan)
        st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if st > tmax:
            break
        for t in wanted:
            if abs(st - t) < best[t][0]:
                best[t] = (abs(st - t), m)
    for t, (_, m) in best.items():
        if m is not None:
            out[t] = m
    return out


def endpoints(scan, pose, rmax):
    x, y, yaw = pose
    a = scan.angle_min + np.arange(len(scan.ranges)) * scan.angle_increment
    r = np.array(scan.ranges)
    ok = np.isfinite(r) & (r > scan.range_min) & (r < min(rmax, scan.range_max))
    px = x + r[ok] * np.cos(yaw + a[ok])
    py = y + r[ok] * np.sin(yaw + a[ok])
    return np.c_[px, py]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag"); ap.add_argument("ref_dir"); ap.add_argument("map_yaml"); ap.add_argument("out_dir")
    ap.add_argument("--times", type=float, nargs="+", required=True)
    ap.add_argument("--run", default=None, help="run dir (est.tum / events.txt)。無ければ基準だけ")
    ap.add_argument("--scan", default="/rfans/scan")
    ap.add_argument("--rmax", type=float, default=30.0)
    ap.add_argument("--hit_dist", type=float, default=0.3)
    ap.add_argument("--view", type=float, default=25.0, help="画像の半径 [m]")
    args = ap.parse_args()
    os.makedirs(args.out_dir, exist_ok=True)

    ref = np.loadtxt(os.path.join(args.ref_dir, "ref_base.tum"), ndmin=2)
    est = None
    t0 = ref[0, 0]
    if args.run:
        est = np.loadtxt(os.path.join(args.run, "est.tum"), ndmin=2)
        for line in open(os.path.join(args.run, "events.txt")):
            if " initialpose " in line:
                t0 = float(line.split()[0]); break
    y = yaml.safe_load(open(args.map_yaml))
    img = cv2.imread(os.path.join(os.path.dirname(args.map_yaml), y["image"]), cv2.IMREAD_GRAYSCALE)
    res, ox, oy = y["resolution"], y["origin"][0], y["origin"][1]
    H, W = img.shape
    occ = (255 - img.astype(np.float32)) / 255.0 > y["occupied_thresh"]
    rows, cols = np.nonzero(occ)
    tree = cKDTree(np.c_[ox + (cols + 0.5) * res, oy + (H - rows - 0.5) * res])

    wanted = [t0 + t for t in args.times]
    scans = read_scans(args.bag, args.scan, wanted)
    results = []
    for t in args.times:
        ta = t0 + t
        if ta not in scans:
            print(f"t+{t:.0f}s: scan not found"); continue
        sc = scans[ta]
        rec = {"t": t}
        poses = {"ref": pose_at(ref, ta)}
        if est is not None and est[0, 0] <= ta <= est[-1, 0]:
            poses["est"] = pose_at(est, ta)
        for name, p in poses.items():
            pts = endpoints(sc, p, args.rmax)
            d, _ = tree.query(pts, distance_upper_bound=args.hit_dist)
            rec[f"hit_{name}"] = float(np.isfinite(d).mean()); rec[f"n_{name}"] = int(len(pts))
            rec[f"pose_{name}"] = [round(float(v), 3) for v in p]
        results.append(rec)
        # 画像: 基準姿勢 (緑) と推定姿勢 (赤) のスキャンを地図に重ねる
        cx, cy = poses["ref"][0], poses["ref"][1]
        c0, c1 = int((cx - args.view - ox) / res), int((cx + args.view - ox) / res)
        r0, r1 = int(H - (cy + args.view - oy) / res), int(H - (cy - args.view - oy) / res)
        c0, r0 = max(c0, 0), max(r0, 0); c1, r1 = min(c1, W), min(r1, H)
        crop = cv2.cvtColor(img[r0:r1, c0:c1], cv2.COLOR_GRAY2BGR)
        sc_up = 3
        crop = cv2.resize(crop, None, fx=sc_up, fy=sc_up, interpolation=cv2.INTER_NEAREST)
        def px(p):
            return (int((p[0] - ox) / res - c0) * sc_up, int(H - (p[1] - oy) / res - r0) * sc_up)
        for name, color in (("ref", (0, 180, 0)), ("est", (0, 0, 255))):
            if name not in poses:
                continue
            for q in endpoints(sc, poses[name], args.rmax):
                u, v = px(q)
                if 0 <= u < crop.shape[1] and 0 <= v < crop.shape[0]:
                    cv2.circle(crop, (u, v), 2, color, -1)
            u, v = px(poses[name])
            cv2.circle(crop, (u, v), 8, color, 2)
            yaw = poses[name][2]
            cv2.line(crop, (u, v), (int(u + 30 * np.cos(yaw)), int(v - 30 * np.sin(yaw))), color, 2)
        txt = f"t+{t:.0f}s  hit ref {rec.get('hit_ref', 0):.2f}" + (f"  est {rec['hit_est']:.2f}" if "hit_est" in rec else "")
        cv2.putText(crop, txt, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 4, cv2.LINE_AA)
        cv2.putText(crop, txt, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 0), 2, cv2.LINE_AA)
        cv2.imwrite(os.path.join(args.out_dir, f"scanfit_{int(t):04d}s.png"), crop)
        print(json.dumps(rec))
    json.dump(results, open(os.path.join(args.out_dir, "scanfit.json"), "w"), indent=1)


if __name__ == "__main__":
    main()
