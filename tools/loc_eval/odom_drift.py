#!/usr/bin/env python3
# claude: 2026-10-15 新設 — 「オドメトリ単独の軌跡」と「自己位置推定 (odom + localization) の軌跡」を同じ地図に重ね、
#   オドメトリの累積ドリフトを距離あたりで数値化する。AMCL の odom 信頼度 (alpha) をどこまで上げられるかの
#   判断材料にする。
#
#   取り出すもの (bag 時刻):
#     wheel   /odom                      車輪オドメトリ (epos4_odometry、EKF 有効時も車輪だけの値)
#     ekf     /odometry/filtered         車輪 + IMU 融合 (robot_localization)。無ければ TF odom→base_link
#     loc     --ref <tum> (GLIM 基準)    または TF map→odom ∘ odom→base_link (AMCL / lidar3d の実走結果)
#   odom 系は「loc の開始姿勢に一致するよう 1 回だけ剛体変換」して地図座標に置く (以後は補正しない = odom 単独)。
#
#   出力:
#     paths.npz          各軌跡 (t, x, y, yaw) 地図座標
#     overlay.png        地図 + 3 本の軌跡 (開始点・終了点・時刻目盛)
#     drift.png          走行距離に対する位置誤差 / 向き誤差
#     segments.md        区間長 10/50/100 m ごとの相対ドリフト (並進 %・回転 deg/100 m) の中央値と p95
#
# 使い方 (main コンテナ内):
#   python3 odom_drift.py <bag_dir> <out_dir> [--map map.yaml | --map bag] [--ref ref_base.tum]
#                         [--wheel /odom] [--ekf /odometry/filtered] [--title "..."]
import argparse
import math
import os

import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import yaml
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py

C_LOC, C_WHEEL, C_EKF = "#2a78d6", "#eb6834", "#1baf7a"  # dataviz 既定の categorical slot 1〜3


# ---------------------------------------------------------------- 2D 剛体変換 (x, y, yaw)
def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def compose(a, b):  # a ∘ b
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], wrap(a[2] + b[2]))


def inverse(a):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (-(c * a[0] + s * a[1]), -(-s * a[0] + c * a[1]), -a[2])


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def apply(T, P):  # P: (N,4) t,x,y,yaw → 同形
    c, s = math.cos(T[2]), math.sin(T[2])
    out = P.copy()
    out[:, 1] = T[0] + c * P[:, 1] - s * P[:, 2]
    out[:, 2] = T[1] + s * P[:, 1] + c * P[:, 2]
    out[:, 3] = np.vectorize(wrap)(P[:, 3] + T[2])
    return out


def interp(P, t):  # 時刻 t の姿勢を線形補間 (yaw は unwrap してから)
    yaw_u = np.unwrap(P[:, 3])
    x = np.interp(t, P[:, 0], P[:, 1])
    y = np.interp(t, P[:, 0], P[:, 2])
    yw = np.vectorize(wrap)(np.interp(t, P[:, 0], yaw_u))
    return np.column_stack([t, x, y, yw])


# ---------------------------------------------------------------- bag 読み出し
def read_bag(bag, wheel_topic, ekf_topic, want_map):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id="mcap"),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()}
    want = {wheel_topic, ekf_topic, "/tf", "/tf_static"} | ({"/map"} if want_map else set())
    reader.set_filter(rosbag2_py.StorageFilter(topics=[t for t in want if t in types]))
    wheel, ekf, tf_ob, tf_mo, grid = [], [], [], [], None
    while reader.has_next():
        topic, raw, _ = reader.read_next()
        m = deserialize_message(raw, types[topic])
        if topic in (wheel_topic, ekf_topic):
            p = m.pose.pose
            row = (m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, p.position.x, p.position.y, yaw_of(p.orientation))
            (wheel if topic == wheel_topic else ekf).append(row)
        elif topic == "/tf":
            for tr in m.transforms:
                par, ch = tr.header.frame_id.lstrip("/"), tr.child_frame_id.lstrip("/")
                tt = tr.transform
                row = (tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9, tt.translation.x, tt.translation.y, yaw_of(tt.rotation))
                if par == "odom" and ch == "base_link":
                    tf_ob.append(row)
                elif par == "map" and ch == "odom":
                    tf_mo.append(row)
        elif topic == "/map" and grid is None:
            grid = m
    f = lambda L: np.array(sorted(L), dtype=float) if L else np.zeros((0, 4))
    return f(wheel), f(ekf), f(tf_ob), f(tf_mo), grid


def load_map_yaml(path):
    y = yaml.safe_load(open(path))
    img = cv2.imread(os.path.join(os.path.dirname(path), y["image"]), cv2.IMREAD_GRAYSCALE)
    if y.get("negate", 0):
        img = 255 - img
    res, ox, oy = y["resolution"], y["origin"][0], y["origin"][1]
    return img, (ox, ox + img.shape[1] * res, oy, oy + img.shape[0] * res)


def map_from_grid(g):  # OccupancyGrid → map_server と同じ見た目 (0=白 254, 100=黒 0, -1=灰 205)
    d = np.array(g.data, dtype=np.int16).reshape(g.info.height, g.info.width)
    img = np.full(d.shape, 205, np.uint8)
    img[d == 0] = 254
    img[d > 50] = 0
    img = img[::-1]  # 行 0 を上 (y 最大) にする
    ox, oy, res = g.info.origin.position.x, g.info.origin.position.y, g.info.resolution
    return img, (ox, ox + g.info.width * res, oy, oy + g.info.height * res)


# ---------------------------------------------------------------- 評価
def errors(ref, P):
    Q = interp(P, ref[:, 0])
    exy = np.hypot(Q[:, 1] - ref[:, 1], Q[:, 2] - ref[:, 2])
    eyaw = np.degrees(np.vectorize(wrap)(Q[:, 3] - ref[:, 3]))
    return exy, eyaw


def yaw_minus_heading(P, win_s=1.0, v_min=0.3):
    """yaw と進行方向 (win_s 秒の変位の向き) の差の中央値 [rad]。前進中 (v > v_min) だけ使う。
    claude: 2026-10-15 — GLIM 基準 / AMCL の base_link yaw は進行方向から数度ずれている (extrinsic の yaw 誤差)。
    これを残すと開始時の yaw 合わせで odom 軌跡が丸ごと回り、EKF の誤差が実際より大きく見える。"""
    t = P[:, 0]
    j = np.searchsorted(t, t + win_s)
    ok = j < len(P)
    i, j = np.arange(len(P))[ok], j[ok]
    dx, dy = P[j, 1] - P[i, 1], P[j, 2] - P[i, 2]
    v = np.hypot(dx, dy) / np.maximum(t[j] - t[i], 1e-6)
    m = v > v_min
    if m.sum() < 10:
        return 0.0
    yaw_u = np.unwrap(P[:, 3])
    off = np.vectorize(wrap)((yaw_u[i][m] + yaw_u[j][m]) / 2 - np.arctan2(dy[m], dx[m]))
    return float(np.median(off))


def segment_drift(ref, P, lengths, step_m=2.0):
    """区間長 L ごとに、始点 i・終点 j (走行距離差 = L) の相対変位を ref と odom で比べる (KITTI 式)。"""
    dist = np.concatenate([[0], np.cumsum(np.hypot(np.diff(ref[:, 1]), np.diff(ref[:, 2])))])
    Q = interp(P, ref[:, 0])
    out = {}
    for L in lengths:
        tr, rot, sc = [], [], []
        for s0 in np.arange(0, dist[-1] - L, step_m):
            i = np.searchsorted(dist, s0)
            j = np.searchsorted(dist, s0 + L)
            if j >= len(ref):
                break
            rr = compose(inverse(tuple(ref[i, 1:])), tuple(ref[j, 1:]))
            qq = compose(inverse(tuple(Q[i, 1:])), tuple(Q[j, 1:]))
            tr.append(math.hypot(rr[0] - qq[0], rr[1] - qq[1]) / L * 100)
            rot.append(abs(math.degrees(wrap(rr[2] - qq[2]))) / L * 100)
            sc.append(math.hypot(*qq[:2]) / max(math.hypot(*rr[:2]), 1e-6))  # 弦長比 (odom / loc)
        out[L] = (np.array(tr), np.array(rot), np.array(sc))
    return out, dist


# ---------------------------------------------------------------- 作図
def draw_overlay(img, ext, ref, srcs, title, out, tick_s=120):
    xs = np.concatenate([ref[:, 1]] + [p[:, 1] for p in srcs.values()])
    ys = np.concatenate([ref[:, 2]] + [p[:, 2] for p in srcs.values()])
    m = 8.0
    x0, x1, y0, y1 = xs.min() - m, xs.max() + m, ys.min() - m, ys.max() + m
    res = (ext[1] - ext[0]) / img.shape[1]
    c0, c1 = int((x0 - ext[0]) / res), int((x1 - ext[0]) / res)
    r0, r1 = int((ext[3] - y1) / res), int((ext[3] - y0) / res)
    c0, r0 = max(c0, 0), max(r0, 0)
    c1, r1 = min(c1, img.shape[1]), min(r1, img.shape[0])
    crop = img[r0:r1, c0:c1]
    cext = (ext[0] + c0 * res, ext[0] + c1 * res, ext[3] - r1 * res, ext[3] - r0 * res)
    w, h = cext[1] - cext[0], cext[3] - cext[2]
    fig, ax = plt.subplots(figsize=(14, max(5.0, 14 * h / w + 1.2)), dpi=130)
    ax.imshow(crop, cmap="gray", extent=cext, vmin=0, vmax=255, interpolation="nearest", alpha=0.75)
    ax.plot(ref[:, 1], ref[:, 2], color=C_LOC, lw=2.0, label=f"loc: {title['loc']}", zorder=3)
    for (name, col), P in zip((("wheel", C_WHEEL), ("ekf", C_EKF)), srcs.values()):
        if P is None:
            continue
        ax.plot(P[:, 1], P[:, 2], color=col, lw=1.6, ls="--", label=f"odom only: {title[name]}", zorder=4)
        ax.plot(P[-1, 1], P[-1, 2], "s", color=col, ms=7, mec="white", mew=1.0, zorder=6)
    ax.plot(ref[0, 1], ref[0, 2], "o", color="#1a1a19", ms=8, mec="white", mew=1.2, zorder=7)
    ax.annotate("start", (ref[0, 1], ref[0, 2]), xytext=(6, 6), textcoords="offset points", fontsize=9)
    ax.plot(ref[-1, 1], ref[-1, 2], "s", color=C_LOC, ms=7, mec="white", mew=1.0, zorder=6)
    for tk in np.arange(tick_s, ref[-1, 0] - ref[0, 0], tick_s):
        k = np.searchsorted(ref[:, 0], ref[0, 0] + tk)
        ax.plot(ref[k, 1], ref[k, 2], ".", color=C_LOC, ms=5, zorder=5)
        ax.annotate(f"{int(tk)}s", (ref[k, 1], ref[k, 2]), xytext=(3, 3), textcoords="offset points", fontsize=7, color="#4b4b48")
    ax.set_aspect("equal")
    ax.set_xlim(cext[0], cext[1]); ax.set_ylim(cext[2], cext[3])
    ax.set_xlabel("x [m] (map)"); ax.set_ylabel("y [m] (map)")
    ax.set_title(title["fig"], fontsize=11)
    ax.legend(loc="best", fontsize=9, framealpha=0.9)
    ax.grid(False)
    fig.tight_layout(); fig.savefig(out); plt.close(fig)


def draw_drift(dist, errs, title, out):
    fig, axs = plt.subplots(2, 1, figsize=(11, 6.5), dpi=130, sharex=True)
    for (name, col), (exy, eyaw) in errs.items():
        axs[0].plot(dist, exy, color=col, lw=1.6, label=name)
        axs[1].plot(dist, eyaw, color=col, lw=1.6, label=name)
    axs[0].set_ylabel("position error [m]"); axs[1].set_ylabel("heading error [deg]")
    axs[1].set_xlabel("distance traveled (loc path) [m]")
    axs[0].set_title(title, fontsize=11)
    for a in axs:
        a.grid(True, color="#dddcd6", lw=0.6); a.legend(fontsize=9, loc="upper left")
        for sp in ("top", "right"):
            a.spines[sp].set_visible(False)
    fig.tight_layout(); fig.savefig(out); plt.close(fig)


# ---------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag"); ap.add_argument("out")
    ap.add_argument("--map", default="bag", help="map.yaml か 'bag' (/map を bag から)")
    ap.add_argument("--ref", default=None, help="GLIM 基準 tum (map 座標の base_link)。無ければ TF map→odom∘odom→base_link")
    ap.add_argument("--wheel", default="/odom"); ap.add_argument("--ekf", default="/odometry/filtered")
    ap.add_argument("--title", default="")
    ap.add_argument("--skip_s", type=float, default=0.0, help="先頭の除外秒 (初期化待ち)")
    ap.add_argument("--align_m", type=float, default=5.0, help="開始姿勢合わせの yaw を決める区間 [m] (yaw 差の中央値)")
    ap.add_argument("--keep_ref_yaw", action="store_true", help="基準 yaw の進行方向バイアスを除去しない")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)

    wheel, ekf, tf_ob, tf_mo, grid = read_bag(a.bag, a.wheel, a.ekf, a.map == "bag")
    if len(ekf) == 0 and len(tf_ob):
        ekf, ekf_name = tf_ob, "TF odom→base_link"
    else:
        ekf_name = a.ekf
    print(f"wheel {len(wheel)}  ekf {len(ekf)} ({ekf_name})  tf odom→base {len(tf_ob)}  tf map→odom {len(tf_mo)}")

    if a.ref:
        R = np.loadtxt(a.ref)
        ref = np.column_stack([R[:, 0], R[:, 1], R[:, 2],
                               [yaw_of(type("q", (), dict(x=q[0], y=q[1], z=q[2], w=q[3]))) for q in R[:, 4:8]]])
        loc_name = os.path.basename(a.ref)
    else:
        if len(tf_mo) == 0:
            raise SystemExit("TF map→odom が無い (自己位置推定の bag でない)。--ref を指定する")
        base = ekf if len(ekf) else wheel  # odom→base_link の供給源
        idx = np.searchsorted(tf_mo[:, 0], base[:, 0], side="right") - 1
        ok = idx >= 0
        base, idx = base[ok], idx[ok]
        ref = np.array([(b[0], *compose(tuple(tf_mo[i, 1:]), tuple(b[1:]))) for b, i in zip(base, idx)])
        loc_name = f"TF map→odom ({len(tf_mo)} msgs) ∘ odom→base_link"

    t0 = max(ref[0, 0], wheel[0, 0], ekf[0, 0] if len(ekf) else -np.inf) + a.skip_s
    t1 = min(ref[-1, 0], wheel[-1, 0], ekf[-1, 0] if len(ekf) else np.inf)
    ref = ref[(ref[:, 0] >= t0) & (ref[:, 0] <= t1)]
    bias = 0.0 if a.keep_ref_yaw else yaw_minus_heading(ref)
    ref[:, 3] = np.vectorize(wrap)(ref[:, 3] - bias)
    print(f"ref yaw - motion heading: {math.degrees(bias):+.2f} deg (removed)" if not a.keep_ref_yaw else "")
    dref = np.concatenate([[0], np.cumsum(np.hypot(np.diff(ref[:, 1]), np.diff(ref[:, 2])))])
    n_al = max(2, int(np.searchsorted(dref, a.align_m)))
    srcs, errs, segs = {}, {}, {}
    for name, col, P in (("wheel", C_WHEEL, wheel), ("ekf", C_EKF, ekf)):
        if len(P) == 0:
            srcs[name] = None; continue
        p0 = interp(P, [t0])[0]
        Q0 = interp(P, ref[:n_al, 0])
        dyaw = float(np.median(np.vectorize(wrap)(ref[:n_al, 3] - Q0[:, 3])))  # 最初の align_m [m] の yaw 差
        print(f"{name}: yaw offset vs loc over first {a.align_m} m = {math.degrees(dyaw):+.2f} deg, odom yaw - heading = {math.degrees(yaw_minus_heading(P)):+.2f} deg")
        T = compose((ref[0, 1], ref[0, 2], wrap(p0[3] + dyaw)), inverse(tuple(p0[1:])))
        Pm = apply(T, P[(P[:, 0] >= t0) & (P[:, 0] <= t1)])
        srcs[name] = Pm
        errs[(name, col)] = errors(ref, Pm)
        segs[name], dist = segment_drift(ref, Pm, (10, 50, 100))

    np.savez(os.path.join(a.out, "paths.npz"), ref=ref, **{k: v for k, v in srcs.items() if v is not None})
    img, ext = (load_map_yaml(a.map) if a.map != "bag" else map_from_grid(grid))
    titles = dict(loc=loc_name, wheel=a.wheel, ekf=ekf_name,
                  fig=f"{a.title or os.path.basename(a.bag)}  ({dist[-1]:.0f} m, {t1 - t0:.0f} s)")
    draw_overlay(img, ext, ref, srcs, titles, os.path.join(a.out, "overlay.png"))
    draw_drift(dist, errs, titles["fig"], os.path.join(a.out, "drift.png"))

    lines = [f"# odom drift — {os.path.basename(a.bag)}", "",
             f"loc = {loc_name}; 走行 {dist[-1]:.0f} m / {t1 - t0:.0f} s; 開始姿勢で 1 回だけ合わせた後は無補正", "",
             "| odom | 終点ずれ [m] | 終点 yaw ずれ [deg] | 位置誤差 中央/最大 [m] | 区間長 | 並進ドリフト 中央/p95 [%] | 回転ドリフト 中央/p95 [deg/100m] | 弦長比 odom/loc 中央 |",
             "|---|---|---|---|---|---|---|---|"]
    for (name, _), (exy, eyaw) in errs.items():
        first = True
        for L, (tr, rot, sc) in segs[name].items():
            if len(tr) == 0:
                continue
            head = (f"{name} | {exy[-1]:.2f} | {eyaw[-1]:+.1f} | {np.median(exy):.2f} / {exy.max():.2f}" if first else " | | | ")
            lines.append(f"| {head} | {L} m | {np.median(tr):.2f} / {np.percentile(tr, 95):.2f} | {np.median(rot):.2f} / {np.percentile(rot, 95):.2f} | {np.median(sc):.3f} |")
            first = False
    open(os.path.join(a.out, "segments.md"), "w").write("\n".join(lines) + "\n")
    print("\n".join(lines))


if __name__ == "__main__":
    main()
