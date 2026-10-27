#!/usr/bin/env python3
# claude: 2026-10-24 新設 — 既存 bag の R-Fans 点群に「擬似人垣」を注入した bag を作る (main コンテナ内で実行)。
#
#   つくば本番の最悪想定「スタート地点の 10〜20 m に人が密集し、前方以外の LiDAR 視界が遮られる」を、
#   人垣なしで録った bag (2026-10-03 seg1) から再現する。人は地図座標に固定した円柱 (半径 0.25 m、高さ 1.75 m) で、
#   各フレームごとに LiDAR 位置 (GLIM の traj_lidar.txt = 地図座標) からレイキャストし、人に当たる点を人の表面へ
#   引き寄せる。人の頭より上を通る上向きリングの点はそのまま残る (3D 自己位置推定が頭越しに建物を見る効果を再現)。
#
#   使い方:
#     python3 crowd_inject.py <src_bag> <dst_bag> <traj_lidar.txt> [--dur 300] [--r_min 10 --r_max 20]
#         [--front_half_deg 45] [--density 0.5] [--seed 0] [--no-crowd]
#   --no-crowd は同じ時間窓・同じ topic だけの素の bag を書く (比較の基準用。同じ書き出し経路を通す)。
#   出力: <dst_bag>/ (mcap) + <dst_bag>_crowd.json (人の配置・統計) + <dst_bag>_frame.png (1 フレームの上面図、確認用)
import argparse
import json
import math
import os
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import PointCloud2

TOPICS = ["/tf", "/tf_static", "/rfans_driver/rfans_points", "/imu/data", "/odom"]
LIDAR_Z = 0.79396          # base_link→rfans の z (bag 内 /tf_static と同じ)
PERSON_R = 0.25            # 人の半径 [m]
PERSON_H = 1.75            # 人の高さ [m] (地面から)
AZ_BINS = 3600             # レイキャストの方位分割 (0.1°)


def quat_to_rot(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class Traj:
    def __init__(self, path):
        a = np.loadtxt(path)
        self.t = a[:, 0]
        self.p = a[:, 1:4]
        self.q = a[:, 4:8]

    def pose_at(self, st):
        k = int(np.clip(np.searchsorted(self.t, st), 1, len(self.t) - 1))
        if abs(self.t[k - 1] - st) < abs(self.t[k] - st):
            k -= 1
        dt = abs(self.t[k] - st)
        if dt <= 0.03:
            return self.p[k], quat_to_rot(self.q[k]), dt
        # 線形補間 (位置) + 近い方の姿勢
        k0 = int(np.clip(np.searchsorted(self.t, st) - 1, 0, len(self.t) - 2))
        a = (st - self.t[k0]) / max(self.t[k0 + 1] - self.t[k0], 1e-6)
        a = float(np.clip(a, 0.0, 1.0))
        p = (1 - a) * self.p[k0] + a * self.p[k0 + 1]
        return p, quat_to_rot(self.q[k0 if a < 0.5 else k0 + 1]), dt


class MapAvoid:
    """claude 10-24 critic: 一様配置だと人の 87 % が地図障害物 (植栽・ベンチ) の 2 m 以内に落ち、終点が地図に整合してしまう。
    map.yaml の占有画素から avoid_dist 以内には置かない"""
    def __init__(self, map_yaml, avoid_dist):
        import yaml
        cfg = yaml.safe_load(open(map_yaml))
        pgm = os.path.join(os.path.dirname(map_yaml), cfg["image"])
        f = open(pgm, "rb"); f.readline(); toks = []
        while len(toks) < 3:
            line = f.readline()
            if line.startswith(b"#"):
                continue
            toks += line.split()
        self.W, self.H = int(toks[0]), int(toks[1]); off = f.tell()
        self.img = np.memmap(pgm, dtype=np.uint8, mode="r", offset=off, shape=(self.H, self.W))
        self.res = float(cfg["resolution"]); self.ox, self.oy = float(cfg["origin"][0]), float(cfg["origin"][1])
        self.r = int(math.ceil(avoid_dist / self.res))

    def ok(self, x, y):
        u = int((x - self.ox) / self.res); v = self.H - 1 - int((y - self.oy) / self.res)
        if not (self.r <= u < self.W - self.r and self.r <= v < self.H - self.r):
            return False
        patch = np.array(self.img[v - self.r:v + self.r + 1, u - self.r:u + self.r + 1])
        return not (patch < 100).any()    # 占有 (黒) 画素が無い


def place_people(center, yaw0, r_min, r_max, front_half, density, seed, avoid=None):
    """前方 ±front_half 以外の環状領域に、最小間隔 0.6 m で人を置く (地図座標)"""
    rng = np.random.default_rng(seed)
    area = math.pi * (r_max ** 2 - r_min ** 2) * (1 - 2 * front_half / (2 * math.pi))
    n_target = int(round(area * density))
    pts = []
    tries = 0
    while len(pts) < n_target and tries < n_target * 50:
        tries += 1
        r = math.sqrt(rng.uniform(r_min ** 2, r_max ** 2))
        th = rng.uniform(-math.pi, math.pi)
        if abs(th) < front_half:          # 前方は空ける (yaw0 基準)
            continue
        x = center[0] + r * math.cos(yaw0 + th)
        y = center[1] + r * math.sin(yaw0 + th)
        if avoid is not None and not avoid.ok(x, y):
            continue
        if pts:
            d = np.hypot(np.array(pts)[:, 0] - x, np.array(pts)[:, 1] - y)
            if d.min() < 2 * PERSON_R + 0.1:
                continue
        pts.append((x, y))
    return np.array(pts), area, n_target


def crowd_profile(people, sensor_xy):
    """センサ位置から見た方位ビンごとの最近の人の表面距離 (inf = 人なし)"""
    prof = np.full(AZ_BINS, np.inf)
    dx = people[:, 0] - sensor_xy[0]
    dy = people[:, 1] - sensor_xy[1]
    dc = np.hypot(dx, dy)
    ang = np.arctan2(dy, dx)
    order = np.argsort(-dc)             # 遠い人から書き、近い人で上書き
    step = 2 * math.pi / AZ_BINS
    for i in order:
        if dc[i] <= PERSON_R + 0.05:
            continue
        hw = math.asin(PERSON_R / dc[i])
        b0 = int(math.floor((ang[i] - hw + math.pi) / step))
        b1 = int(math.floor((ang[i] + hw + math.pi) / step))
        bins = np.arange(b0, b1 + 1) % AZ_BINS
        prof[bins] = np.minimum(prof[bins], dc[i] - PERSON_R)
    return prof


def inject(cloud: PointCloud2, pos, R, prof, ground_z):
    """点群 (センサ座標) の人に当たる点を人の表面へ引き寄せる。戻り値 = (新 data bytes, 置換点数, 有効点数)"""
    step = cloud.point_step
    buf = np.frombuffer(bytes(cloud.data), dtype=np.uint8).reshape(-1, step).copy()
    xyz = np.ascontiguousarray(buf[:, 0:12]).view(np.float32).reshape(-1, 3)
    valid = np.isfinite(xyz).all(axis=1) & (np.abs(xyz).sum(axis=1) > 0)
    p = xyz[valid].astype(np.float64)
    w = p @ R.T + pos                     # 地図座標
    hx, hy = w[:, 0] - pos[0], w[:, 1] - pos[1]
    rng_xy = np.hypot(hx, hy)
    ok = rng_xy > 0.3
    az = np.arctan2(hy, hx)
    bins = np.floor((az + math.pi) / (2 * math.pi / AZ_BINS)).astype(int) % AZ_BINS
    dc = prof[bins]
    hit = ok & (dc < rng_xy)
    # 光線上で人の位置に来たときの高さ
    s = np.where(rng_xy > 0, dc / np.maximum(rng_xy, 1e-6), 0.0)
    zc = pos[2] + (w[:, 2] - pos[2]) * s
    hit &= (zc >= ground_z - 0.05) & (zc <= ground_z + PERSON_H)
    if hit.any():
        p_new = p.copy()
        p_new[hit] *= s[hit][:, None]     # センサ原点からの光線なので比例縮小でよい
        xyz_v = xyz.copy()
        xyz_v[valid] = p_new.astype(np.float32)
        buf[:, 0:12] = xyz_v.view(np.uint8).reshape(-1, 12)
    return buf.tobytes(), int(hit.sum()), int(valid.sum())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("src")
    ap.add_argument("dst")
    ap.add_argument("traj")
    ap.add_argument("--dur", type=float, default=300.0, help="bag 先頭から書き出す秒数")
    ap.add_argument("--r_min", type=float, default=10.0)
    ap.add_argument("--r_max", type=float, default=20.0)
    ap.add_argument("--front_half_deg", type=float, default=45.0, help="空けておく前方扇の半角 [deg]")
    ap.add_argument("--density", type=float, default=0.5, help="人/m^2")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--no-crowd", action="store_true", help="注入なし (同じ切り出しの基準 bag)")
    ap.add_argument("--avoid_map", default=None, help="map.yaml: 占有画素から --avoid_dist 以内には人を置かない (開けた場所だけ)")
    ap.add_argument("--avoid_dist", type=float, default=1.5)
    ap.add_argument("--png_at", type=float, default=5.0, help="確認図を描くフレームの時刻 [s, bag 先頭から]")
    args = ap.parse_args()

    traj = Traj(args.traj)
    p0, R0, _ = traj.pose_at(traj.t[0])
    yaw0 = math.atan2(R0[1, 0], R0[0, 0])
    ground_z = p0[2] - LIDAR_Z
    avoid = MapAvoid(args.avoid_map, args.avoid_dist) if args.avoid_map else None
    people, area, n_target = place_people(p0[:2], yaw0, args.r_min, args.r_max,
                                         math.radians(args.front_half_deg), args.density, args.seed, avoid)
    if args.no_crowd:
        people = np.zeros((0, 2))
    print(f"start ({p0[0]:.2f},{p0[1]:.2f}) yaw {math.degrees(yaw0):.1f} deg, ground z {ground_z:.3f}; "
          f"crowd area {area:.0f} m^2 -> {len(people)} people (target {n_target})")

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=args.src, storage_id="mcap"), rosbag2_py.ConverterOptions("", ""))
    topics = {t.name: t for t in reader.get_all_topics_and_types()}
    reader.set_filter(rosbag2_py.StorageFilter(topics=TOPICS))
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=args.dst, storage_id="mcap"), rosbag2_py.ConverterOptions("", ""))
    for name in TOPICS:
        if name in topics:
            writer.create_topic(topics[name])

    t_end = None
    n_frames = n_hit_frames = 0
    hit_frac = []
    dt_max = 0.0
    png_done = args.no_crowd
    while reader.has_next():
        topic, data, ts = reader.read_next()
        if t_end is None:
            t_end = ts + int(args.dur * 1e9)
        if ts > t_end:
            break
        if topic == "/rfans_driver/rfans_points" and len(people):
            msg = deserialize_message(data, PointCloud2)
            st = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            pos, R, dt = traj.pose_at(st)
            dt_max = max(dt_max, dt)
            prof = crowd_profile(people, pos[:2])
            new_data, nh, nv = inject(msg, pos, R, prof, ground_z)
            n_frames += 1
            if nh:
                n_hit_frames += 1
            hit_frac.append(nh / max(nv, 1))
            if not png_done and st - traj.t[0] >= args.png_at:
                draw_frame(msg, new_data, pos, R, people, args.dst + "_frame.png")
                png_done = True
            msg.data = new_data
            data = serialize_message(msg)
        writer.write(topic, data, ts)
    del writer
    hf = np.array(hit_frac) if hit_frac else np.zeros(1)
    stats = {
        "src": args.src, "dst": args.dst, "dur_s": args.dur, "no_crowd": args.no_crowd,
        "start_xy": [float(p0[0]), float(p0[1])], "start_yaw_deg": math.degrees(yaw0), "ground_z": float(ground_z),
        "r_min": args.r_min, "r_max": args.r_max, "front_half_deg": args.front_half_deg, "density": args.density,
        "seed": args.seed, "avoid_map": args.avoid_map, "avoid_dist": args.avoid_dist, "n_people": int(len(people)), "person_r": PERSON_R, "person_h": PERSON_H,
        "frames": n_frames, "frames_with_hit": n_hit_frames,
        "replaced_frac_first30s": float(hf[:300].mean()), "replaced_frac_max": float(hf.max()),
        "traj_dt_max": dt_max, "people": people.round(3).tolist(),
    }
    with open(args.dst + "_crowd.json", "w") as f:
        json.dump(stats, f, indent=1)
    print(f"frames {n_frames}, with hit {n_hit_frames}, replaced frac first 30 s {stats['replaced_frac_first30s']:.3f} "
          f"max {stats['replaced_frac_max']:.3f}, traj dt max {dt_max:.3f} s")


def draw_frame(msg, new_data, pos, R, people, path):
    """1 フレームの上面図 (元点群 = 灰、注入後 = 黒、人 = 赤丸)。確認用"""
    from PIL import Image, ImageDraw
    step = msg.point_step
    def xyz_of(b):
        a = np.frombuffer(b, dtype=np.uint8).reshape(-1, step)
        return np.ascontiguousarray(a[:, 0:12]).view(np.float32).reshape(-1, 3)
    S = 1000; R_m = 30.0
    im = Image.new("RGB", (S, S), (255, 255, 255)); d = ImageDraw.Draw(im)
    def px(w):
        return ((w[:, 0] - pos[0]) / R_m * (S / 2) + S / 2, S / 2 - (w[:, 1] - pos[1]) / R_m * (S / 2))
    for b, col in ((bytes(msg.data), (190, 190, 190)), (new_data, (0, 0, 0))):
        xyz = xyz_of(b).astype(np.float64)
        v = np.isfinite(xyz).all(axis=1) & (np.abs(xyz).sum(axis=1) > 0)
        w = xyz[v] @ R.T + pos
        u, vv = px(w)
        m = (u >= 0) & (u < S) & (vv >= 0) & (vv < S)
        for a, b2 in zip(u[m].astype(int), vv[m].astype(int)):
            d.point((a, b2), fill=col)
    if len(people):
        u, vv = px(np.column_stack([people, np.zeros(len(people))]))
        for a, b2 in zip(u, vv):
            d.ellipse([a - 4, b2 - 4, a + 4, b2 + 4], outline=(255, 0, 0))
    d.ellipse([S / 2 - 5, S / 2 - 5, S / 2 + 5, S / 2 + 5], fill=(0, 0, 255))
    im.save(path)


if __name__ == "__main__":
    main()
