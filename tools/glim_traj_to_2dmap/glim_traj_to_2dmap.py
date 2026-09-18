#!/usr/bin/env python3
# claude: 2026-09-14 作成
# bag の生点群 (間引き前) を GLIM dump の最適化済み軌跡 traj_lidar.txt で世界座標へ再投影し、
# Nav2 用 2D 占有格子 (map.pgm + map.yaml) を作る。
#
# 既存 glim_dump_to_2dmap (submap 点群を読む) との違い:
#   - 点の出どころが GLIM の submap (0.3 m ボクセル間引き済み) ではなく bag の生スキャン。
#     画素の濃さが「重なった submap 数」でなく壁に当たった点の物理密度になる。
#   - 姿勢は dump 根元の traj_lidar.txt (TUM 形式、1 行 = 1 スキャン、大域最適化後 =
#     ループクロージング反映済み)。GLIM が自分の地図を世界座標に置くときと同じ掛け算を
#     生点群に対して行うだけなので、GLIM の再実行は不要。
#   - 高さ帯の基準は既定でセンサ座標の z (--height_frame sensor)。実機の /scan
#     (pointcloud_to_laserscan, base_link 相対) と同じ切り方になり、traj 姿勢のピッチ誤差
#     (z ドーム斜面) が帯に混入しない。--height_frame world にすると「世界 z − 姿勢 z」で
#     切る (glim_dump_to_2dmap の sensor モードと同じ意味)。--height_frame ground は
#     スキャンごとに近傍点へ地面平面を最小二乗で当て、その平面からの高さで切る (2026-09-14
#     追加: センサの取付ロール/ピッチにも GLIM 姿勢の傾き誤差にも依存しない。5号館 08-14 bag
#     では取付ロール由来の「左 3 m の地面が帯に入る」筋を消すのに必要だった)。
#
# 依存: numpy + rosbag2_py/rclpy (ROS 2 Jazzy。glim_env / rerobot_env どちらでも可)。
#
# 使い方例 (glim_env 内):
#   source /opt/ros/jazzy/setup.bash
#   python3 /workspace/tools/glim_traj_to_2dmap/glim_traj_to_2dmap.py \
#       /workspace/bags/<bag_dir> /workspace/bags/<dump_dir>/traj_lidar.txt <dest_dir> \
#       -r 0.05 --map_width 6144 --map_height 6144 --min_height -0.45 --max_height 0.75
#   --floor_probe を付けると地図を書かずにセンサ座標の床 z (最頻値) だけ出す。

import argparse
import os
import sys
import time

import numpy as np

PC2_DTYPES = {1: "<i1", 2: "<u1", 3: "<i2", 4: "<u2", 5: "<i4", 6: "<u4", 7: "<f4", 8: "<f8"}


def load_traj(path):
    """TUM 形式 (stamp x y z qx qy qz qw) → (stamps[N], R[N,3,3], t[N,3])"""
    a = np.loadtxt(path)
    if a.ndim == 1:
        a = a[None, :]
    st = a[:, 0]
    t = a[:, 1:4]
    q = a[:, 4:8]  # x y z w
    x, y, z, w = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    R = np.empty((len(a), 3, 3))
    R[:, 0, 0] = 1 - 2 * (y * y + z * z)
    R[:, 0, 1] = 2 * (x * y - z * w)
    R[:, 0, 2] = 2 * (x * z + y * w)
    R[:, 1, 0] = 2 * (x * y + z * w)
    R[:, 1, 1] = 1 - 2 * (x * x + z * z)
    R[:, 1, 2] = 2 * (y * z - x * w)
    R[:, 2, 0] = 2 * (x * z - y * w)
    R[:, 2, 1] = 2 * (y * z + x * w)
    R[:, 2, 2] = 1 - 2 * (x * x + y * y)
    order = np.argsort(st)
    return st[order], R[order], t[order], q[order]


def slerp(q0, q1, u):
    """単位四元数 (x y z w) の球面線形補間。u: (M,) → (M,4)"""
    d = np.dot(q0, q1)
    if d < 0:
        q1 = -q1
        d = -d
    if d > 0.9995:
        out = q0[None, :] + u[:, None] * (q1 - q0)[None, :]
        return out / np.linalg.norm(out, axis=1, keepdims=True)
    th = np.arccos(d)
    s = np.sin(th)
    return (np.sin((1 - u) * th) / s)[:, None] * q0[None, :] + (np.sin(u * th) / s)[:, None] * q1[None, :]


def quat_rotate(q, v):
    """q: (M,4) x y z w, v: (M,3) → (M,3)"""
    x, y, z, w = q[:, 0:1], q[:, 1:2], q[:, 2:3], q[:, 3:4]
    uv = np.cross(q[:, :3], v)
    uuv = np.cross(q[:, :3], uv)
    return v + 2 * (w * uv + uuv)


def fit_ground_plane(p, z0, rmax, iters=2, tol=0.12):
    """センサ座標の点 p (N,3) から地面平面 z = a x + b y + c を当てる。z0 = 床 z の初期値。
    返り値 (a, b, c, n_used)。点が足りなければ水平面 (0, 0, z0) にフォールバック"""
    rng = np.hypot(p[:, 0], p[:, 1])
    m = (rng < rmax) & (np.abs(p[:, 2] - z0) < 0.3)
    a = b = 0.0
    c = z0
    for _ in range(iters + 1):
        q = p[m]
        if len(q) < 200:
            return 0.0, 0.0, z0, int(m.sum())
        A = np.column_stack([q[:, 0], q[:, 1], np.ones(len(q))])
        (a, b, c), *_ = np.linalg.lstsq(A, q[:, 2], rcond=None)
        resid = p[:, 2] - (a * p[:, 0] + b * p[:, 1] + c)
        m = (rng < rmax) & (np.abs(resid) < tol)
    return float(a), float(b), float(c), int(m.sum())


def cloud_to_numpy(msg):
    dt = np.dtype({
        "names": [f.name for f in msg.fields],
        "formats": [PC2_DTYPES[f.datatype] for f in msg.fields],
        "offsets": [f.offset for f in msg.fields],
        "itemsize": msg.point_step,
    })
    return np.frombuffer(msg.data, dtype=dt)


def open_bag(bag_dir, topic):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag_dir, storage_id=""),
                rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if topic not in types:
        raise SystemExit(f"topic {topic} not in bag. available: {list(types)}")
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    msg_type = get_message(types[topic])

    def gen():
        while reader.has_next():
            _, data, _ = reader.read_next()
            yield deserialize_message(data, msg_type)
    return gen()


def main():
    ap = argparse.ArgumentParser(description="bag raw scans x GLIM optimized trajectory -> Nav2 2D occupancy grid")
    ap.add_argument("bag_dir")
    ap.add_argument("traj", help="GLIM dump の traj_lidar.txt (TUM)。dump ディレクトリを渡すと中の traj_lidar.txt を使う")
    ap.add_argument("dest_dir")
    ap.add_argument("--topic", default="/rfans_driver/rfans_points")
    ap.add_argument("-r", "--resolution", type=float, default=0.05, help="m / pixel")
    ap.add_argument("--map_width", type=int, default=0, help="pixels; 0 = auto (要 2 パス。大きい bag では明示推奨)")
    ap.add_argument("--map_height", type=int, default=0)
    ap.add_argument("--center", choices=["world", "auto"], default="world",
                    help="world: 既製ツール互換 (world 原点中心) / auto: 軌跡 bbox 中心")
    ap.add_argument("--height_frame", choices=["sensor", "world", "ground"], default="sensor",
                    help="sensor: センサ座標の z で切る (実機 /scan と同じ) / world: 世界 z − 姿勢 z で切る / "
                         "ground: スキャンごとに近傍点へ地面平面を当て、その平面からの高さで切る (min/max_height は地上高)")
    ap.add_argument("--ground_z", type=float, default=None,
                    help="ground モードの地面探索の初期値 (センサ座標の床 z)。省略時は先頭 50 スキャンの最頻値から自動推定")
    ap.add_argument("--ground_range", type=float, default=12.0, help="ground モードで平面当てに使う水平距離 [m]")
    ap.add_argument("--min_height", type=float, default=-0.45)
    ap.add_argument("--max_height", type=float, default=0.75)
    ap.add_argument("--range_min", type=float, default=0.5, help="これ未満の点 (車体・無効点) を捨てる [m]")
    ap.add_argument("--range_max", type=float, default=40.0, help="これ超の点を捨てる [m]")
    ap.add_argument("--min_points_in_pix", type=int, default=2)
    ap.add_argument("--max_points_in_pix", type=int, default=5)
    ap.add_argument("--stamp_tol", type=float, default=0.02, help="header stamp と traj stamp の許容差 [s]")
    ap.add_argument("--deskew", action="store_true",
                    help="点ごとの time フィールドで前後スキャン姿勢を補間 (slerp) して動き補正する")
    ap.add_argument("--skip", type=int, default=1, help="N スキャンに 1 つだけ使う (速度試験用)")
    ap.add_argument("--floor_probe", action="store_true",
                    help="地図を書かず、センサ座標 z のヒストグラム最頻値 (床) を出して終わる")
    ap.add_argument("--save_counts", default=None, help="画素点数配列 (.npy) を保存する (解析用)")
    args = ap.parse_args()

    traj_path = args.traj
    if os.path.isdir(traj_path):
        traj_path = os.path.join(traj_path, "traj_lidar.txt")
    st, R, t, q = load_traj(traj_path)
    print(f"traj: {len(st)} poses, {st[0]:.3f} .. {st[-1]:.3f} ({st[-1]-st[0]:.0f} s)")

    res = args.resolution
    if args.center == "world":
        cx, cy = 0.0, 0.0
    else:
        cx = (t[:, 0].min() + t[:, 0].max()) / 2
        cy = (t[:, 1].min() + t[:, 1].max()) / 2
    W, H = args.map_width, args.map_height
    if W == 0 or H == 0:
        # 軌跡 bbox + range_max 余白で自動決定 (64 px 単位)
        need_w = int(np.ceil((2 * np.abs(t[:, 0] - cx).max() + 2 * args.range_max + 4.0) / res / 64) * 64)
        need_h = int(np.ceil((2 * np.abs(t[:, 1] - cy).max() + 2 * args.range_max + 4.0) / res / 64) * 64)
        W = W or need_w
        H = H or need_h
    print(f"map: {W}x{H} px @ {res} m/px, center=({cx:.2f},{cy:.2f}), height_frame={args.height_frame} "
          f"band [{args.min_height},{args.max_height}]")
    counts = np.zeros(W * H, dtype=np.int32)

    n_msg = n_used = n_nomatch = 0
    dts = []
    n_pts_in = n_pts_band = 0
    floor_hist = np.zeros(int(2.5 / 0.02), dtype=np.int64)  # z in [-2.0, 0.5)
    ground_z = args.ground_z
    ground_hist = np.zeros_like(floor_hist)
    ground_tilt = []  # (a, b) per scan in ground mode
    t0 = time.time()
    for msg in open_bag(args.bag_dir, args.topic):
        n_msg += 1
        if (n_msg - 1) % args.skip:
            continue
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        k = int(np.searchsorted(st, stamp))
        cands = [i for i in (k - 1, k) if 0 <= i < len(st)]
        k = min(cands, key=lambda i: abs(st[i] - stamp))
        dt = st[k] - stamp
        if abs(dt) > args.stamp_tol:
            n_nomatch += 1
            continue
        dts.append(dt)
        a = cloud_to_numpy(msg)
        p = np.stack([a["x"], a["y"], a["z"]], axis=1).astype(np.float64)
        rng = np.hypot(p[:, 0], p[:, 1])
        ok = np.isfinite(p).all(axis=1) & (rng >= args.range_min) & (rng <= args.range_max)
        p = p[ok]
        n_pts_in += len(p)
        if args.floor_probe:
            z = p[:, 2]
            zz = z[(z >= -2.0) & (z < 0.5)]
            floor_hist += np.bincount(((zz + 2.0) / 0.02).astype(np.int64), minlength=len(floor_hist))[:len(floor_hist)]
            n_used += 1
            continue

        if args.height_frame == "ground":
            if ground_z is None:
                # 先頭 50 スキャンで床 z を自動推定 (それまでのスキャンは捨てる)
                z = p[:, 2]
                zz = z[(z >= -2.0) & (z < 0.5)]
                ground_hist += np.bincount(((zz + 2.0) / 0.02).astype(np.int64), minlength=len(ground_hist))[:len(ground_hist)]
                if len(dts) >= 50:
                    ground_z = -2.0 + 0.02 * int(ground_hist.argmax()) + 0.01
                    print(f"ground_z auto-estimated from first 50 scans: {ground_z:+.3f} m")
                continue
            a_, b_, c_, nfit = fit_ground_plane(p, ground_z, args.ground_range)
            ground_tilt.append((a_, b_, nfit))
            h = p[:, 2] - (a_ * p[:, 0] + b_ * p[:, 1] + c_)   # 地上高
            keep = (h >= args.min_height) & (h <= args.max_height)
            p = p[keep]
            if args.deskew and "time" in a.dtype.names and k + 1 < len(st):
                tt = a["time"][ok][keep].astype(np.float64)
                u = np.clip((stamp + tt - st[k]) / max(st[k + 1] - st[k], 1e-6), 0.0, 1.0)
                qi = slerp(q[k], q[k + 1], u)
                w = quat_rotate(qi, p) + t[k][None, :] + u[:, None] * (t[k + 1] - t[k])[None, :]
            else:
                w = p @ R[k].T + t[k]
        elif args.height_frame == "sensor":
            keep = (p[:, 2] >= args.min_height) & (p[:, 2] <= args.max_height)
            p = p[keep]
            if args.deskew and "time" in a.dtype.names and k + 1 < len(st):
                tt = a["time"][ok][keep].astype(np.float64)
                u = np.clip((stamp + tt - st[k]) / max(st[k + 1] - st[k], 1e-6), 0.0, 1.0)
                qi = slerp(q[k], q[k + 1], u)
                w = quat_rotate(qi, p) + t[k][None, :] + u[:, None] * (t[k + 1] - t[k])[None, :]
            else:
                w = p @ R[k].T + t[k]
        else:
            if args.deskew and "time" in a.dtype.names and k + 1 < len(st):
                tt = a["time"][ok].astype(np.float64)
                u = np.clip((stamp + tt - st[k]) / max(st[k + 1] - st[k], 1e-6), 0.0, 1.0)
                qi = slerp(q[k], q[k + 1], u)
                w = quat_rotate(qi, p) + t[k][None, :] + u[:, None] * (t[k + 1] - t[k])[None, :]
                zref = t[k, 2] + u * (t[k + 1, 2] - t[k, 2])
            else:
                w = p @ R[k].T + t[k]
                zref = t[k, 2]
            rel = w[:, 2] - zref
            keep = (rel >= args.min_height) & (rel <= args.max_height)
            w = w[keep]
        n_pts_band += len(w)
        px = np.floor((w[:, 0] - cx) / res).astype(np.int64) + W // 2
        py = np.floor(-(w[:, 1] - cy) / res).astype(np.int64) + H // 2
        inside = (px >= 0) & (px < W) & (py >= 0) & (py < H)
        # claude: 出現した画素だけ加算する (bincount(minlength=W*H) は毎スキャン 6144^2 の配列を
        # 確保して 1 スキャン 0.1 s 以上かかる — 2026-09-14 実測で全体 19 分 → この方式で数分)
        idx, cnt = np.unique(py[inside] * W + px[inside], return_counts=True)
        counts[idx] += cnt.astype(np.int32)
        n_used += 1
        if n_used % 1000 == 0:
            print(f"  {n_used} scans, {time.time()-t0:.0f} s", flush=True)

    dts = np.array(dts) if dts else np.zeros(1)
    print(f"scans: {n_msg} in bag, {n_used} used, {n_nomatch} without traj match (|dt|>{args.stamp_tol}s); "
          f"stamp diff median {1e3*np.median(dts):+.2f} ms, max {1e3*np.abs(dts).max():.2f} ms; "
          f"points: {n_pts_in} in range, {n_pts_band} in height band; {time.time()-t0:.0f} s")

    if ground_tilt:
        g = np.array(ground_tilt)
        roll = np.degrees(np.arctan(g[:, 1]))   # 左右方向の勾配 (y)
        pitch = np.degrees(np.arctan(g[:, 0]))  # 前後方向の勾配 (x)
        print(f"ground plane (sensor frame): tilt about x-axis (y-slope) median {np.median(roll):+.2f} deg "
              f"[p5 {np.percentile(roll,5):+.2f}, p95 {np.percentile(roll,95):+.2f}], "
              f"y-axis (x-slope) median {np.median(pitch):+.2f} deg [p5 {np.percentile(pitch,5):+.2f}, p95 {np.percentile(pitch,95):+.2f}], "
              f"fit points median {int(np.median(g[:,2]))}, scans with fallback {(g[:,2] < 200).sum()}")

    if args.floor_probe:
        e = np.arange(-2.0, 0.5 + 1e-9, 0.02)
        i = int(floor_hist.argmax())
        print(f"floor (sensor z) mode = {e[i]+0.01:+.3f} m  (band 'floor+0.3..+1.5' -> "
              f"--min_height {e[i]+0.01+0.3:+.2f} --max_height {e[i]+0.01+1.5:+.2f})")
        return 0

    counts = counts.reshape(H, W)
    lo, hi = args.min_points_in_pix, args.max_points_in_pix
    img = np.clip(255.0 - 255.0 * (counts - lo) / (hi - lo), 0, 255).astype(np.uint8)
    os.makedirs(args.dest_dir, exist_ok=True)
    if args.save_counts:
        np.save(args.save_counts, counts)
    pgm = os.path.join(args.dest_dir, "map.pgm")
    with open(pgm, "wb") as f:
        f.write(f"P5\n{W} {H}\n255\n".encode())
        f.write(img.tobytes())
    origin_x = cx - res * W / 2
    origin_y = cy - res * H / 2
    with open(os.path.join(args.dest_dir, "map.yaml"), "w") as f:
        f.write(f"image: map.pgm\nresolution: {res}\n"
                f"origin: [{origin_x}, {origin_y}, 0.0]\n"
                "occupied_thresh: 0.5\nfree_thresh: 0.2\nnegate: 0\n")
    occ = int((img < 128).sum())
    print(f"wrote {pgm} and map.yaml (origin=[{origin_x:.2f},{origin_y:.2f}]); occupied px {occ} = {occ*res*res:.1f} m^2")
    return 0


if __name__ == "__main__":
    sys.exit(main())
