#!/usr/bin/env python3
# claude: 2026-08-20 作成
# GLIM dump (submap 群) を直接読んで Nav2 用 2D 占有格子 (map.pgm + map.yaml) を作る。
#
# 既製 pointcloud_to_2dmap との違い:
#   - 入力がマージ済み PCD ではなく GLIM dump そのもの。各点が「どの submap
#     (= どのセンサ姿勢) から観測されたか」の対応が残っているので、高さスライスを
#     世界座標の絶対 z ではなく「その submap のセンサ z からの相対高さ」で行える
#     (--height_mode sensor, 既定)。地図全体で z がドリフトしていても、スライス帯が
#     センサと一緒に上下するため場所によらず同じ高さ帯の壁が取れる。
#   - --export_pcd で全点をマージした世界座標 PCD も書ける (既製ツールとの比較用)。
#
# 依存: numpy のみ (glim / rerobot_env コンテナに導入済み)。出力は PGM
# (map_server は png/pgm どちらも可)。
#
# 使い方例:
#   python3 glim_dump_to_2dmap.py <dump_dir> <dest_dir> \
#       -r 0.05 --map_width 6144 --map_height 6144 \
#       --min_height -0.5 --max_height 0.7        # センサ z 基準の帯

import argparse
import os
import re
import sys

import numpy as np


def load_submap(dump_dir, idx):
    """submap の T_world_origin (4x4) と点群 (N,3 float32, submap 原点フレーム) を返す"""
    d = os.path.join(dump_dir, f"{idx:06d}")
    txt = open(os.path.join(d, "data.txt")).read()
    m = re.search(r"T_world_origin:\s*\n((?:.+\n){4})", txt)
    if m is None:
        raise RuntimeError(f"T_world_origin not found in {d}/data.txt")
    T = np.array([[float(v) for v in line.split()] for line in m.group(1).strip().split("\n")])
    pts = np.fromfile(os.path.join(d, "points_compact.bin"), dtype=np.float32).reshape(-1, 3)
    return T, pts


def load_viewpoints(dump_dir, idx):
    """claude: 2026-10-02 submap 内の各スキャンのセンサ位置 (T_world_lidar の並進, (F,3)) を返す"""
    txt = open(os.path.join(dump_dir, f"{idx:06d}", "data.txt")).read()
    blocks = re.findall(r"T_world_lidar:\s*\n((?:.+\n){4})", txt)
    return np.array([[float(line.split()[3]) for line in b.strip().split("\n")[:3]] for b in blocks])


def add_free_fan(free, sx, sy, wx, wy, obst, cx, cy, res, rmax, n_bins):
    """claude: 2026-10-02 センサ (sx,sy) から見えた範囲を空きとして free (H,W uint8) に塗る。
    方位を n_bins に分け、各方位で「帯内の点 (障害物) の最近距離」と「全点の最遠距離」の小さい方
    までを空きとする (= 光線が障害物で止まる。帯の外の点 = 地面などは「そこまで何も無かった」証拠)。
    光線を 1 本ずつ引く代わりに、方位ごとの端点をつないだ星形多角形を塗る"""
    import cv2
    H, W = free.shape
    dx, dy = wx - sx, wy - sy
    r = np.hypot(dx, dy)
    m = (r > 0.05) & (r <= rmax)
    if not m.any():
        return
    b = ((np.arctan2(dy[m], dx[m]) + np.pi) / (2 * np.pi) * n_bins).astype(np.int64) % n_bins
    rm = r[m]
    rfar = np.zeros(n_bins)
    np.maximum.at(rfar, b, rm)
    robs = np.full(n_bins, np.inf)
    om = obst[m]
    np.minimum.at(robs, b[om], rm[om])
    rf = np.minimum(rfar, robs)
    ang = (np.arange(n_bins) + 0.5) / n_bins * 2 * np.pi - np.pi
    ex = sx + rf * np.cos(ang)
    ey = sy + rf * np.sin(ang)
    # 画素系は地図本体と同じ (x 右, 画像行は -y)。shift=2 で 1/4 画素精度
    px = ((ex - cx) / res + W // 2) * 4
    py = (-(ey - cy) / res + H // 2) * 4
    cv2.fillPoly(free, [np.stack([px, py], axis=1).round().astype(np.int32)], 1, lineType=cv2.LINE_8, shift=2)


def list_submaps(dump_dir):
    ids = sorted(int(n) for n in os.listdir(dump_dir) if n.isdigit() and len(n) == 6)
    if not ids:
        raise RuntimeError(f"no submap directories under {dump_dir}")
    return ids


def write_pcd(path, points):
    """世界座標の全点を binary PCD (x y z float32) で書き出す"""
    n = len(points)
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
        f"WIDTH {n}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {n}\nDATA binary\n"
    )
    with open(path, "wb") as f:
        f.write(header.encode())
        f.write(points.astype(np.float32).tobytes())


def main():
    ap = argparse.ArgumentParser(description="GLIM dump -> Nav2 2D occupancy grid")
    ap.add_argument("dump_dir")
    ap.add_argument("dest_dir")
    ap.add_argument("-r", "--resolution", type=float, default=0.05, help="m / pixel")
    ap.add_argument("--map_width", type=int, default=0, help="pixels; 0 = auto fit to points")
    ap.add_argument("--map_height", type=int, default=0, help="pixels; 0 = auto fit to points")
    ap.add_argument("--center", choices=["world", "auto"], default="world",
                    help="world: 既製ツール互換 (world 原点中心) / auto: 点群 bbox 中心")
    ap.add_argument("--height_mode", choices=["sensor", "absolute"], default="sensor",
                    help="sensor: submap センサ z 基準の相対高さ / absolute: 世界座標 z")
    ap.add_argument("--min_height", type=float, default=-0.5)
    ap.add_argument("--max_height", type=float, default=0.7)
    ap.add_argument("--min_points_in_pix", type=int, default=2)
    ap.add_argument("--max_points_in_pix", type=int, default=5)
    ap.add_argument("--export_pcd", default=None,
                    help="全点 (高さフィルタ前) の世界座標マージ PCD をこのパスへ書き出す")
    # claude: 2026-10-02 未観測域を灰 (map_server で unknown) にする
    ap.add_argument("--mark_unknown", action="store_true",
                    help="センサから見えた範囲を空き、見えていない範囲を --unknown_value で塗る (要 cv2)")
    ap.add_argument("--unknown_value", type=int, default=180,
                    help="未観測画素の値。occupied/free_thresh 0.5/0.2 では 128〜204 が unknown")
    ap.add_argument("--free_range_max", type=float, default=30.0, help="空きを伸ばす最大水平距離 [m]")
    ap.add_argument("--view_stride", type=int, default=10,
                    help="submap 内のスキャン姿勢を N 個に 1 個だけ視点に使う (点とスキャンの対応は dump に無いので近似)")
    ap.add_argument("--free_bins", type=int, default=720, help="空き判定の方位分割数 (720 = 0.5°)")
    args = ap.parse_args()

    ids = list_submaps(args.dump_dir)
    world_pts = []      # スライス帯を通過した点 (地図用)
    all_pts = []        # 全点 (--export_pcd 用)
    views = []          # (視点 (F,3), 全点 xy, 帯内マスク) — --mark_unknown 用
    for i in ids:
        T, pts = load_submap(args.dump_dir, i)
        w = pts @ T[:3, :3].T + T[:3, 3]
        if args.export_pcd:
            all_pts.append(w)
        z_ref = T[2, 3] if args.height_mode == "sensor" else 0.0
        rel = w[:, 2] - z_ref
        keep = (rel >= args.min_height) & (rel <= args.max_height)
        world_pts.append(w[keep, :2])
        if args.mark_unknown:
            vp = load_viewpoints(args.dump_dir, i)
            vp = vp[::max(args.view_stride, 1)] if len(vp) else T[None, :3, 3]
            views.append((vp, w[:, :2].copy(), keep))
    sel = np.concatenate(world_pts)
    print(f"submaps: {len(ids)}, points in height band: {len(sel)}")

    if args.export_pcd:
        merged = np.concatenate(all_pts)
        write_pcd(args.export_pcd, merged)
        print(f"exported merged PCD: {args.export_pcd} ({len(merged)} points, "
              f"x[{merged[:,0].min():.1f},{merged[:,0].max():.1f}] "
              f"y[{merged[:,1].min():.1f},{merged[:,1].max():.1f}] "
              f"z[{merged[:,2].min():.1f},{merged[:,2].max():.1f}])")

    res = args.resolution
    if args.center == "world":
        cx, cy = 0.0, 0.0
    else:
        cx = (sel[:, 0].min() + sel[:, 0].max()) / 2
        cy = (sel[:, 1].min() + sel[:, 1].max()) / 2
    W, H = args.map_width, args.map_height
    if W == 0 or H == 0:
        # bbox が入る最小サイズ + 2 m マージン (64 px 単位に切り上げ)
        need_w = int(np.ceil((2 * np.abs(sel[:, 0] - cx).max() + 4.0) / res / 64) * 64)
        need_h = int(np.ceil((2 * np.abs(sel[:, 1] - cy).max() + 4.0) / res / 64) * 64)
        W = W or need_w
        H = H or need_h
    print(f"map: {W}x{H} px @ {res} m/px, center=({cx:.2f},{cy:.2f})")

    # 既製ツールと同じ画素系: x 右向き、y は上向き (画像行は -y)。中心画素 = (cx,cy)
    px = np.floor((sel[:, 0] - cx) / res).astype(np.int64) + W // 2
    py = np.floor(-(sel[:, 1] - cy) / res).astype(np.int64) + H // 2
    ok = (px >= 0) & (px < W) & (py >= 0) & (py < H)
    dropped = np.count_nonzero(~ok)
    if dropped:
        print(f"warning: {dropped} points fell outside the map ({100*dropped/len(sel):.1f}%)")
    counts = np.bincount(py[ok] * W + px[ok], minlength=W * H).reshape(H, W)

    # 既製ツールと同じ濃度変換: count<=min → 白 (自由), count>=max → 黒 (占有)
    lo, hi = args.min_points_in_pix, args.max_points_in_pix
    img = np.clip(255.0 - 255.0 * (counts - lo) / (hi - lo), 0, 255).astype(np.uint8)

    if args.mark_unknown:
        # claude: 2026-10-02 白 (点数 <= lo) のうち、どの視点からも見えていない画素を unknown に
        free = np.zeros((H, W), np.uint8)
        nv = 0
        for vp, wxy, keep in views:
            for v in vp:
                add_free_fan(free, v[0], v[1], wxy[:, 0], wxy[:, 1], keep, cx, cy, res,
                             args.free_range_max, args.free_bins)
                nv += 1
        unk = (img == 255) & (free == 0)
        img[unk] = args.unknown_value
        print(f"mark_unknown: {nv} viewpoints, free {int(free.sum())} px, unknown {int(unk.sum())} px")

    os.makedirs(args.dest_dir, exist_ok=True)
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
    print(f"wrote {pgm} and map.yaml (origin=[{origin_x:.2f},{origin_y:.2f}])")
    return 0


if __name__ == "__main__":
    sys.exit(main())
