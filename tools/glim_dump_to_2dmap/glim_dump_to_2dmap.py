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
    args = ap.parse_args()

    ids = list_submaps(args.dump_dir)
    world_pts = []      # スライス帯を通過した点 (地図用)
    all_pts = []        # 全点 (--export_pcd 用)
    for i in ids:
        T, pts = load_submap(args.dump_dir, i)
        w = pts @ T[:3, :3].T + T[:3, 3]
        if args.export_pcd:
            all_pts.append(w)
        z_ref = T[2, 3] if args.height_mode == "sensor" else 0.0
        rel = w[:, 2] - z_ref
        keep = (rel >= args.min_height) & (rel <= args.max_height)
        world_pts.append(w[keep, :2])
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
