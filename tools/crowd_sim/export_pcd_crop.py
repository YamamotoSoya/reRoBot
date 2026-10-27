#!/usr/bin/env python3
# claude: 2026-10-24 — GLIM dump の submap を世界座標でマージし、bbox で切り抜いた binary PCD を書く
#   (glim_dump_to_2dmap --export_pcd は 2D 地図も同時に作るので、3D 地図だけ欲しいときの軽量版。
#    つくば composite_4map は 640 submap ≈ 3,200 万点 → 評価区間だけに絞って lidar_localization の NDT 目標を軽くする)
#   使い方: export_pcd_crop.py <dump_dir> <out.pcd> --bbox xmin xmax ymin ymax [--voxel 0]
import argparse, os, sys
import numpy as np
sys.path.insert(0, "/workspace/tools/glim_dump_to_2dmap")
from glim_dump_to_2dmap import load_submap, write_pcd

ap = argparse.ArgumentParser()
ap.add_argument("dump_dir"); ap.add_argument("out")
ap.add_argument("--bbox", type=float, nargs=4, required=True, metavar=("XMIN", "XMAX", "YMIN", "YMAX"))
ap.add_argument("--voxel", type=float, default=0.0, help=">0 で voxel 間引き [m]")
a = ap.parse_args()
ids = sorted(int(n) for n in os.listdir(a.dump_dir) if n.isdigit() and len(n) == 6)
keep = []; total = 0
for i in ids:
    T, pts = load_submap(a.dump_dir, i)
    w = pts @ T[:3, :3].T + T[:3, 3]
    total += len(w)
    m = (w[:, 0] >= a.bbox[0]) & (w[:, 0] <= a.bbox[1]) & (w[:, 1] >= a.bbox[2]) & (w[:, 1] <= a.bbox[3])
    if m.any():
        keep.append(w[m].astype(np.float32))
w = np.concatenate(keep)
if a.voxel > 0:
    key = np.floor(w / a.voxel).astype(np.int64)
    _, idx = np.unique(key, axis=0, return_index=True)
    w = w[np.sort(idx)]
write_pcd(a.out, w)
print(f"submaps {len(ids)}, points total {total}, kept {len(w)} -> {a.out}")
