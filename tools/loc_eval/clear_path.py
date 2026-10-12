#!/usr/bin/env python3
# claude: 2026-10-12 新設 — 2D 地図の軌跡周り半径 R [m] の占有を free にした複製を作る (地図に焼き込まれた随伴者の帯の診断用)。
#   使い方: python3 clear_path.py <map.yaml> <traj.tum (t x y ...)> <radius_m> <out_dir>  → out_dir/map.{yaml,pgm}
import sys, os, numpy as np, yaml, cv2, shutil
my, tum, R, out = sys.argv[1], sys.argv[2], float(sys.argv[3]), sys.argv[4]
y = yaml.safe_load(open(my)); res = y["resolution"]; ox, oy = y["origin"][:2]
img = cv2.imread(os.path.join(os.path.dirname(my), y["image"]), 0); H = img.shape[0]
tr = np.loadtxt(tum)[:, 1:3]
mask = np.zeros_like(img)
for x, yy in tr[::2]:
    cv2.circle(mask, (int((x - ox) / res), int(H - (yy - oy) / res)), int(round(R / res)), 255, -1)
occ = (255 - img.astype(np.float32)) / 255.0 > y["occupied_thresh"]
n_before = int(occ.sum()); img2 = img.copy(); img2[(mask > 0) & occ] = 254
os.makedirs(out, exist_ok=True); cv2.imwrite(os.path.join(out, "map.pgm"), img2)
y["image"] = "map.pgm"; yaml.safe_dump(y, open(os.path.join(out, "map.yaml"), "w"))
print(f"R={R} occupied {n_before} -> {int(((255 - img2.astype(np.float32)) / 255.0 > y['occupied_thresh']).sum())}")
