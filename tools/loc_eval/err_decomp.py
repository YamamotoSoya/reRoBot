#!/usr/bin/env python3
# claude: 2026-10-12 新設 — 推定誤差を基準姿勢の前後 (along, 前+) / 左右 (across, 左+) / ヨー [deg] に分解して時刻ごとに出す。
#   「どの方向に外れたか」で機構を絞る (ヨーが先か、along が溜まるか)。
# 使い方: python3 err_decomp.py <ref_dir> <run_dir>... [--times 60 150 ...]
import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from scan_fit import pose_at  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("ref_dir"); ap.add_argument("runs", nargs="+")
ap.add_argument("--times", type=float, nargs="+", default=[60, 150, 200, 240, 270, 300, 320, 340, 377])
a = ap.parse_args()
ref = np.loadtxt(os.path.join(a.ref_dir, "ref_base.tum"), ndmin=2); t0 = ref[0, 0]
for r in a.runs:
    est = np.loadtxt(os.path.join(r, "est.tum"), ndmin=2); print(os.path.basename(r), " t: along across yaw[deg]")
    row = []
    for t in a.times:
        ta = t0 + t
        if not (est[0, 0] <= ta <= est[-1, 0]):
            continue
        x, y, yaw = pose_at(ref, ta); ex, ey, eyaw = pose_at(est, ta); dx, dy = ex - x, ey - y
        al = dx * np.cos(yaw) + dy * np.sin(yaw); ac = -dx * np.sin(yaw) + dy * np.cos(yaw)
        dyaw = np.degrees(np.arctan2(np.sin(eyaw - yaw), np.cos(eyaw - yaw)))
        row.append(f"{t:.0f}: {al:+.2f} {ac:+.2f} {dyaw:+.1f}")
    print("  " + " | ".join(row))
