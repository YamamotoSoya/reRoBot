#!/usr/bin/env python3
# claude: 2026-10-12 新設 — 「基準姿勢を前後 / 左右 / ヨーに振ったらスキャンの当たり率がどれだけ落ちるか」= 尤度の地形の平らさ。
#   平ら = その場所では地図が姿勢を決められない (手法によらず外れやすい)。見失い直前の診断用。
#   --model hit  : スキャン終点が占有セルから hit_dist [m] 以内にある割合 (scan_fit.py と同じ。AMCL の尤度より平らに見える)
#   --model amcl : nav2_amcl likelihood_field の式 (max_beams 本を等間隔、Σ(z_hit·exp(−d²/2σ²) + z_rand/range_max)³) を
#                  基準姿勢の値で割った比。AMCL の見え方に近い (critic 2026-10-12 の指摘で追加)
#   --yaw_profile を付けるとヨー −24..+24° を 2° 刻みでも出す (別の山があるか)
# 使い方 (コンテナ内、ROS を source):
#   python3 fit_landscape.py <bag_dir> <ref_dir> <map.yaml[,map2.yaml...]> --times 60 150 240 [--model amcl] [--yaw_profile]
import argparse
import os
import sys

import cv2
import numpy as np
import yaml
from scipy.spatial import cKDTree

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from scan_fit import endpoints, pose_at, read_scans  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag"); ap.add_argument("ref_dir"); ap.add_argument("map_yamls", help="カンマ区切りで複数可")
    ap.add_argument("--times", type=float, nargs="+", required=True); ap.add_argument("--scan", default="/rfans/scan")
    ap.add_argument("--rmax", type=float, default=30.0); ap.add_argument("--hit_dist", type=float, default=0.3)
    ap.add_argument("--model", choices=["hit", "amcl"], default="hit")
    ap.add_argument("--max_beams", type=int, default=60); ap.add_argument("--sigma_hit", type=float, default=0.2)
    ap.add_argument("--z_hit", type=float, default=0.5); ap.add_argument("--z_rand", type=float, default=0.5)
    ap.add_argument("--yaw_profile", action="store_true")
    a = ap.parse_args()
    ref = np.loadtxt(os.path.join(a.ref_dir, "ref_base.tum"), ndmin=2); t0 = ref[0, 0]
    scans = read_scans(a.bag, a.scan, [t0 + t for t in a.times])
    for my in a.map_yamls.split(","):
        y = yaml.safe_load(open(my)); img = cv2.imread(os.path.join(os.path.dirname(my), y["image"]), 0)
        res, ox, oy = y["resolution"], y["origin"][0], y["origin"][1]; H, W = img.shape
        occ = (255 - img.astype(np.float32)) / 255.0 > y["occupied_thresh"]; rows, cols = np.nonzero(occ)
        tree = cKDTree(np.c_[ox + (cols + 0.5) * res, oy + (H - rows - 0.5) * res])

        def score(sc, p):
            pts = endpoints(sc, p, a.rmax)
            if a.model == "hit":
                d, _ = tree.query(pts, distance_upper_bound=a.hit_dist); return np.isfinite(d).mean()
            step = max(1, len(pts) // a.max_beams); pts = pts[::step]
            d, _ = tree.query(pts, distance_upper_bound=2.0); d = np.where(np.isfinite(d), d, 2.0)
            pz = a.z_hit * np.exp(-d * d / (2 * a.sigma_hit ** 2)) + a.z_rand / a.rmax
            return float((pz ** 3).sum())

        print(f"map: {my}  model: {a.model}  (各 = 2 方向の平均、基準姿勢の値を 1 とした比)")
        print("    t | along ±1 ±2 ±5 m | across ±0.5 ±1 ±2 m | yaw ±5 ±10 ±20 deg")
        for t in a.times:
            sc = scans.get(t0 + t)
            if sc is None:
                print(f"{t:5.0f} no scan"); continue
            x, yy, yaw = pose_at(ref, t0 + t); s0 = score(sc, (x, yy, yaw)); c, s = np.cos(yaw), np.sin(yaw)
            rel = lambda vals: " ".join(f"{v / s0:.2f}" for v in vals)  # noqa: E731
            al = [np.mean([score(sc, (x + d * c, yy + d * s, yaw)) for d in (-m, m)]) for m in (1, 2, 5)]
            ac = [np.mean([score(sc, (x - d * s, yy + d * c, yaw)) for d in (-m, m)]) for m in (0.5, 1, 2)]
            yw = [np.mean([score(sc, (x, yy, yaw + np.radians(d))) for d in (-m, m)]) for m in (5, 10, 20)]
            print(f"{t:5.0f} | {rel(al)} | {rel(ac)} | {rel(yw)}" + (f"   (hit@ref {s0:.2f})" if a.model == "hit" else ""))
            if a.yaw_profile:
                degs = range(-24, 25, 2); vals = [score(sc, (x, yy, yaw + np.radians(d))) / s0 for d in degs]
                print("      yaw " + " ".join(f"{d:+d}:{v:.2f}" for d, v in zip(degs, vals)) + f"  peak {list(degs)[int(np.argmax(vals))]:+d}°")


if __name__ == "__main__":
    main()
