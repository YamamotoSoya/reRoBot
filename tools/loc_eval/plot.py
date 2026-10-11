#!/usr/bin/env python3
# claude: 2026-10-11 新設 — score.py の err.csv から比較図を作る (rerobot_env: matplotlib + cv2)。
#
#   timeline.png   横軸 bag 時刻 (initialpose からの秒)、縦軸 xy 誤差。全構成を重ね描き、0.5 / 1.0 m の線入り
#   cdf.png        xy 誤差の累積分布 (構成ごと 1 本、反復 run は中央値の run を太く)
#   map_<群>.png   2D 地図 (map.yaml) の上に推定軌跡を誤差で色分け (基準軌跡は細い灰線)
#   worst_<群>.png 誤差最大時刻の前後の軌跡拡大 (推定 vs 基準)
#
# 使い方 (コンテナ内):
#   python3 plot.py <ref_dir> <map.yaml> <out_dir> <run_dir>... [--lost_m 1.0]
import argparse
import glob
import json
import os
import re

import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import yaml


def group_of(run):
    return re.sub(r"_r\d+$", "", os.path.basename(os.path.normpath(run)))


def load_runs(runs):
    out = {}
    for r in runs:
        p = os.path.join(r, "err.csv")
        if not os.path.exists(p):
            continue
        e = np.loadtxt(p, skiprows=1, ndmin=2)
        s = json.load(open(os.path.join(r, "score.json")))
        out.setdefault(group_of(r), []).append((os.path.basename(r), e, s))
    return out


def median_run(items):
    """p95 が中央の run を代表にする"""
    items = sorted(items, key=lambda x: x[2]["p95_xy_m"])
    return items[len(items) // 2]


def load_map(map_yaml):
    y = yaml.safe_load(open(map_yaml))
    img = cv2.imread(os.path.join(os.path.dirname(map_yaml), y["image"]), cv2.IMREAD_GRAYSCALE)
    res, ox, oy = y["resolution"], y["origin"][0], y["origin"][1]
    return img, res, ox, oy


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("ref_dir")
    ap.add_argument("map_yaml")
    ap.add_argument("out_dir")
    ap.add_argument("runs", nargs="+")
    ap.add_argument("--lost_m", type=float, default=1.0)
    ap.add_argument("--vmax", type=float, default=1.0, help="地図上の色分けの上限 [m]")
    args = ap.parse_args()
    os.makedirs(args.out_dir, exist_ok=True)
    runs = sorted(set(sum([glob.glob(r) for r in args.runs], [])))
    groups = load_runs(runs)
    ref = np.loadtxt(os.path.join(args.ref_dir, "ref_base.tum"), ndmin=2)
    colors = plt.cm.tab10(np.linspace(0, 1, 10))

    # 1) 誤差の時系列
    fig, ax = plt.subplots(figsize=(14, 4.5))
    for i, (g, items) in enumerate(groups.items()):
        rep = median_run(items)
        for name, e, s in items:
            lw, al = (1.6, 1.0) if name == rep[0] else (0.6, 0.35)
            ax.plot(e[:, 0] - s["t0"], e[:, 1], color=colors[i % 10], lw=lw, alpha=al,
                    label=f"{g} (n={len(items)}, p95 {rep[2]['p95_xy_m']:.2f} m)" if name == rep[0] else None)
    ax.axhline(0.5, color="k", ls=":", lw=0.8); ax.axhline(args.lost_m, color="r", ls="--", lw=0.8)
    ax.set_xlabel("time since initialpose [s]"); ax.set_ylabel("xy error [m]")
    ax.set_ylim(0, max(2.0, min(10.0, max(e[:, 1].max() for it in groups.values() for _, e, _ in it) * 1.05)))
    ax.grid(alpha=0.3); ax.legend(fontsize=8, loc="upper left")
    fig.tight_layout(); fig.savefig(os.path.join(args.out_dir, "timeline.png"), dpi=130); plt.close(fig)

    # 2) CDF
    fig, ax = plt.subplots(figsize=(7, 5))
    for i, (g, items) in enumerate(groups.items()):
        rep = median_run(items)
        for name, e, s in items:
            x = np.sort(e[:, 1]); yv = np.arange(1, len(x) + 1) / len(x)
            lw, al = (2.0, 1.0) if name == rep[0] else (0.7, 0.35)
            ax.plot(x, yv, color=colors[i % 10], lw=lw, alpha=al, label=g if name == rep[0] else None)
    ax.axvline(0.5, color="k", ls=":", lw=0.8); ax.axvline(args.lost_m, color="r", ls="--", lw=0.8)
    ax.axhline(0.5, color="gray", lw=0.5); ax.axhline(0.95, color="gray", lw=0.5)
    ax.set_xlim(0, 3.0); ax.set_ylim(0, 1); ax.set_xlabel("xy error [m]"); ax.set_ylabel("fraction of time ≤ x")
    ax.grid(alpha=0.3); ax.legend(fontsize=8, loc="lower right")
    fig.tight_layout(); fig.savefig(os.path.join(args.out_dir, "cdf.png"), dpi=130); plt.close(fig)

    # 3) 地図上の軌跡 (群ごと、代表 run)
    img, res, ox, oy = load_map(args.map_yaml)
    H, W = img.shape
    # 軌跡の周囲だけ切り出す
    rx, ry = ref[:, 1], ref[:, 2]
    pad = 15.0
    x0, x1 = max(ox, rx.min() - pad), min(ox + W * res, rx.max() + pad)
    y0, y1 = max(oy, ry.min() - pad), min(oy + H * res, ry.max() + pad)
    c0, c1 = int((x0 - ox) / res), int((x1 - ox) / res)
    r0, r1 = int(H - (y1 - oy) / res), int(H - (y0 - oy) / res)
    crop = img[r0:r1, c0:c1]
    ext = [ox + c0 * res, ox + c1 * res, oy + (H - r1) * res, oy + (H - r0) * res]
    for g, items in groups.items():
        name, e, s = median_run(items)
        fig, ax = plt.subplots(figsize=(13, 13 * (y1 - y0) / (x1 - x0) + 0.8))
        ax.imshow(crop, cmap="gray", extent=ext, vmin=0, vmax=255, interpolation="nearest")
        ax.plot(rx, ry, color="deepskyblue", lw=0.8, alpha=0.9, label="reference (GLIM)")
        sc = ax.scatter(e[:, 4], e[:, 5], c=np.minimum(e[:, 1], args.vmax), cmap="jet", s=3, vmin=0, vmax=args.vmax, label="estimate")
        lost = e[:, 1] > args.lost_m
        if lost.any():
            ax.scatter(e[lost, 4], e[lost, 5], facecolors="none", edgecolors="magenta", s=20, lw=0.6, label=f"> {args.lost_m} m")
        ax.plot(rx[0], ry[0], "g^", ms=10, label="start")
        cb = fig.colorbar(sc, ax=ax, fraction=0.03, pad=0.01); cb.set_label("xy error [m]")
        ax.set_title(f"{g} / {name}: median {s['median_xy_m']:.2f} m, p95 {s['p95_xy_m']:.2f} m, "
                     f">{args.lost_m} m {100 * s['frac_over_lost']:.1f} %, lost {s['lost_episodes']}x")
        ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]"); ax.legend(fontsize=8, loc="upper right")
        fig.tight_layout(); fig.savefig(os.path.join(args.out_dir, f"map_{g}.png"), dpi=110); plt.close(fig)

        # 4) 最悪時刻の拡大
        k = int(np.argmax(e[:, 1]))
        tk = e[k, 0]
        w = (e[:, 0] > tk - 20) & (e[:, 0] < tk + 20)
        rw = (ref[:, 0] > tk - 20) & (ref[:, 0] < tk + 20)
        if w.any() and rw.any():
            cx, cy = e[k, 4], e[k, 5]
            fig, ax = plt.subplots(figsize=(8, 8))
            ax.imshow(crop, cmap="gray", extent=ext, vmin=0, vmax=255, interpolation="nearest")
            ax.plot(ref[rw, 1], ref[rw, 2], "-", color="deepskyblue", lw=2, label="reference ±20 s")
            ax.scatter(e[w, 4], e[w, 5], c=np.minimum(e[w, 1], args.vmax), cmap="jet", s=10, vmin=0, vmax=args.vmax, label="estimate ±20 s")
            ax.plot(cx, cy, "mx", ms=14, mew=2, label=f"worst {e[k, 1]:.2f} m @ {tk - s['t0']:.0f} s")
            ax.set_xlim(cx - 15, cx + 15); ax.set_ylim(cy - 15, cy + 15)
            ax.set_title(f"{g} / {name}: worst moment"); ax.legend(fontsize=8)
            fig.tight_layout(); fig.savefig(os.path.join(args.out_dir, f"worst_{g}.png"), dpi=110); plt.close(fig)
    print("written:", sorted(os.listdir(args.out_dir)))


if __name__ == "__main__":
    main()
