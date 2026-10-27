#!/usr/bin/env python3
# claude: 2026-10-26 作成
"""静止した R-Fans の点群 bag から櫛の偽点を赤で描く (真上から).

判定は rfans_comb_check.py と同じ: 同じ laserid を時刻順に並べ、両隣 2 点ずつが直線に乗る (4 cm 以内)
のに中央だけ k x 0.686 m (k=±1..±5, ±6 cm) ずれた点を偽点とする。ロボットが止まっている前提で
全フレームを重ねる (姿勢補正なし)。

例 (main コンテナ内, ws を source してから):
  python3 rfans_comb_image.py <bag_dir> -o out.png [--zoom X Y R]...
"""
import argparse

import numpy as np

P = 0.686


def read_clouds(path, topic):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py import point_cloud2

    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=path, storage_id=""), rosbag2_py.ConverterOptions("", ""))
    r.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    while r.has_next():
        _, data, _ = r.read_next()
        m = deserialize_message(data, PointCloud2)
        a = point_cloud2.read_points(m, field_names=("x", "y", "z", "intensity", "laserid", "time"))
        yield np.stack([a[f].astype(np.float64) for f in a.dtype.names], 1)


def flag(c):
    """c: (N,6) 1 フレーム。戻り値: 偽点フラグ, 判定可能フラグ."""
    ghost = np.zeros(len(c), bool)
    valid = np.zeros(len(c), bool)
    for lid in np.unique(c[:, 4]):
        idx = np.where(c[:, 4] == lid)[0]
        idx = idx[np.argsort(c[idx, 5], kind="stable")]
        r = np.linalg.norm(c[idx, :3], axis=1)
        if len(r) < 5:
            continue
        m2, m1, cc, p1, p2 = r[:-4], r[1:-3], r[2:-2], r[3:-1], r[4:]
        slope = (2 * (p2 - m2) + (p1 - m1)) / 10.0
        mean = (m2 + m1 + p1 + p2) / 4.0
        res = np.max(np.abs(np.stack([m2 - (mean - 2 * slope), m1 - (mean - slope),
                                      p1 - (mean + slope), p2 - (mean + 2 * slope)])), axis=0)
        v = (res < 0.04) & (np.minimum.reduce([m2, m1, p1, p2]) > 0.5) & (cc > 0)
        off = cc - mean
        k = np.round(off / P)
        g = v & (np.abs(k) >= 1) & (np.abs(k) <= 5) & (np.abs(off - k * P) < 0.06)
        valid[idx[2:-2]] = v
        ghost[idx[2:-2]] = g
    return ghost, valid


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("-o", "--out", required=True)
    ap.add_argument("--topic", default="/rfans_driver/rfans_points")
    ap.add_argument("--extent", type=float, default=20.0, help="全体図の半幅 [m]")
    ap.add_argument("--zoom", nargs=3, type=float, action="append", default=[], metavar=("X", "Y", "R"))
    ap.add_argument("--zmin", type=float, default=-0.6, help="描く高さの下限 (rfans 座標) — 床を除く")
    ap.add_argument("--zmax", type=float, default=2.0)
    args = ap.parse_args()

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams["font.family"] = ["Droid Sans Fallback", "DejaVu Sans"]

    P_all, G_all = [], []
    nv = ng = nf = 0
    for c in read_clouds(args.bag, args.topic):
        c = c[np.isfinite(c[:, :3]).all(1) & (np.linalg.norm(c[:, :3], axis=1) > 0.3)]
        g, v = flag(c)
        nv += v.sum(); ng += g.sum(); nf += 1
        P_all.append(c[:, :3]); G_all.append(g)
    P = np.concatenate(P_all); G = np.concatenate(G_all)
    print(f"frames {nf}, points {len(P)}, ghost {G.sum()} ({100 * ng / max(nv, 1):.2f}% of valid)")
    band = (P[:, 2] > args.zmin) & (P[:, 2] < args.zmax)

    panels = [(0.0, 0.0, args.extent, "全体 (真上から)")] + [(x, y, r, f"拡大 ({x:.1f}, {y:.1f})") for x, y, r in args.zoom]
    fig, axs = plt.subplots(1, len(panels), figsize=(8 * len(panels), 8), squeeze=False)
    for ax, (cx, cy, r, title) in zip(axs[0], panels):
        sel = band & (np.abs(P[:, 0] - cx) < r) & (np.abs(P[:, 1] - cy) < r)
        s_ok, s_g = sel & ~G, sel & G
        ax.scatter(P[s_ok, 0], P[s_ok, 1], s=0.3 if r > 5 else 1.0, c="0.35", lw=0, label=f"通常 {s_ok.sum()}")
        ax.scatter(P[s_g, 0], P[s_g, 1], s=2 if r > 5 else 5, c="red", lw=0, label=f"櫛の偽点 {s_g.sum()}")
        ax.plot(0, 0, "b^", ms=10, label="LiDAR")
        if r > 5:
            for x, y, rr in args.zoom:
                ax.add_patch(plt.Rectangle((x - rr, y - rr), 2 * rr, 2 * rr, fill=False, ec="b", lw=1))
        ax.set_xlim(cx - r, cx + r); ax.set_ylim(cy - r, cy + r)
        ax.set_aspect("equal"); ax.grid(alpha=0.3)
        ax.set_title(f"{title}  z {args.zmin}〜{args.zmax} m"); ax.set_xlabel("x [m] (rfans 座標)"); ax.set_ylabel("y [m]")
        ax.legend(loc="upper right", markerscale=4, fontsize=8)
    fig.tight_layout()
    fig.savefig(args.out, dpi=110)
    print("wrote", args.out)


if __name__ == "__main__":
    main()
