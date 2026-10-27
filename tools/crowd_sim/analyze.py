#!/usr/bin/env python3
# claude: 2026-10-24 — 人垣試験の採点補助。score.py が書いた err.csv を「人垣の中 (スタートから r_max+5 m 以内)」と
#   「抜けた後」の 2 相に分けて集計し、構成ごとの誤差時系列 + スタート付近の推定軌跡を描く。
#   使い方: analyze.py <ref_dir> <runs_glob> <crowd_json> <out_dir>
import argparse
import glob
import json
import os
import re

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def group_of(run):
    return re.sub(r"_r\d+$", "", os.path.basename(run))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("ref_dir"); ap.add_argument("runs"); ap.add_argument("crowd_json"); ap.add_argument("out")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    ref = np.loadtxt(os.path.join(a.ref_dir, "ref_base.tum"))
    cj = json.load(open(a.crowd_json))
    sx, sy, r_out = cj["start_xy"][0], cj["start_xy"][1], cj["r_max"] + 5.0
    t_ref0 = ref[0, 0]
    d_ref = np.hypot(ref[:, 1] - sx, ref[:, 2] - sy)
    t_exit = ref[np.argmax(d_ref > r_out), 0] - t_ref0   # 人垣の外へ出る時刻 [s, 基準開始から]

    rows = []; series = {}
    for run in sorted(glob.glob(a.runs)):
        f = os.path.join(run, "err.csv")
        if not os.path.exists(f):
            continue
        e = np.loadtxt(f, skiprows=1, ndmin=2)
        t = e[:, 0] - t_ref0; xy = e[:, 1]; yaw = e[:, 2]
        inside = t <= t_exit
        after = ~inside
        lost = []
        bad = xy > 1.0
        i = 0
        while i < len(t):
            if bad[i]:
                j = i
                while j < len(t) and bad[j]:
                    j += 1
                if t[j - 1] - t[i] >= 1.0:
                    lost.append((round(float(t[i]), 1), round(float(t[j - 1]), 1)))
                i = j
            else:
                i += 1
        tail = t > t[-1] - 30
        rows.append({
            "run": os.path.basename(run), "group": group_of(run), "t_first": float(t[0]),
            "init_xy": float(np.median(xy[t <= t[0] + 3.0])), "init_yaw": float(np.median(yaw[t <= t[0] + 3.0])),
            "in_med": float(np.median(xy[inside])) if inside.any() else np.nan,
            "in_max": float(xy[inside].max()) if inside.any() else np.nan,
            "in_yaw_p95": float(np.percentile(yaw[inside], 95)) if inside.any() else np.nan,
            "out_med": float(np.median(xy[after])) if after.any() else np.nan,
            "out_max": float(xy[after].max()) if after.any() else np.nan,
            "tail_med": float(np.median(xy[tail])),
            "lost": lost, "lost_s": float(sum(b - a_ for a_, b in lost)),
        })
        series[os.path.basename(run)] = (t, xy, e[:, 4], e[:, 5])

    # --- 表 ---
    def agg(vals):
        v = [x for x in vals if not (isinstance(x, float) and np.isnan(x))]
        if not v:
            return "—"
        return f"{np.median(v):.2f}" if len(v) == 1 else f"{np.median(v):.2f} ({min(v):.2f}–{max(v):.2f})"
    groups = sorted(set(r["group"] for r in rows), key=lambda g: (g.split("_")[1], g.split("_")[0]))
    lines = [f"# 人垣試験 集計 (人垣 = スタート {cj['r_min']}〜{cj['r_max']} m、{cj['n_people']} 人、前方 ±{cj['front_half_deg']}° は空き。"
             f"「中」= スタートから {r_out:.0f} m 以内 = 基準開始 {t_exit:.0f} s まで、「後」= それ以降 300 s まで)", "",
             "| 構成 | runs | 初期 xy m | 初期 yaw ° | 中 xy 中央値 m | 中 xy 最大 m | 中 yaw p95 ° | 後 xy 中央値 m | 後 xy 最大 m | 末尾 30 s 中央値 m | 見失い 回 / 合計 s |",
             "|---|---|---|---|---|---|---|---|---|---|---|"]
    for g in groups:
        rs = [r for r in rows if r["group"] == g]
        lines.append(f"| {g} | {len(rs)} | {agg([r['init_xy'] for r in rs])} | {agg([r['init_yaw'] for r in rs])} | "
                     f"{agg([r['in_med'] for r in rs])} | {agg([r['in_max'] for r in rs])} | {agg([r['in_yaw_p95'] for r in rs])} | "
                     f"{agg([r['out_med'] for r in rs])} | {agg([r['out_max'] for r in rs])} | {agg([r['tail_med'] for r in rs])} | "
                     f"{agg([float(len(r['lost'])) for r in rs])} / {agg([r['lost_s'] for r in rs])} |")
    lines += ["", "## run ごと", "", "| run | 初期化 s | 初期 xy | 中 最大 | 後 中央値 | 見失い区間 (基準開始からの秒) |", "|---|---|---|---|---|---|"]
    for r in rows:
        lines.append(f"| {r['run']} | {r['t_first']:.1f} | {r['init_xy']:.2f} | {r['in_max']:.2f} | {r['out_med']:.2f} | {r['lost']} |")
    open(os.path.join(a.out, "summary.md"), "w").write("\n".join(lines) + "\n")
    json.dump(rows, open(os.path.join(a.out, "rows.json"), "w"), indent=1)
    print("\n".join(lines))

    # --- 誤差時系列 (構成ごとに 1 枚、base 破線 / crowd 実線) ---
    kinds = sorted(set(g.split("_", 1)[1] for g in groups))
    fig, axes = plt.subplots(len(kinds), 1, figsize=(11, 2.6 * len(kinds)), sharex=True, squeeze=False)
    for ax, k in zip(axes[:, 0], kinds):
        for name, (t, xy, _, _) in series.items():
            g = group_of(os.path.join("x", name))
            if g.split("_", 1)[1] != k:
                continue
            ax.plot(t, xy, ("--" if g.startswith("base") else "-"), lw=1, label=name)
        ax.axvspan(0, t_exit, color="orange", alpha=0.12)
        ax.set_ylim(0, 6); ax.set_ylabel("xy err [m]"); ax.set_title(k, loc="left", fontsize=10)
        ax.grid(alpha=0.3); ax.legend(fontsize=7, ncol=3, loc="upper right")
    axes[-1, 0].set_xlabel("s from reference start (orange = inside crowd ring)")
    fig.tight_layout(); fig.savefig(os.path.join(a.out, "timeline.png"), dpi=110)

    # --- スタート付近の推定軌跡 (crowd run だけ、基準 = 黒) ---
    fig, ax = plt.subplots(figsize=(8, 8))
    m = (ref[:, 0] - t_ref0) <= t_exit + 30
    ax.plot(ref[m, 1], ref[m, 2], "k-", lw=2, label="reference")
    people = np.array(cj["people"]) if cj.get("people") else np.zeros((0, 2))
    if len(people):
        ax.plot(people[:, 0], people[:, 1], ".", color="0.6", ms=3, label="people")
    for name, (t, xy, x, y) in series.items():
        if not name.startswith("crowd"):
            continue
        mm = t <= t_exit + 30
        ax.plot(x[mm], y[mm], "-", lw=1, label=name)
    ax.set_aspect("equal"); ax.grid(alpha=0.3); ax.legend(fontsize=7)
    ax.set_title("estimated path near start (crowd runs)")
    fig.tight_layout(); fig.savefig(os.path.join(a.out, "paths_start.png"), dpi=110)


if __name__ == "__main__":
    main()
