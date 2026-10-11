#!/usr/bin/env python3
# claude: 2026-10-11 新設 — replay_one.sh の記録 (est.tum) を基準軌跡 (ref_base.tum) と突き合わせて採点する。
#
#   位置合わせはしない (地図座標のまま比較。上流 lidar_localization_ros2 tools/benchmark と同じ考え方 —
#   合わせると「間違った場所に初期化してそのまま追従」が隠れる)。指標の定義も上流 benchmark_eval.py に揃える。
#
#   run ごと: err.csv (時刻, xy 誤差, yaw 誤差, 自己申告フラグ) + score.json
#   全体:     summary.md + results.json (run 名の末尾 _rN を除いた名前で群にまとめ、群の中央値と範囲を出す)
#
# 使い方 (コンテナ内):
#   python3 score.py <ref_dir> <run_dir>... [--out <dir>] [--cov_thr 0.5] [--lost_m 1.0] [--conv_m 0.3]
import argparse
import glob
import json
import os
import re

import numpy as np


def quat_yaw(q):
    x, y, z, w = q.T
    return np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def interp_ref(ref, t):
    """基準を est の時刻 t に線形補間 (xy, z) + yaw (unwrap 後に補間)。範囲外は NaN。"""
    rt = ref[:, 0]
    inside = (t >= rt[0]) & (t <= rt[-1])
    out = np.full((len(t), 4), np.nan)
    yaw = np.unwrap(quat_yaw(ref[:, 4:8]))
    for k, col in enumerate((1, 2, 3)):
        out[inside, k] = np.interp(t[inside], rt, ref[:, col])
    out[inside, 3] = wrap(np.interp(t[inside], rt, yaw))
    return out


def episodes(mask, t, min_len):
    """mask が連続 True の区間 [(t0, t1)] を min_len 秒以上のものだけ返す"""
    out = []
    i = 0
    n = len(mask)
    while i < n:
        if mask[i]:
            j = i
            while j + 1 < n and mask[j + 1]:
                j += 1
            if t[j] - t[i] >= min_len:
                out.append((float(t[i]), float(t[j])))
            i = j + 1
        else:
            i += 1
    return out


def load_events(run):
    init_t = None
    try:
        for line in open(os.path.join(run, "events.txt")):
            if " initialpose " in line and init_t is None:
                init_t = float(line.split()[0])
    except OSError:
        pass
    return init_t


def load_flags(run, meta, cov_thr):
    """自己申告「怪しい」フラグの時系列 (stamp, flag)。amcl/emcl2 = 共分散 sqrt(xx+yy) > cov_thr、lidar3d = alignment level != 0"""
    rows = []
    if meta.get("loc") == "lidar3d":
        try:
            for line in open(os.path.join(run, "alignment.jsonl")):
                r = json.loads(line)
                rows.append((r["stamp"], 1.0 if r["level"] != 0 else 0.0))
        except OSError:
            pass
    else:
        try:
            for line in open(os.path.join(run, "cov.txt")):
                p = line.split()
                rows.append((float(p[0]), 1.0 if np.sqrt(float(p[2]) + float(p[3])) > cov_thr else 0.0))
        except (OSError, IndexError):
            pass
    rows.sort()
    return np.array(rows) if rows else np.empty((0, 2))


def flags_at(stamps, flags, max_age=1.0):
    """各 stamp 直前の自己申告。max_age 秒以上申告が無ければ「怪しい」扱い (上流と同じ)"""
    if len(flags) == 0:
        return np.ones(len(stamps), bool)
    idx = np.searchsorted(flags[:, 0], stamps, side="right") - 1
    ok = idx >= 0
    age = np.where(ok, stamps - flags[idx.clip(0), 0], np.inf)
    f = np.where(ok, flags[idx.clip(0), 1], 1.0)
    return (age > max_age) | (f != 0)


def score_run(run, ref, args):
    meta = json.load(open(os.path.join(run, "meta.json"))) if os.path.exists(os.path.join(run, "meta.json")) else {}
    est = np.loadtxt(os.path.join(run, "est.tum"), ndmin=2)
    init_t = load_events(run)
    if len(est) == 0:
        return {"run": run, "group": group_of(run), "initialized": False}
    if init_t is not None:
        est = est[est[:, 0] >= init_t]
    r = interp_ref(ref, est[:, 0])
    ok = ~np.isnan(r[:, 0])
    est, r = est[ok], r[ok]
    t = est[:, 0]
    xy = np.hypot(est[:, 1] - r[:, 0], est[:, 2] - r[:, 1])
    yaw = np.degrees(np.abs(wrap(quat_yaw(est[:, 4:8]) - r[:, 3])))
    flagged = flags_at(t, load_flags(run, meta, args.cov_thr))
    np.savetxt(os.path.join(run, "err.csv"), np.c_[t, xy, yaw, flagged, est[:, 1], est[:, 2]],
               fmt=["%.3f", "%.4f", "%.3f", "%d", "%.4f", "%.4f"], header="t xy_m yaw_deg flagged x y", comments="")

    t0 = init_t if init_t is not None else t[0]
    # 収束: initialpose 後、conv_m 以内が 1 s 続いた最初の時刻
    conv = None
    for a, b in episodes(xy < args.conv_m, t, 1.0):
        conv = a - t0
        break
    lost = episodes(xy > args.lost_m, t, 1.0)
    lost_total = sum(b - a for a, b in lost)
    recovered = sum(1 for a, b in lost if b < t[-1] - 2.0)
    off, right = xy > args.lost_m, xy < args.conv_m
    ref_span = ref[(ref[:, 0] >= t0) & (ref[:, 0] <= t[-1]), 0]
    cover = len(set(np.round(t / 0.1).astype(int)) & set(np.round(ref_span / 0.1).astype(int))) / max(1, len(set(np.round(ref_span / 0.1).astype(int))))
    # 鮮度: 「いま」と TF 連鎖の最新共通時刻の差 (record_pose.py latency.txt)。Nav2 が同じ lookup で得る姿勢の古さ
    lat = None
    try:
        lt = np.loadtxt(os.path.join(run, "latency.txt"), ndmin=2)
        lt = lt[lt[:, 0] >= t0]
        lat = (float(np.median(lt[:, 1])), float(np.percentile(lt[:, 1], 95))) if len(lt) else None
    except OSError:
        pass
    s = {
        "run": run, "group": group_of(run), "loc": meta.get("loc"), "initialized": True,
        "latency_median_s": lat and lat[0], "latency_p95_s": lat and lat[1],
        "n": int(len(xy)), "duration_s": float(t[-1] - t0), "coverage": float(cover),
        "converge_s": conv,
        "median_xy_m": float(np.median(xy)), "p95_xy_m": float(np.percentile(xy, 95)), "max_xy_m": float(xy.max()),
        "frac_over_0_5_m": float(np.mean(xy > 0.5)), "frac_over_lost": float(np.mean(off)),
        "median_yaw_deg": float(np.median(yaw)), "p95_yaw_deg": float(np.percentile(yaw, 95)),
        "lost_episodes": len(lost), "lost_total_s": float(lost_total),
        "lost_max_s": float(max((b - a for a, b in lost), default=0.0)), "lost_recovered": recovered,
        "lost_list": [(round(a - t0, 1), round(b - t0, 1)) for a, b in lost],
        "flag_when_off": float(np.mean(flagged[off])) if off.any() else None,
        "flag_when_right": float(np.mean(flagged[right])) if right.any() else None,
        "t0": float(t0),
    }
    json.dump(s, open(os.path.join(run, "score.json"), "w"), indent=1, ensure_ascii=False)
    return s


def group_of(run):
    return re.sub(r"_r\d+$", "", os.path.basename(os.path.normpath(run)))


def fmt(v, d=2):
    if v is None:
        return "—"
    return f"{v:.{d}f}"


def agg(vals):
    vals = [v for v in vals if v is not None]
    if not vals:
        return "—"
    if len(vals) == 1:
        return fmt(vals[0])
    return f"{fmt(float(np.median(vals)))} ({fmt(min(vals))}–{fmt(max(vals))})"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("ref_dir")
    ap.add_argument("runs", nargs="+", help="run dir (glob 可)")
    ap.add_argument("--out", default=None, help="summary の出力先 (既定: 最初の run の親)")
    ap.add_argument("--cov_thr", type=float, default=0.5, help="amcl/emcl2 の自己申告しきい値 sqrt(cov_xx+cov_yy) [m]")
    ap.add_argument("--lost_m", type=float, default=1.0, help="見失い判定 [m]")
    ap.add_argument("--conv_m", type=float, default=0.3, help="収束・正解判定 [m]")
    args = ap.parse_args()
    ref = np.loadtxt(os.path.join(args.ref_dir, "ref_base.tum"), ndmin=2)
    runs = sorted(set(sum([glob.glob(r) for r in args.runs], [])))
    runs = [r for r in runs if os.path.exists(os.path.join(r, "est.tum"))]
    scores = [score_run(r, ref, args) for r in runs]
    out = args.out or os.path.dirname(os.path.normpath(runs[0]))
    os.makedirs(out, exist_ok=True)
    json.dump(scores, open(os.path.join(out, "results.json"), "w"), indent=1, ensure_ascii=False)

    groups = {}
    for s in scores:
        groups.setdefault(s["group"], []).append(s)
    lines = [f"# 自己位置推定 再生評価 (基準: {args.ref_dir}, 見失い >{args.lost_m} m, 正解 <{args.conv_m} m, 自己申告 cov_thr {args.cov_thr} m)", "",
             "群の値は run の中央値 (最小–最大)。自己申告 = 実誤差 >lost のとき怪しいと言えた割合 / <conv のとき誤警報した割合", "",
             "| 構成 | runs | 収束 s | xy 中央値 m | xy p95 m | xy 最大 m | >0.5 m | >1 m | yaw 中央 ° | yaw p95 ° | 見失い 回/合計 s | 申告 当たり/誤警報 | 記録率 | 鮮度 中央/p95 s |",
             "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
    for g, ss in groups.items():
        ss = [s for s in ss if s.get("initialized")]
        if not ss:
            lines.append(f"| {g} | 0 | 初期化されず |")
            continue
        lines.append("| {g} | {n} | {conv} | {med} | {p95} | {mx} | {f05} | {f1} | {ym} | {yp} | {le} | {fl} | {cov} | {lat} |".format(
            lat=agg([s["latency_median_s"] for s in ss]) + " / " + agg([s["latency_p95_s"] for s in ss]),
            g=g, n=len(ss), conv=agg([s["converge_s"] for s in ss]), med=agg([s["median_xy_m"] for s in ss]),
            p95=agg([s["p95_xy_m"] for s in ss]), mx=agg([s["max_xy_m"] for s in ss]),
            f05=agg([100 * s["frac_over_0_5_m"] for s in ss]) + " %", f1=agg([100 * s["frac_over_lost"] for s in ss]) + " %",
            ym=agg([s["median_yaw_deg"] for s in ss]), yp=agg([s["p95_yaw_deg"] for s in ss]),
            le=agg([float(s["lost_episodes"]) for s in ss]) + " / " + agg([s["lost_total_s"] for s in ss]),
            fl=agg([None if s["flag_when_off"] is None else 100 * s["flag_when_off"] for s in ss]) + " % / "
               + agg([None if s["flag_when_right"] is None else 100 * s["flag_when_right"] for s in ss]) + " %",
            cov=agg([100 * s["coverage"] for s in ss]) + " %"))
    lines += ["", "## run ごと", "", "| run | 収束 s | xy 中央値 | p95 | 最大 | >1 m | 見失い区間 (initialpose からの秒) |", "|---|---|---|---|---|---|---|"]
    for s in scores:
        if not s.get("initialized"):
            lines.append(f"| {os.path.basename(s['run'])} | 初期化されず |")
            continue
        lines.append(f"| {os.path.basename(s['run'])} | {fmt(s['converge_s'], 1)} | {fmt(s['median_xy_m'])} | {fmt(s['p95_xy_m'])} | "
                     f"{fmt(s['max_xy_m'])} | {100 * s['frac_over_lost']:.1f} % | {s['lost_list']} |")
    open(os.path.join(out, "summary.md"), "w").write("\n".join(lines) + "\n")
    print("\n".join(lines))


if __name__ == "__main__":
    main()
