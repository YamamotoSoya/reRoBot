#!/usr/bin/env python3
# claude: 2026-10-01 新設。
# AMCL 並走比較の評価。replay.sh が記録した /loc_<name>/amcl_pose と /scan_<name> を、元 bag の
# /odometry/filtered・/map・/scan (base 用) と突き合わせ、変種ごとに 3 指標を出す。
#   1. base との差      : 同時刻 (最近傍) の位置差 [m]・yaw 差 [deg]
#   2. 飛び             : 連続する amcl_pose 間の移動量が odom 移動量と 0.5 m 以上 (or yaw 20°以上) 食い違う回数
#                         (/initialpose 直後 3 s は再初期化なので除外)。共分散トレース (xx+yy)
#   3. スキャン整合     : 推定姿勢 (amcl_pose と同 stamp のスキャン) で終点を地図に置き、最寄り占有セルまで
#                         ≤ 0.15 m の有限ビームの割合 (真値なしで「地図に合っているか」を測る)
# 使い方: compare.py <orig_bag> <replay_bag> <out_prefix> [--segments "名前:t0-t1,..."]
#   t は元 bag 先頭からの秒。図 <out_prefix>.png と表 <out_prefix>.md を書く。
import argparse, math, sys
import numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from scipy.ndimage import distance_transform_edt
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
import warnings; warnings.filterwarnings("ignore", message="Glyph")
from matplotlib import font_manager as _fm
_jp = [f.name for f in _fm.fontManager.ttflist if any(k in f.name for k in ("CJK", "IPA", "Noto Sans JP", "Droid Sans Fallback"))]
plt.rcParams["font.family"] = (_jp[:1] or []) + ["DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False  # Droid Sans Fallback に U+2212 が無い

TYPES = {"amcl_pose": PoseWithCovarianceStamped, "initialpose": PoseWithCovarianceStamped,
         "odometry/filtered": Odometry, "map": OccupancyGrid}

def yaw(q): return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
def stamp(h): return h.stamp.sec + h.stamp.nanosec * 1e-9
def wrap(a): return (a + np.pi) % (2 * np.pi) - np.pi

def open_bag(uri, topics):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=uri, storage_id="mcap"), rosbag2_py.ConverterOptions("", ""))
    avail = {t.name: t.type for t in r.get_all_topics_and_types()}
    r.set_filter(rosbag2_py.StorageFilter(topics=[t for t in topics if t in avail]))
    return r, avail

def msgtype(topic, typ):
    if typ.endswith("LaserScan"): return LaserScan
    for k, v in TYPES.items():
        if topic.endswith(k): return v
    raise KeyError(topic)

def read_all(uri, want):
    r, avail = open_bag(uri, want)
    out = {t: [] for t in want if t in avail}
    while r.has_next():
        t, d, ts = r.read_next()
        out[t].append(deserialize_message(d, msgtype(t, avail[t])))
    return out

def poses(msgs):
    a = np.array([(stamp(m.header), m.pose.pose.position.x, m.pose.pose.position.y, yaw(m.pose.pose.orientation),
                   m.pose.covariance[0] + m.pose.covariance[7]) for m in msgs])
    return a[np.argsort(a[:, 0])] if len(a) else np.zeros((0, 5))

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("orig"); ap.add_argument("replay"); ap.add_argument("out")
    ap.add_argument("--segments", default="")
    ap.add_argument("--hit", type=float, default=0.15)
    a = ap.parse_args()

    # --- 元 bag: odom / map / /scan / 先頭時刻
    r0, _ = open_bag(a.orig, ["/odometry/filtered"]); t, d, ts = r0.read_next(); bag_t0 = ts / 1e9
    orig = read_all(a.orig, ["/odometry/filtered", "/map", "/scan"])
    od = np.array([(stamp(m.header), m.pose.pose.position.x, m.pose.pose.position.y, yaw(m.pose.pose.orientation))
                   for m in orig["/odometry/filtered"]])
    gm = orig["/map"][0]; W, H, res = gm.info.width, gm.info.height, gm.info.resolution
    ox, oy = gm.info.origin.position.x, gm.info.origin.position.y
    grid = np.array(gm.data, dtype=np.int16).reshape(H, W)
    dist = distance_transform_edt(grid < 65) * res  # 最寄り占有セルまでの距離 [m]

    # --- replay bag
    r1, avail = open_bag(a.replay, [])
    names = sorted({t.split("/")[1][4:] for t in avail if t.startswith("/loc_") and t.endswith("/amcl_pose")})
    want = [f"/loc_{n}/amcl_pose" for n in names] + [t for t in avail if t.startswith("/scan_")] + ["/initialpose"]
    rep = read_all(a.replay, want)
    resets = np.array(sorted({stamp(m.header) for m in rep.get("/initialpose", [])})) if rep.get("/initialpose") else np.array([])
    # 再送 (同一 stamp) はまとめる。stamp が 0 のものは受信時刻が無いので除外
    resets = resets[resets > 0]

    # 変種 → スキャン topic (base は元 bag の /scan、allS は /scan_all を共有)
    def scan_topic(n):
        n = n.split("_r")[0]  # 反復 run (<group>_rN) は group のスキャンを共有
        if n == "base": return "/scan"
        if f"/scan_{n}" in rep: return f"/scan_{n}"
        if n.startswith("all"): return "/scan_all"
        return None
    scans = {"/scan": orig["/scan"], **{t: v for t, v in rep.items() if t.startswith("/scan_")}}
    scan_idx = {t: {round(stamp(m.header), 3): m for m in v} for t, v in scans.items()}

    def odom_at(ts):
        i = np.clip(np.searchsorted(od[:, 0], ts), 1, len(od) - 1)
        return od[i]

    def near_reset(ts, win=3.0):
        return len(resets) and np.min(np.abs(resets - ts)) < win

    P = {n: poses(rep[f"/loc_{n}/amcl_pose"]) for n in names}
    base = P.get("base")
    res_rows, series = [], {}
    for n in names:
        p = P[n]
        if len(p) < 2: res_rows.append((n, len(p))); continue
        # 1. base との差
        if base is not None and len(base):
            j = np.clip(np.searchsorted(base[:, 0], p[:, 0]), 0, len(base) - 1)
            dpos = np.hypot(p[:, 1] - base[j, 1], p[:, 2] - base[j, 2]); dyaw = np.degrees(np.abs(wrap(p[:, 3] - base[j, 3])))
        else:
            dpos = dyaw = np.zeros(len(p))
        # 2. 飛び
        jumps = []
        for k in range(1, len(p)):
            if near_reset(p[k, 0]): continue
            o0, o1 = odom_at(p[k - 1, 0]), odom_at(p[k, 0])
            da = math.hypot(p[k, 1] - p[k - 1, 1], p[k, 2] - p[k - 1, 2]); do = math.hypot(o1[1] - o0[1], o1[2] - o0[2])
            dy = abs(wrap(p[k, 3] - p[k - 1, 3]) - wrap(o1[3] - o0[3]))
            if abs(da - do) > 0.5 or math.degrees(dy) > 20: jumps.append(p[k, 0])
        # 3. スキャン整合
        st = scan_topic(n); score = np.full(len(p), np.nan)
        if st:
            idx = scan_idx[st]
            for k in range(len(p)):
                m = idx.get(round(p[k, 0], 3))
                if m is None: continue
                rr = np.asarray(m.ranges, dtype=float)
                th = m.angle_min + m.angle_increment * np.arange(len(rr))
                ok = np.isfinite(rr) & (rr < 30.0) & (rr > m.range_min)
                if ok.sum() < 10: continue
                c, s = math.cos(p[k, 3]), math.sin(p[k, 3])
                lx, ly = rr[ok] * np.cos(th[ok]), rr[ok] * np.sin(th[ok])
                gx = p[k, 1] + c * lx - s * ly; gy = p[k, 2] + s * lx + c * ly
                ix = ((gx - ox) / res).astype(int); iy = ((gy - oy) / res).astype(int)
                inside = (ix >= 0) & (ix < W) & (iy >= 0) & (iy < H)
                dd = np.full(len(ix), np.inf); dd[inside] = dist[iy[inside], ix[inside]]
                score[k] = np.mean(dd <= a.hit)
        series[n] = dict(t=p[:, 0] - bag_t0, dpos=dpos, dyaw=dyaw, cov=p[:, 4], score=score,
                         jumps=np.array(jumps) - bag_t0)

    # --- 区間
    segs = [("全体", -1e9, 1e9)]
    for s in [x for x in a.segments.split(",") if x]:
        nm, rg = s.split(":"); t0, t1 = map(float, rg.split("-")); segs.append((nm, t0, t1))

    lines = [f"# AMCL 変種比較\n\n- 元 bag: `{a.orig}`\n- 再生 bag: `{a.replay}`\n- 整合スコア = 地図の占有セルまで ≤ {a.hit} m の有限ビーム割合\n",
             "| 区間 | 変種 | pose 数 | base 差 中央 [m] | base 差 最大 [m] | 飛び回数 | cov 中央 | 整合 中央 | 整合 10%点 |",
             "|---|---|---|---|---|---|---|---|---|"]
    for nm, t0, t1 in segs:
        for n, S in series.items():
            m = (S["t"] >= t0) & (S["t"] <= t1)
            if not m.any(): continue
            sc = S["score"][m]; sc = sc[np.isfinite(sc)]
            nj = int(((S["jumps"] >= t0) & (S["jumps"] <= t1)).sum())
            lines.append(f"| {nm} | {n} | {m.sum()} | {np.median(S['dpos'][m]):.2f} | {S['dpos'][m].max():.2f} | {nj} | "
                         f"{np.median(S['cov'][m]):.3f} | {np.median(sc) if len(sc) else float('nan'):.3f} | "
                         f"{np.percentile(sc, 10) if len(sc) else float('nan'):.3f} |")
    for n, k in res_rows: lines.append(f"\n⚠ {n}: pose {k} 個しか無く評価不能")
    # --- 反復 run の集計 (変種名 <group>_rN)。コンセンサス = 全 run の位置の時刻ごと中央値
    groups = {}; devser = {}
    # 姿勢列が完全一致するコピー (同じ乱数種 + 同じ更新位相) は独立な試行ではないので 1 つにまとめる
    seen = {}
    for n in series:
        key = (n.split("_r")[0], P[n][:, 1:4].round(6).tobytes())
        if key in seen: dup_of = seen[key]; continue
        seen[key] = n; groups.setdefault(n.split("_r")[0], []).append(n)
    if any(len(v) > 1 for v in groups.values()):
        tg = np.arange(max(min(S["t"][0] for S in series.values()), -1e9), max(S["t"][-1] for S in series.values()), 0.5)
        XY = {}
        for n in series:
            p = P[n]; tt = p[:, 0] - bag_t0
            XY[n] = (np.interp(tg, tt, p[:, 1]), np.interp(tg, tt, p[:, 2]), (tg >= tt[0]) & (tg <= tt[-1]))
        cx = np.median(np.array([v[0] for v in XY.values()]), axis=0); cy = np.median(np.array([v[1] for v in XY.values()]), axis=0)
        for n in series: devser[n] = (tg[XY[n][2]], np.hypot(XY[n][0] - cx, XY[n][1] - cy)[XY[n][2]])
        lines += ["", "## 反復 run の集計 (コンセンサス = 全 run の時刻ごと中央値位置)", "",
                  "| 区間 | 変種 | 独立 run 数 (一致コピー除外後) | 飛んだ run | 飛び合計 | 1 m 超逸脱の run | 逸脱時間割合 | 最大逸脱 [m] | 整合 中央 |",
                  "|---|---|---|---|---|---|---|---|---|"]
        for nm, t0, t1 in segs:
            mt = (tg >= t0) & (tg <= t1)
            for g, ns in groups.items():
                jr = [int(((series[n]["jumps"] >= t0) & (series[n]["jumps"] <= t1)).sum()) for n in ns]
                dev = [np.hypot(XY[n][0] - cx, XY[n][1] - cy)[mt & XY[n][2]] for n in ns]
                lost_runs = sum(1 for d in dev if len(d) and d.max() > 1.0)
                frac = np.mean(np.concatenate([d > 1.0 for d in dev])) if any(len(d) for d in dev) else float("nan")
                mx = max((d.max() for d in dev if len(d)), default=float("nan"))
                sc = np.concatenate([series[n]["score"][(series[n]["t"] >= t0) & (series[n]["t"] <= t1)] for n in ns]); sc = sc[np.isfinite(sc)]
                lines.append(f"| {nm} | {g} | {len(ns)} | {sum(1 for j in jr if j)} | {sum(jr)} | {lost_runs} | {100*frac:.1f}% | {mx:.2f} | {np.median(sc) if len(sc) else float('nan'):.3f} |")
    open(a.out + ".md", "w").write("\n".join(lines) + "\n"); print("\n".join(lines))

    fig, ax = plt.subplots(4, 1, figsize=(14, 13), sharex=True)
    for n, S in series.items():
        if devser: ax[0].semilogy(devser[n][0], np.maximum(devser[n][1], 1e-2), "-", lw=0.8, label=n)
        else: ax[0].plot(S["t"], S["dpos"], ".-", ms=2, lw=0.8, label=n)
        ax[1].semilogy(S["t"], np.maximum(S["cov"], 1e-4), ".-", ms=2, lw=0.8, label=n)
        ax[2].plot(S["t"], S["score"], ".-", ms=2, lw=0.8, label=n)
        ax[3].plot(S["jumps"], np.full(len(S["jumps"]), list(series).index(n)), "x", label=n)
    for x in resets - bag_t0:
        for k in range(4): ax[k].axvline(x, color="gray", lw=0.5, ls=":")
    for nm, t0, t1 in segs[1:]:
        for k in range(4): ax[k].axvspan(t0, t1, color="orange", alpha=0.08)
    ax[0].set_ylabel("コンセンサスからの逸脱 [m]" if devser else "base との位置差 [m]"); ax[1].set_ylabel("cov xx+yy"); ax[2].set_ylabel("整合スコア")
    ax[3].set_yticks(range(len(series))); ax[3].set_yticklabels(list(series)); ax[3].set_ylabel("飛び")
    ax[3].set_xlabel("元 bag 先頭からの時刻 [s] (灰点線 = /initialpose)"); ax[0].legend(ncol=6, fontsize=8)
    for k in range(3): ax[k].grid(alpha=0.3)
    fig.tight_layout(); fig.savefig(a.out + ".png", dpi=110); print(f"\nwrote {a.out}.md / .png")

if __name__ == "__main__":
    main()
