#!/usr/bin/env python3
# claude: 2026-10-11 新設 — 基準軌跡を「地図にスキャンが最も乗る姿勢」で補正する (別 bag を評価するときの基準作り)。
#
#   背景: 別 bag (1611) の GLIM 軌跡を 2D 地図同士の ICP で地図 bag (1643) の座標へ剛体変換したところ、
#   2 本の GLIM 地図は形が少し違う (開始付近で約 1 m、区間によって向きも違う) ため剛体変換では 1〜2 m 残った
#   (10-11 実測: scan_fit で基準姿勢の hit 0.55〜0.62、emcl2 の推定姿勢は 0.88〜0.94)。
#   評価で測りたいのは「地図に対する位置」なので、基準も地図基準で定義する:
#     各キーフレーム (1 s ごと) の /rfans/scan を、GLIM 軌跡の姿勢の周り (±3 m, ±6°) で総当たりし、
#     地図の占有セルからの距離 (距離変換) の平均が最小の姿勢を採る → 補正量 (dx, dy, dyaw) を得る。
#     補正量は GLIM の相対運動が正確な短い区間では滑らかに変わるはずなので、中央値フィルタで外れ値
#     (廊下方向の退化・動体) を除き、キーフレーム間は線形補間して全行に適用する。
#   出力の hit 率と補正量の変化が、この基準自身の信頼度 (= 測れる誤差の下限) になる。
#
# 使い方 (コンテナ内、rosbag2_py が要るので source /opt/ros/jazzy/setup.bash してから):
#   python3 refine_reference.py <bag_dir> <ref_dir (make_reference.py の出力)> <map.yaml> <out_dir>
#       [--scan /rfans/scan] [--dt 1.0] [--search_xy 3.0] [--search_yaw 6.0] [--trunc 0.5]
#   出力: <out_dir>/ref_base.tum (補正後) + ref_glim.tum (補正前のコピー) + initialpose.yaml + corrections.csv
#         + refine.png (補正量と hit の時系列) + refine.json (統計)
import argparse
import json
import os
import shutil

import cv2
import numpy as np
import yaml
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from sensor_msgs.msg import LaserScan


def quat_yaw(q):
    x, y, z, w = q
    return np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def yaw_quat(yaw):
    return np.array([0.0, 0.0, np.sin(yaw / 2), np.cos(yaw / 2)])


def read_keyframe_scans(bag, topic, times, tol):
    """times の各時刻に最も近いスキャン (dict k -> LaserScan)。bag を 1 回だけ順に読む"""
    r = SequentialReader()
    r.open(StorageOptions(uri=bag, storage_id="mcap"), ConverterOptions("", ""))
    t0, dt, n = times[0], times[1] - times[0], len(times)
    best = {}
    while r.has_next():
        tp, data, _ = r.read_next()
        if tp != topic:
            continue
        m = deserialize_message(data, LaserScan)
        st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        k = int(round((st - t0) / dt))
        if k < 0 or k >= n:
            continue
        e = abs(st - times[k])
        if e < tol and (k not in best or e < best[k][0]):
            best[k] = (e, m)
    return {k: m for k, (_, m) in best.items()}


def body_points(scan, rmax):
    a = scan.angle_min + np.arange(len(scan.ranges)) * scan.angle_increment
    r = np.array(scan.ranges)
    ok = np.isfinite(r) & (r > scan.range_min) & (r < min(rmax, scan.range_max))
    return np.c_[r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])]


class DistMap:
    """占有格子の距離変換 (各セルから最寄り占有セルまでの距離 [m]、trunc で頭打ち)"""

    def __init__(self, map_yaml, trunc):
        y = yaml.safe_load(open(map_yaml))
        img = cv2.imread(os.path.join(os.path.dirname(map_yaml), y["image"]), cv2.IMREAD_GRAYSCALE)
        self.res, self.ox, self.oy = y["resolution"], y["origin"][0], y["origin"][1]
        self.H, self.W = img.shape
        occ = (255 - img.astype(np.float32)) / 255.0 > y["occupied_thresh"]
        free = (~occ).astype(np.uint8)
        self.D = np.minimum(cv2.distanceTransform(free, cv2.DIST_L2, 5) * self.res, trunc).astype(np.float32)
        self.trunc = trunc

    def lookup(self, pts):
        """pts (..., 2) → 距離 (...)。地図外は trunc"""
        c = np.floor((pts[..., 0] - self.ox) / self.res).astype(np.int64)
        r = (self.H - 1 - np.floor((pts[..., 1] - self.oy) / self.res)).astype(np.int64)
        ok = (c >= 0) & (c < self.W) & (r >= 0) & (r < self.H)
        out = np.full(pts.shape[:-1], self.trunc, np.float32)
        out[ok] = self.D[r[ok], c[ok]]
        return out


def _rigid2d(src, dst):
    """src→dst の最小二乗剛体 (R, t)"""
    cs, cd = src.mean(0), dst.mean(0)
    Hm = (src - cs).T @ (dst - cd)
    U, _, Vt = np.linalg.svd(Hm)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    R = Vt.T @ np.diag([1, d]) @ U.T
    return R, cd - R @ cs


def search(dm, body, x, y, yaw, dxy, dyaw_deg, step_xy, step_yaw_deg):
    """(x,y,yaw) の周り ±dxy / ±dyaw を総当たりし、(best (dx,dy,dyaw), best score, 1 m 以上離れた次善の score)"""
    offs = np.arange(-dxy, dxy + 1e-9, step_xy)
    yaws = np.radians(np.arange(-dyaw_deg, dyaw_deg + 1e-9, step_yaw_deg))
    gx, gy = np.meshgrid(offs, offs, indexing="ij")
    grid = np.c_[gx.ravel(), gy.ravel()]  # (G, 2)
    best = (None, np.inf)
    scores = np.empty((len(yaws), len(grid)), np.float32)
    for i, dy_ in enumerate(yaws):
        a = yaw + dy_
        R = np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]])
        w = body @ R.T + np.array([x, y])  # (N, 2)
        pts = w[None, :, :] + grid[:, None, :]  # (G, N, 2)
        s = dm.lookup(pts).mean(axis=1)
        scores[i] = s
    i, j = np.unravel_index(np.argmin(scores), scores.shape)
    bdx, bdy, byaw = grid[j, 0], grid[j, 1], yaws[i]
    far = np.hypot(grid[:, 0] - bdx, grid[:, 1] - bdy) > 1.0
    second = float(scores[:, far].min()) if far.any() else np.inf
    return (float(bdx), float(bdy), float(byaw)), float(scores[i, j]), second


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag"); ap.add_argument("ref_dir"); ap.add_argument("map_yaml"); ap.add_argument("out_dir")
    ap.add_argument("--scan", default="/rfans/scan")
    ap.add_argument("--dt", type=float, default=1.0, help="キーフレーム間隔 [s]")
    ap.add_argument("--rmax", type=float, default=30.0)
    ap.add_argument("--trunc", type=float, default=0.5, help="距離の頭打ち [m] (外れ点の影響を抑える)")
    ap.add_argument("--search_xy", type=float, default=3.0)
    ap.add_argument("--search_yaw", type=float, default=6.0)
    ap.add_argument("--hit_dist", type=float, default=0.3)
    ap.add_argument("--min_hit", type=float, default=0.6, help="これ未満のキーフレームは補正に使わない")
    ap.add_argument("--min_margin", type=float, default=0.05,
                    help="次善 (1 m 以上離れた姿勢) との score 差の比がこれ未満 = 退化とみなし補正に使わない")
    ap.add_argument("--median_win", type=int, default=7, help="1 段目の外れ値判定に使う中央値フィルタ窓 (キーフレーム数)")
    ap.add_argument("--outlier_xy", type=float, default=0.3, help="中央値からこれ以上離れた補正は外れ値 [m]")
    ap.add_argument("--trend_win", type=int, default=31,
                    help="2 段目の探索中心 (傾向) を出す中央値窓 [キーフレーム数]。GLIM の相対運動がこの長さで十分正確なこと")
    ap.add_argument("--local_xy", type=float, default=0.6, help="2 段目: 傾向の周りの探索範囲 ±[m]")
    ap.add_argument("--local_yaw", type=float, default=2.0, help="2 段目: 傾向の周りの探索範囲 ±[deg]")
    args = ap.parse_args()
    os.makedirs(args.out_dir, exist_ok=True)

    ref = np.loadtxt(os.path.join(args.ref_dir, "ref_base.tum"), ndmin=2)
    times = np.arange(ref[0, 0], ref[-1, 0], args.dt)
    print(f"ref {len(ref)} rows, {len(times)} keyframes every {args.dt} s; reading {args.scan} ...")
    scans = read_keyframe_scans(args.bag, args.scan, times, tol=args.dt / 2)
    print(f"  {len(scans)} keyframes have a scan")
    dm = DistMap(args.map_yaml, args.trunc)

    def ref_pose(t):
        i = np.searchsorted(ref[:, 0], t).clip(1, len(ref) - 1)
        a, b = ref[i - 1], ref[i]
        f = (t - a[0]) / max(1e-6, b[0] - a[0])
        ya, yb = quat_yaw(a[4:8]), quat_yaw(b[4:8])
        return a[1] + f * (b[1] - a[1]), a[2] + f * (b[2] - a[2]), ya + f * np.arctan2(np.sin(yb - ya), np.cos(yb - ya))

    def hit_of(body, x, y, yaw):
        R = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
        return float((dm.lookup(body @ R.T + np.array([x, y])) < args.hit_dist).mean())

    # ---- 1 段目: 広い範囲の総当たり (粗 0.2 m/1° → 細 0.05 m/0.25°)。周期構造 (並木) や廊下方向の退化で
    #      1〜3 m 飛ぶ候補が混じるので、ここでは「傾向」を出すだけ
    rows = []  # k, t, dx, dy, dyaw, score, hit, margin
    bodies = {}
    for k in range(len(times)):
        if k not in scans:
            continue
        t = times[k]
        x, y, yaw = ref_pose(t)
        body = body_points(scans[k], args.rmax)
        if len(body) < 50:
            continue
        bodies[k] = body
        sub = body[:: max(1, len(body) // 400)]
        (dx, dy, dyaw), s1, s2 = search(dm, sub, x, y, yaw, args.search_xy, args.search_yaw, 0.2, 1.0)
        (fx, fy, fyaw), s1, _ = search(dm, body, x + dx, y + dy, yaw + dyaw, 0.3, 1.5, 0.05, 0.25)
        dx, dy, dyaw = dx + fx, dy + fy, dyaw + fyaw
        hit = hit_of(body, x + dx, y + dy, yaw + dyaw)
        margin = (s2 - s1) / max(s1, 1e-6)
        rows.append((k, t, dx, dy, dyaw, s1, hit, margin))
        if len(rows) % 100 == 0:
            print(f"  pass1 {len(rows)} keyframes (t+{t - times[0]:.0f}s): dx {dx:+.2f} dy {dy:+.2f} dyaw {np.degrees(dyaw):+.2f} hit {hit:.2f}")
    C = np.array(rows)
    kk, tt, DX, DY, DYAW, S, HIT, MARG = C.T

    def medfilt(v, m, win):
        out = v.copy(); idx = np.nonzero(m)[0]; h = win // 2
        for n, i in enumerate(idx):
            lo, hi = max(0, n - h), min(len(idx), n + h + 1)
            out[i] = np.median(v[idx[lo:hi]])
        return out

    good = (HIT >= args.min_hit) & (MARG >= args.min_margin)
    mDX, mDY, mDYAW = medfilt(DX, good, args.median_win), medfilt(DY, good, args.median_win), medfilt(DYAW, good, args.median_win)
    used1 = good & (np.hypot(DX - mDX, DY - mDY) <= args.outlier_xy) & (np.abs(DYAW - mDYAW) <= np.radians(1.0))
    if used1.sum() < 10:
        raise SystemExit(f"usable keyframes too few after pass 1: {used1.sum()}")
    # 傾向 = 1 段目の採用点を広い窓で中央値 → 全キーフレームへ補間
    tDX, tDY, tDYAW = medfilt(DX, used1, args.trend_win), medfilt(DY, used1, args.trend_win), medfilt(DYAW, used1, args.trend_win)
    TDX = np.interp(tt, tt[used1], tDX[used1]); TDY = np.interp(tt, tt[used1], tDY[used1])
    TDYAW = np.interp(tt, tt[used1], np.unwrap(tDYAW[used1]))

    # ---- 2 段目: 傾向の周り ±local_xy / ±local_yaw だけを細かく再探索 (範囲の端に張り付いた結果は不採用)
    DX2, DY2, DYAW2, HIT2 = DX.copy(), DY.copy(), DYAW.copy(), HIT.copy()
    for n in range(len(C)):
        k, t = int(kk[n]), tt[n]
        x, y, yaw = ref_pose(t)
        (fx, fy, fyaw), s, _ = search(dm, bodies[k], x + TDX[n], y + TDY[n], yaw + TDYAW[n],
                                      args.local_xy, args.local_yaw, 0.05, 0.25)
        DX2[n], DY2[n], DYAW2[n] = TDX[n] + fx, TDY[n] + fy, TDYAW[n] + fyaw
        HIT2[n] = hit_of(bodies[k], x + DX2[n], y + DY2[n], yaw + DYAW2[n])
        if (n + 1) % 200 == 0:
            print(f"  pass2 {n + 1}/{len(C)}")
    at_edge = (np.abs(DX2 - TDX) >= args.local_xy - 1e-6) | (np.abs(DY2 - TDY) >= args.local_xy - 1e-6) \
        | (np.abs(DYAW2 - TDYAW) >= np.radians(args.local_yaw) - 1e-6)
    used = (HIT2 >= args.min_hit) & ~at_edge
    DX, DY, DYAW, HIT = DX2, DY2, DYAW2, HIT2
    mDX, mDY = medfilt(DX, used, args.median_win), medfilt(DY, used, args.median_win)
    if used.sum() < 10:
        raise SystemExit(f"usable keyframes too few after pass 2: {used.sum()}")
    cdx = np.interp(ref[:, 0], tt[used], DX[used]); cdy = np.interp(ref[:, 0], tt[used], DY[used])
    cyaw = np.interp(ref[:, 0], tt[used], np.unwrap(DYAW[used]))

    out = ref.copy()
    out[:, 1] += cdx; out[:, 2] += cdy
    for i in range(len(out)):
        out[i, 4:8] = yaw_quat(quat_yaw(ref[i, 4:8]) + cyaw[i])
    shutil.copy(os.path.join(args.ref_dir, "ref_base.tum"), os.path.join(args.out_dir, "ref_glim.tum"))
    np.savetxt(os.path.join(args.out_dir, "ref_base.tum"), out,
               fmt=["%.9f", "%.6f", "%.6f", "%.6f", "%.9f", "%.9f", "%.9f", "%.9f"])
    init_src = yaml.safe_load(open(os.path.join(args.ref_dir, "initialpose.yaml")))
    init = dict(init_src)
    init.update({"x": float(out[0, 1]), "y": float(out[0, 2]), "yaw": float(quat_yaw(out[0, 4:8])),
                 "refined_from": os.path.abspath(args.ref_dir), "map": os.path.abspath(args.map_yaml)})
    yaml.safe_dump(init, open(os.path.join(args.out_dir, "initialpose.yaml"), "w"), sort_keys=False)
    with open(os.path.join(args.out_dir, "corrections.csv"), "w") as f:
        f.write("t dx dy dyaw_deg hit used trend_dx trend_dy trend_dyaw_deg pass1_dx pass1_dy pass1_margin\n")
        for i in range(len(C)):
            f.write(f"{tt[i]:.3f} {DX[i]:.3f} {DY[i]:.3f} {np.degrees(DYAW[i]):.3f} {HIT[i]:.3f} {int(used[i])} "
                    f"{TDX[i]:.3f} {TDY[i]:.3f} {np.degrees(TDYAW[i]):.3f} {C[i, 2]:.3f} {C[i, 3]:.3f} {MARG[i]:.3f}\n")
    # 補正量が位置の剛体変換 1 つで説明できるか (claude: critic 10-11 提案 — θ が 0 でなければ入力の登録 (ICP) が誤っている。
    #   残差 p95 が「2 走行の地図の非剛体ずれ」の上限)
    P = np.array([ref_pose(t)[:2] for t in tt[used]]); Q = P + np.c_[DX[used], DY[used]]
    Rr, tr_ = _rigid2d(P, Q)
    rigid_theta = float(np.degrees(np.arctan2(Rr[1, 0], Rr[0, 0])))
    rigid_res = np.hypot(*((P @ Rr.T + tr_) - Q).T)
    stats = {"keyframes": int(len(C)), "used": int(used.sum()), "used_pass1": int(used1.sum()),
             "rigid_fit_theta_deg": rigid_theta, "rigid_fit_t_m": [float(tr_[0]), float(tr_[1])],
             "residual_after_rigid_median_m": float(np.median(rigid_res)),
             "residual_after_rigid_p95_m": float(np.percentile(rigid_res, 95)),
             "hit_median": float(np.median(HIT)),
             "hit_p05": float(np.percentile(HIT, 5)),
             "corr_xy_median_m": float(np.median(np.hypot(DX[used], DY[used]))),
             "corr_xy_max_m": float(np.hypot(DX[used], DY[used]).max()),
             "corr_yaw_median_deg": float(np.degrees(np.median(np.abs(DYAW[used])))),
             "residual_xy_vs_smoothed_p95_m": float(np.percentile(np.hypot(DX[used] - mDX[used], DY[used] - mDY[used]), 95)),
             "dev_from_trend_p95_m": float(np.percentile(np.hypot(DX[used] - TDX[used], DY[used] - TDY[used]), 95)),
             "map": os.path.abspath(args.map_yaml), "ref_dir": os.path.abspath(args.ref_dir)}
    json.dump(stats, open(os.path.join(args.out_dir, "refine.json"), "w"), indent=1)
    print("stats:", json.dumps(stats))

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axs = plt.subplots(4, 1, figsize=(14, 10), sharex=True)
    tr = tt - tt[0]
    for ax, v, p1, tr_, lab in zip(axs[:3], (DX, DY, np.degrees(DYAW)), (C[:, 2], C[:, 3], np.degrees(C[:, 4])),
                                   (TDX, TDY, np.degrees(TDYAW)), ("dx [m]", "dy [m]", "dyaw [deg]")):
        ax.plot(tr, p1, "x", color="0.75", ms=3, label="pass 1 (wide search)")
        ax.plot(tr, tr_, "-", color="tab:orange", lw=1, label="trend (median %d)" % args.trend_win)
        ax.plot(tr[~used], v[~used], "x", color="tab:red", ms=4, label="pass 2 not used")
        ax.plot(tr[used], v[used], ".", color="tab:blue", ms=4, label="pass 2 used")
        ax.set_ylabel(lab); ax.grid(alpha=0.3)
    axs[0].legend(loc="upper right"); axs[0].set_title(f"reference correction vs GLIM (scan fit to {os.path.basename(os.path.dirname(args.map_yaml))}): used {used.sum()}/{len(C)}")
    axs[3].plot(tr, HIT, ".", ms=3, color="tab:green"); axs[3].set_ylabel("hit (<0.3 m)"); axs[3].set_ylim(0, 1.02)
    axs[3].set_xlabel("t from ref start [s]"); axs[3].grid(alpha=0.3)
    plt.tight_layout(); plt.savefig(os.path.join(args.out_dir, "refine.png"), dpi=90)


if __name__ == "__main__":
    main()
