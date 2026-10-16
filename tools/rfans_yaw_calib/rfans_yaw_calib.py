#!/usr/bin/env python3
# claude: 2026-10-16 新設 — R-Fans の方位ゼロ (yaw) を車体の進行方向から較正する。
#
#   原理: 差動二輪は横滑りできないので「車体が実際に進む向き」= base_link の x 軸。直進中の連続スキャン同士を
#         2D ICP で合わせると、LiDAR 座標系 (正確には scan が表現されている frame) での移動ベクトルが出る。
#         その角度 θ_m が 0 でなければ、scan の frame は車体 x から −θ_m だけ回っている → URDF の rfans yaw に −θ_m を足す。
#   同時に urg (/urg_front/scan、frame laser) を同時刻の R-Fans スキャンに ICP で合わせ、urg の R-Fans に対する相対角 δ を出す。
#         urg の新 yaw = 現 yaw + δ + (R-Fans の補正)。つまり「R-Fans を回すなら urg も同じ分回す」が自動で入る。
#   背景: docs/issue/2026-10-16_amcl_lateral_bias_rfans_yaw_offset.md (ε ≈ 5.6° が AMCL の左 0.3〜0.4 m バイアスの原因)。
#
#   入力 bag の条件: 直進 10〜20 m (多少曲がっても可、1 s あたりのヨー変化 <1.5° の区間だけ使う)、建物・壁が視野にあること。
#   使い方 (コンテナ内, ROS source 済み):
#     python3 tools/rfans_yaw_calib/rfans_yaw_calib.py <bag_dir> [--rfans /rfans/scan_all] [--urg /urg_front/scan] [--out calib.json]
#   出力: 画面に表 + JSON (現 yaw / 補正 / 新 yaw / 統計)。URDF の書き換えは手動 (印字される rpy を貼る)。
import argparse, json, math, os, sys
import numpy as np
from scipy.spatial import cKDTree
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions, StorageFilter
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage


def st(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def quat_xy_block(q):
    """3D 回転行列の xy ブロックと、base z 軸まわりの yaw (roll π 取付でも x 軸の像の向きで定義)"""
    x, y, z, w = q.x, q.y, q.z, q.w
    R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w)], [2 * (x * y + z * w), 1 - 2 * (x * x + z * z)]])
    return R, math.atan2(R[1, 0], R[0, 0])


def scan_points(m, rmin, rmax, nmax):
    ang = m.angle_min + np.arange(len(m.ranges)) * m.angle_increment
    rg = np.array(m.ranges, dtype=np.float64)
    ok = np.isfinite(rg) & (rg > rmin) & (rg < rmax)
    p = np.c_[rg[ok] * np.cos(ang[ok]), rg[ok] * np.sin(ang[ok])]
    if len(p) > nmax:
        p = p[np.linspace(0, len(p) - 1, nmax).astype(int)]
    return p


def icp2d(src, dst, iters=30, max_d=0.5, tol=1e-5):
    """src を dst に合わせる剛体変換 (theta, t): dst ≈ R(theta) src + t。戻り: theta, t, rms, inlier 率"""
    tree = cKDTree(dst)
    th = 0.0; t = np.zeros(2)
    for _ in range(iters):
        c, s = math.cos(th), math.sin(th)
        P = src @ np.array([[c, -s], [s, c]]).T + t
        d, j = tree.query(P)
        ok = d < max_d
        if ok.sum() < 30:
            return None
        A = src[ok]; B = dst[j[ok]]
        ca, cb = A.mean(0), B.mean(0)
        H = (A - ca).T @ (B - cb)
        U, _, Vt = np.linalg.svd(H)
        D = np.diag([1, np.sign(np.linalg.det(Vt.T @ U.T))])
        R = Vt.T @ D @ U.T
        th_new = math.atan2(R[1, 0], R[0, 0]); t_new = cb - R @ ca
        done = abs(wrap(th_new - th)) < tol and np.linalg.norm(t_new - t) < tol
        th, t = th_new, t_new
        if done:
            break
    c, s = math.cos(th), math.sin(th)
    P = src @ np.array([[c, -s], [s, c]]).T + t
    d, _ = tree.query(P); ok = d < max_d
    return th, t, float(np.sqrt((d[ok] ** 2).mean())), float(ok.mean())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag"); ap.add_argument("--rfans", default="/rfans/scan_all"); ap.add_argument("--urg", default="/urg_front/scan")
    ap.add_argument("--pair_dt", type=float, default=1.0, help="ε 用のスキャン対の時間差 [s]")
    ap.add_argument("--min_move", type=float, default=0.2); ap.add_argument("--max_move", type=float, default=1.2)
    ap.add_argument("--max_rot_deg", type=float, default=1.5, help="対の間のヨー変化がこれ以上の対は捨てる (旋回)")
    ap.add_argument("--rmax", type=float, default=30.0); ap.add_argument("--npts", type=int, default=800)
    ap.add_argument("--out", default=None); ap.add_argument("--no_urg", action="store_true")
    a = ap.parse_args()

    r = SequentialReader(); r.open(StorageOptions(uri=a.bag, storage_id="mcap"), ConverterOptions("", ""))
    r.set_filter(StorageFilter(topics=["/tf_static", a.rfans, a.urg]))
    rf, ug = [], []; tf = {}
    while r.has_next():
        tp, d, ts = r.read_next()
        if tp == "/tf_static":
            for t in deserialize_message(d, TFMessage).transforms:
                if t.header.frame_id == "base_link":
                    tf[t.child_frame_id] = t.transform
        elif tp == a.rfans:
            m = deserialize_message(d, LaserScan); rf.append((st(m.header), m))
        else:
            m = deserialize_message(d, LaserScan); ug.append((st(m.header), m))
    if not rf:
        sys.exit(f"no scans on {a.rfans}")
    rf_frame = rf[0][1].header.frame_id
    cur = {}
    for name in ("rfans", "laser"):
        if name in tf:
            R, yaw = quat_xy_block(tf[name].rotation); cur[name] = {"yaw": yaw, "R": R, "t": np.array([tf[name].translation.x, tf[name].translation.y])}
    print(f"bag: {a.bag}\n  {a.rfans}: {len(rf)} scans, frame={rf_frame}   {a.urg}: {len(ug)} scans")
    print("  現 URDF (tf_static): " + ", ".join(f"{k} yaw {math.degrees(v['yaw']):+.2f}°" for k, v in cur.items()))
    # R-Fans スキャンが rfans frame で出ている場合 (/rfans/scan は base_link なので通常は不要) は base_link に直す
    rf_R = cur["rfans"]["R"] if (rf_frame == "rfans" and "rfans" in cur) else np.eye(2)
    rf_t = cur["rfans"]["t"] if (rf_frame == "rfans" and "rfans" in cur) else np.zeros(2)

    # ---- ε: 連続スキャン対の ICP ----
    ts = np.array([x[0] for x in rf]); pts_cache = {}

    def P(i):
        if i not in pts_cache:
            pts_cache[i] = scan_points(rf[i][1], 1.0, a.rmax, a.npts) @ rf_R.T + rf_t
        return pts_cache[i]

    dirs, rots, moves = [], [], []
    i = 0
    while i < len(rf):
        j = int(np.searchsorted(ts, ts[i] + a.pair_dt))
        if j >= len(rf):
            break
        res = icp2d(P(j), P(i), max_d=0.5)
        if res is not None:
            th, t, rms, inl = res; mv = float(np.linalg.norm(t))
            if a.min_move <= mv <= a.max_move and abs(math.degrees(th)) <= a.max_rot_deg and inl > 0.5 and rms < 0.12:
                dirs.append(math.atan2(t[1], t[0])); rots.append(th); moves.append(mv)
        i = j
    dirs = np.array(dirs)
    if len(dirs) < 5:
        sys.exit(f"直進対が {len(dirs)} 本しか取れない (直進 10 m 以上・建物が見える場所で録り直し)")
    # 平均は角度の円平均、ばらつきは中央値まわりの MAD ベース
    mean_dir = math.atan2(np.sin(dirs).mean(), np.cos(dirs).mean())
    dev = wrap(dirs - mean_dir); med_dir = wrap(mean_dir + np.median(dev)); sd = 1.4826 * np.median(np.abs(dev - np.median(dev)))
    good = np.abs(dev - np.median(dev)) < 3 * max(sd, 1e-3)
    mean_dir_g = math.atan2(np.sin(dirs[good]).mean(), np.cos(dirs[good]).mean()); sem = dirs[good].std() / math.sqrt(good.sum())
    corr = -mean_dir_g   # scan frame を車体 x に揃える補正 (yaw に足す)
    out = {"bag": a.bag, "rfans_topic": a.rfans, "rfans_frame": rf_frame,
           "rfans": {"cur_yaw_rad": cur.get("rfans", {}).get("yaw", 0.0), "motion_dir_in_scan_frame_deg": math.degrees(mean_dir_g),
                     "motion_dir_median_deg": math.degrees(med_dir), "sem_deg": math.degrees(sem), "sd_deg": math.degrees(dirs[good].std()),
                     "n_pairs": int(good.sum()), "n_rejected": int((~good).sum()),
                     "correction_rad": corr, "new_yaw_rad": cur.get("rfans", {}).get("yaw", 0.0) + corr}}
    print(f"\n[ε] 直進対 {good.sum()} 本 (外れ {(~good).sum()}): scan frame で見た移動方向 {math.degrees(mean_dir_g):+.2f}° (中央値 {math.degrees(med_dir):+.2f}, sd {math.degrees(dirs[good].std()):.2f}, sem {math.degrees(sem):.2f})")
    print(f"     → rfans_joint yaw の補正 {math.degrees(corr):+.2f}° : 現 {math.degrees(out['rfans']['cur_yaw_rad']):+.3f}° → 新 {math.degrees(out['rfans']['new_yaw_rad']):+.3f}° ({out['rfans']['new_yaw_rad']:+.5f} rad)")

    # ---- δ: urg → R-Fans の相対角 ----
    if ug and not a.no_urg and "laser" in cur:
        Rl, tl = cur["laser"]["R"], cur["laser"]["t"]; uts = np.array([x[0] for x in ug]); deltas, shifts = [], []
        k = 0
        for i in range(0, len(rf), max(1, len(rf) // 300)):
            k = int(np.argmin(np.abs(uts - ts[i])))
            if abs(uts[k] - ts[i]) > 0.03:
                continue
            pu = scan_points(ug[k][1], 0.5, 20.0, 600) @ Rl.T + tl
            res = icp2d(pu, P(i), max_d=0.3)
            if res is None:
                continue
            th, t, rms, inl = res
            if inl > 0.4 and rms < 0.12 and np.linalg.norm(t) < 0.3:
                deltas.append(th); shifts.append(t)
        deltas = np.array(deltas)
        if len(deltas) >= 5:
            md = np.median(deltas); sdd = 1.4826 * np.median(np.abs(deltas - md)); g = np.abs(deltas - md) < 3 * max(sdd, 1e-3)
            dmean = deltas[g].mean(); dsem = deltas[g].std() / math.sqrt(g.sum()); sh = np.array(shifts)[g].mean(0)
            new_laser = cur["laser"]["yaw"] + dmean + corr
            out["laser"] = {"cur_yaw_rad": cur["laser"]["yaw"], "delta_to_rfans_deg": math.degrees(dmean), "sem_deg": math.degrees(dsem), "n": int(g.sum()),
                            "mean_shift_m": sh.tolist(), "new_yaw_rad": new_laser}
            print(f"\n[δ] urg→R-Fans の相対角 {math.degrees(dmean):+.2f}° (sem {math.degrees(dsem):.2f}, n {g.sum()}, 平行移動 {sh[0]:+.3f},{sh[1]:+.3f} m)")
            print(f"     → laser_joint yaw: 現 {math.degrees(cur['laser']['yaw']):+.3f}° + δ {math.degrees(dmean):+.2f}° + R-Fans 補正 {math.degrees(corr):+.2f}° = 新 {math.degrees(new_laser):+.3f}° ({new_laser:+.5f} rad)")
        else:
            print(f"\n[δ] urg の対が {len(deltas)} 本で不足 (壁が両方に見える区間が要る)")
    print("\nURDF に書く値 (rpy の yaw だけ差し替え。roll/pitch は現行のまま):")
    print(f"  rfans_joint: yaw = {out['rfans']['new_yaw_rad']:+.5f}")
    if "laser" in out:
        print(f"  laser_joint: yaw = {out['laser']['new_yaw_rad']:+.5f}   (roll π 取付でも base の z まわり反時計回りが正)")
    print("  その後 GLIM の T_lidar_imu を tools/rfans_yaw_calib/glim_t_lidar_imu.py で再計算すること")
    if a.out:
        json.dump(out, open(a.out, "w"), indent=1); print(f"  JSON: {a.out}")


if __name__ == "__main__":
    main()
