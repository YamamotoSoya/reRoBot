#!/usr/bin/env python3
# claude: 2026-10-01 新設。
# ロボット静止区間のスキャンを地図に総当たりで当て、尤度場 (AMCL likelihood_field と同じ式) が最大の姿勢を探す。
# AMCL の推定・operator の initialpose のどちらが地図に合っているか、スコア面が通路方向に細長いか (退化) を見る。
# 使い方: brute_force_pose.py <bag> <t0_s> <t1_s> <cx> <cy> <cyaw_deg> [--others "名前:x,y,yaw;..."]
import argparse, math, numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from scipy.ndimage import distance_transform_edt

ap = argparse.ArgumentParser(); ap.add_argument("bag"); ap.add_argument("t0", type=float); ap.add_argument("t1", type=float)
ap.add_argument("cx", type=float); ap.add_argument("cy", type=float); ap.add_argument("cyaw", type=float)
ap.add_argument("--others", default=""); ap.add_argument("--sigma", type=float, default=0.2); ap.add_argument("--maxd", type=float, default=2.0)
ap.add_argument("--xy", type=float, default=2.0); ap.add_argument("--dxy", type=float, default=0.05)
ap.add_argument("--yaw", type=float, default=10.0); ap.add_argument("--dyaw", type=float, default=0.5)
a = ap.parse_args()
r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=a.bag, storage_id="mcap"), rosbag2_py.ConverterOptions("", ""))
r.set_filter(rosbag2_py.StorageFilter(topics=["/scan", "/map"])); scans = []; grid = None; bt0 = None
while r.has_next():
    t, d, ts = r.read_next(); bt0 = bt0 or ts; rel = (ts - bt0) / 1e9
    if t == "/map" and grid is None: grid = deserialize_message(d, OccupancyGrid)
    elif t == "/scan" and a.t0 <= rel <= a.t1: scans.append(deserialize_message(d, LaserScan))
    if rel > a.t1 and grid is not None: break
W, H, res = grid.info.width, grid.info.height, grid.info.resolution; ox, oy = grid.info.origin.position.x, grid.info.origin.position.y
occ = np.array(grid.data, dtype=np.int16).reshape(H, W) >= 65
dist = np.minimum(distance_transform_edt(~occ) * res, a.maxd)
lik = np.exp(-dist ** 2 / (2 * a.sigma ** 2))  # 尤度場 (z_hit 項のみ)
pts = []
for m in scans[:: max(1, len(scans) // 10)]:
    rr = np.asarray(m.ranges, float); th = m.angle_min + m.angle_increment * np.arange(len(rr))
    ok = np.isfinite(rr) & (rr < 30) & (rr > m.range_min); pts.append(np.stack([rr[ok] * np.cos(th[ok]), rr[ok] * np.sin(th[ok])], 1))
pts = np.concatenate(pts); print(f"scans {len(scans)} (使用 {min(10, len(scans))}), beams {len(pts)}")
offs = np.arange(-a.xy, a.xy + 1e-9, a.dxy); yaws = np.radians(a.cyaw + np.arange(-a.yaw, a.yaw + 1e-9, a.dyaw))
S = np.zeros((len(yaws), len(offs), len(offs)))
for k, yw in enumerate(yaws):
    c, s = math.cos(yw), math.sin(yw); gx = c * pts[:, 0] - s * pts[:, 1]; gy = s * pts[:, 0] + c * pts[:, 1]
    for i, dx in enumerate(offs):
        ix = ((gx + a.cx + dx - ox) / res).astype(int)
        for j, dy in enumerate(offs):
            iy = ((gy + a.cy + dy - oy) / res).astype(int)
            ins = (ix >= 0) & (ix < W) & (iy >= 0) & (iy < H)
            S[k, i, j] = lik[iy[ins], ix[ins]].sum() / len(pts)
k, i, j = np.unravel_index(S.argmax(), S.shape)
bx, by, byaw = a.cx + offs[i], a.cy + offs[j], math.degrees(yaws[k])
print(f"最良姿勢: ({bx:.2f}, {by:.2f}) yaw {byaw:.1f}°  score {S.max():.4f}   [探索中心 (operator) からの差 dx {offs[i]:+.2f} dy {offs[j]:+.2f} dyaw {byaw - a.cyaw:+.1f}°]")
def score_at(x, y, yawd):
    kk = np.argmin(abs(np.degrees(yaws) - yawd)); ii = np.argmin(abs(offs - (x - a.cx))); jj = np.argmin(abs(offs - (y - a.cy)))
    return S[kk, ii, jj], (abs(offs[ii] - (x - a.cx)) > a.dxy) or (abs(offs[jj] - (y - a.cy)) > a.dxy)
print(f"operator 姿勢のスコア: {score_at(a.cx, a.cy, a.cyaw)[0]:.4f}")
for item in [x for x in a.others.split(";") if x]:
    nm, v = item.split(":"); x, y, yd = map(float, v.split(",")); sc, out = score_at(x, y, yd)
    print(f"{nm} ({x:.2f},{y:.2f},{yd:.1f}°) のスコア: {sc:.4f}{'  (探索範囲外)' if out else ''}   最良からの距離 {math.hypot(x - bx, y - by):.2f} m")
# 最良 yaw でのスコア面の形: 最大値の 95% 以上の領域の主軸 (退化の向きと長さ)
P = S[k]; m95 = P >= 0.95 * P.max(); ii, jj = np.nonzero(m95)
X = np.stack([offs[ii], offs[jj]], 1); C = np.cov(X.T) if len(X) > 2 else np.eye(2) * 1e-6
w, V = np.linalg.eigh(C); major = V[:, 1]
print(f"95% 領域: {m95.sum()} セル, 主軸方向 {math.degrees(math.atan2(major[1], major[0])):.0f}° (map 座標), 長軸/短軸 σ = {math.sqrt(w[1]):.2f} / {math.sqrt(max(w[0],0)):.2f} m")
d = np.array([bx - a.cx, by - a.cy]); n = np.linalg.norm(d)
if n > 0.05: print(f"operator→最良 のずれ方向 {math.degrees(math.atan2(d[1], d[0])):.0f}°, 主軸との |cos| = {abs(d @ major) / n:.2f}")
