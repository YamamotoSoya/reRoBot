#!/usr/bin/env python3
# claude: 2026-10-01 新設。
# 変換ノードの実装検証: 再生で記録した /scan_<name> と、元 bag の /scan (pointcloud_to_laserscan 最近点)
# を同一 header.stamp で突き合わせ、ビームごとの一致率を出す。nearest は ≥ 99% 一致が期待値。
# 使い方: compare_scans.py <orig_bag> <replay_bag> [scan_topic=/scan_near] [tol_m=0.05]
import sys, numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan

def read(bag, topic):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id="mcap"), rosbag2_py.ConverterOptions("", ""))
    r.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    out = {}
    while r.has_next():
        t, d, ts = r.read_next()
        m = deserialize_message(d, LaserScan)
        out[(m.header.stamp.sec, m.header.stamp.nanosec)] = m
    return out

orig, rep = sys.argv[1], sys.argv[2]
topic = sys.argv[3] if len(sys.argv) > 3 else "/scan_near"
tol = float(sys.argv[4]) if len(sys.argv) > 4 else 0.05
A = read(orig, "/scan"); B = read(rep, topic)
common = sorted(set(A) & set(B))
print(f"/scan {len(A)} msgs, {topic} {len(B)} msgs, same-stamp pairs {len(common)}")
if not common: sys.exit(1)
agree = both_inf = n = 0; diffs = []
for k in common:
    a = np.array(A[k].ranges, dtype=float); b = np.array(B[k].ranges, dtype=float)
    if len(a) != len(b) or abs(A[k].angle_increment - B[k].angle_increment) > 1e-9:
        print(f"shape/angle mismatch: {len(a)} vs {len(b)}, inc {A[k].angle_increment} vs {B[k].angle_increment}"); sys.exit(1)
    ia, ib = ~np.isfinite(a), ~np.isfinite(b)
    both_inf += int((ia & ib).sum())
    fin = ~ia & ~ib
    d = np.abs(a[fin] - b[fin]); diffs.append(d)
    agree += int((ia & ib).sum() + (d <= tol).sum()); n += len(a)
d = np.concatenate(diffs) if diffs else np.array([])
print(f"beam agreement (both inf or |Δ|≤{tol} m): {100*agree/n:.2f}%   both-inf share {100*both_inf/n:.1f}%")
if len(d): print(f"finite-pair |Δ|: median {np.median(d):.4f} m, 95% {np.percentile(d,95):.3f} m, max {d.max():.2f} m")
only_a = only_b = 0
for k in common:
    a = np.isfinite(A[k].ranges); b = np.isfinite(B[k].ranges); only_a += int((a & ~b).sum()); only_b += int((~a & b).sum())
print(f"finite only in /scan: {only_a}, only in {topic}: {only_b}  (of {n} beams)")
