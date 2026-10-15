#!/usr/bin/env python3
# claude: 2026-10-15 新設 — bag の時間窓 [t_from, t_to] (bag 開始からの秒) で /imu/data の姿勢 yaw・gyro_z 積分、
#   /odom (車輪) と /odometry/filtered (EKF) の yaw 変化と走行距離を 1 s 刻みで並べる。
#   EKF の yaw が跳んだとき「IMU 姿勢 (磁気補正) か・ジャイロか・車輪か」を切り分ける用。
#   使い方 (main コンテナ内): python3 yaw_probe.py <bag_dir> <t_from> <t_to>
import sys, math, numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py
bag, ta, tb = sys.argv[1], float(sys.argv[2]), float(sys.argv[3])
r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=bag, storage_id="mcap"), rosbag2_py.ConverterOptions("cdr","cdr"))
types = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()}
meta = rosbag2_py.Info().read_metadata(bag, "mcap"); t0 = meta.starting_time.nanoseconds * 1e-9
r.set_filter(rosbag2_py.StorageFilter(topics=[t for t in ("/imu/data","/odom","/odometry/filtered") if t in types]))
r.seek(int((t0 + ta) * 1e9))
yaw = lambda q: math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
S = {k: [] for k in ("/imu/data","/odom","/odometry/filtered")}
while r.has_next():
    topic, raw, ts = r.read_next()
    if ts * 1e-9 - t0 > tb: break
    m = deserialize_message(raw, types[topic])
    st = m.header.stamp.sec + m.header.stamp.nanosec*1e-9 - t0
    q = m.orientation if topic == "/imu/data" else m.pose.pose.orientation
    row = [st, yaw(q)]
    if topic != "/imu/data": row += [m.pose.pose.position.x, m.pose.pose.position.y]
    else: row += [m.angular_velocity.z]
    S[topic].append(row)
for k, v in S.items():
    if not v: continue
    v = np.array(v); yu = np.degrees(np.unwrap(v[:,1]))
    print(f"{k}: n={len(v)} t {v[0,0]:.1f}..{v[-1,0]:.1f}  yaw change {yu[-1]-yu[0]:+.1f} deg", end="")
    if k != "/imu/data": print(f"  dist {np.hypot(np.diff(v[:,2]),np.diff(v[:,3])).sum():.2f} m")
    else: print(f"  integ gyro_z {np.degrees(np.trapz(v[:,2], v[:,0])):+.1f} deg")
    # 1 s グリッドで yaw 列挙
    grid = np.arange(math.ceil(v[0,0]), v[-1,0], 1.0)
    print("   ", " ".join(f"{t:.0f}:{np.interp(t, v[:,0], yu)-yu[0]:+.1f}" for t in grid))
