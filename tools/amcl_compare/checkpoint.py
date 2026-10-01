#!/usr/bin/env python3
# claude: 2026-10-01 新設。
# 反復再生 (複数の replay bag) を、ロボット静止中に operator が与えた /initialpose (= 参照点) の直前姿勢で評価する。
# 姿勢列が完全一致するコピー (同じ乱数種) は 1 run にまとめ、変種ごとの「大誤差 run 数 / 独立 run 数」と
# base との Fisher 正確検定 (片側) を出す。
# 使い方: checkpoint.py <orig_bag> <checkpoint_t_s> <big_err_m> <replay_bag> [<replay_bag> ...]
import sys, math, collections, numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import PoseWithCovarianceStamped
from scipy.stats import fisher_exact

def rd(uri, pred):
    r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=uri, storage_id="mcap"), rosbag2_py.ConverterOptions("", ""))
    av = [t.name for t in r.get_all_topics_and_types() if pred(t.name)]; r.set_filter(rosbag2_py.StorageFilter(topics=av))
    out = collections.defaultdict(list)
    while r.has_next():
        t, d, ts = r.read_next(); m = deserialize_message(d, PoseWithCovarianceStamped); p = m.pose.pose; q = p.orientation
        out[t].append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, ts / 1e9, p.position.x, p.position.y, math.atan2(2 * q.w * q.z, 1 - 2 * q.z * q.z)))
    return {k: np.array(v) for k, v in out.items()}

orig, tcp, big = sys.argv[1], float(sys.argv[2]), float(sys.argv[3]); reps = sys.argv[4:]
r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=orig, storage_id="mcap"), rosbag2_py.ConverterOptions("", "")); t0 = r.read_next()[2] / 1e9
ip = rd(orig, lambda n: n == "/initialpose")["/initialpose"]
cp = ip[np.argmin(abs(ip[:, 1] - t0 - tcp))]; tc = cp[1] - t0
print(f"参照点: operator /initialpose @ {tc:.1f}s = ({cp[2]:.2f}, {cp[3]:.2f}) yaw {math.degrees(cp[4]):.1f}°")
res = collections.defaultdict(list)
for rep in reps:
    P = rd(rep, lambda n: n.startswith("/loc_") and n.endswith("/amcl_pose")); seen = set()
    for topic, a in sorted(P.items()):
        g = topic.split("/")[1][4:].split("_r")[0]
        key = (g, a[:, 2:5].round(6).tobytes())
        if key in seen: continue
        seen.add(key)
        k = np.where(a[:, 0] - t0 < tc - 0.3)[0]
        if not len(k): continue
        p = a[k[-1]]
        res[g].append((math.hypot(p[2] - cp[2], p[3] - cp[3]), abs(math.degrees((p[4] - cp[4] + math.pi) % (2 * math.pi) - math.pi)), p[2] - cp[2], p[3] - cp[3]))
bl = res.get("base"); nb_big = sum(e > big for e, *_ in bl) if bl else 0
print(f"| 変種 | 独立 run | 大誤差 (>{big:g} m) run | 割合 | Fisher 片側 p (vs base) | 平常 run の誤差 中央 [m] | 平常 run の dx, dy 中央 [m] | yaw 誤差 中央 [deg] |")
print("|---|---|---|---|---|---|---|---|")
for g, v in sorted(res.items()):
    e = np.array([x[0] for x in v]); nbig = int((e > big).sum()); ok = e <= big
    p = fisher_exact([[nb_big, len(bl) - nb_big], [nbig, len(v) - nbig]], alternative="greater")[1] if bl and g != "base" else float("nan")
    dx = np.median([x[2] for x, o in zip(v, ok) if o]) if ok.any() else float("nan"); dy = np.median([x[3] for x, o in zip(v, ok) if o]) if ok.any() else float("nan")
    print(f"| {g} | {len(v)} | {nbig} | {100 * nbig / len(v):.0f}% | {p:.3f} | {np.median(e[ok]) if ok.any() else float('nan'):.2f} | {dx:+.2f}, {dy:+.2f} | {np.median([x[1] for x in v]):.1f} |")
