#!/usr/bin/env python3
# claude: 2026-10-28 作成
"""bag の R-Fans 点群に櫛偽点フィルタ (rfans_comb_filter.ghost_mask) を掛けた bag を作る.

背景: docs/issue/2026-10-25_rfans_range_comb_ghost.md §8。
- 点群トピック (既定 /rfans_driver/rfans_points) は同じ名前のまま、偽点の行を抜いて書く
  (driver の点群は高さ 1 の並びなので、行を抜いても形は崩れない)。
- 0.3 m 未満・NaN の点は判定に掛けず、そのまま残す。
- ほかのトピックは元の QoS 情報ごとそのままコピーする (--drop で除外可)。
  ⚠️ /rfans/scan など、元の点群から作られたトピックはフィルタ前のまま。
- 1 フレームごとの点数・除去数を <出力>/comb_filter_stats.csv に残す。

例 (main コンテナ内、ws を source してから):
  python3 rfans_comb_filter_bag.py /workspace/bags/raw/<bag> /workspace/bags/derived/<bag>/combfilter
"""
import argparse
import os
import sys
import time

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import PointCloud2

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from rfans_comb_filter import ghost_mask  # noqa: E402

# PointField.datatype → numpy
_NP = {1: "<i1", 2: "<u1", 3: "<i2", 4: "<u2", 5: "<i4", 6: "<u4", 7: "<f4", 8: "<f8"}


def filter_cloud(m):
    """PointCloud2 から偽点の行を抜いたメッセージと (点数, 除去数) を返す."""
    n = m.width * m.height
    dt = np.dtype({"names": [f.name for f in m.fields], "formats": [_NP[f.datatype] for f in m.fields],
                   "offsets": [f.offset for f in m.fields], "itemsize": m.point_step})
    a = np.frombuffer(bytes(m.data), dtype=dt, count=n)
    xyz = np.stack([a["x"], a["y"], a["z"]], 1).astype(np.float64)
    ok = np.isfinite(xyz).all(1) & (np.linalg.norm(xyz, axis=1) > 0.3)
    drop = np.zeros(n, bool)
    if ok.sum() > 0:
        drop[ok] = ghost_mask(xyz[ok], a["laserid"][ok].astype(np.float64), a["time"][ok].astype(np.float64))
    keep = a[~drop]
    m.height, m.width = 1, len(keep)
    m.row_step = m.point_step * len(keep)
    m.data = keep.tobytes()
    return m, n, int(drop.sum())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("inp")
    ap.add_argument("out")
    ap.add_argument("--topic", default="/rfans_driver/rfans_points")
    ap.add_argument("--drop", nargs="*", default=[], help="コピーしないトピック")
    a = ap.parse_args()

    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=a.inp, storage_id=""), rosbag2_py.ConverterOptions("", ""))
    topics = [t for t in r.get_all_topics_and_types() if t.name not in a.drop]
    r.set_filter(rosbag2_py.StorageFilter(topics=[t.name for t in topics]))
    dur = r.get_metadata().duration.nanoseconds * 1e-9
    t_start = r.get_metadata().starting_time.nanoseconds

    w = rosbag2_py.SequentialWriter()
    w.open(rosbag2_py.StorageOptions(uri=a.out, storage_id="mcap"), rosbag2_py.ConverterOptions("", ""))
    for t in topics:
        w.create_topic(t)

    stats = open(os.path.join(a.out, "comb_filter_stats.csv"), "w")
    stats.write("bag_time_s,points,removed\n")
    tot = rem = nf = 0
    t0 = time.time(); last = t0
    while r.has_next():
        name, d, ts = r.read_next()
        if name == a.topic:
            m, n, k = filter_cloud(deserialize_message(d, PointCloud2))
            d = serialize_message(m)
            tot += n; rem += k; nf += 1
            stats.write(f"{(ts - t_start) * 1e-9:.3f},{n},{k}\n")
            if time.time() - last > 30:
                last = time.time()
                print(f"  {(ts - t_start) * 1e-9:7.0f}/{dur:.0f} s  frames {nf}  removed {100 * rem / max(tot, 1):.2f}%  ({last - t0:.0f} s 経過)", flush=True)
        w.write(name, d, ts)
    stats.close()
    del w  # close
    print(f"done: frames {nf}, points {tot}, removed {rem} ({100 * rem / max(tot, 1):.2f}%), {time.time() - t0:.0f} s")


if __name__ == "__main__":
    main()
