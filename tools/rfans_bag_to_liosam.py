#!/usr/bin/env python3
# claude: rosbag2 (mcap) の R-Fans 点群を LIO-SAM が食える形に変換するワンショットツール (2026-09-04)。
#
# 背景 (docs/claude/PROJECT_STATE.md 2026-09-04 参照):
#   surestar_rfans_ros2 の PointCloud2 は LIO-SAM の velodyne 形式と 3 点で不整合:
#     1) is_dense=false → imageProjection が起動直後に rclcpp::shutdown() するハードチェック
#     2) チャンネル番号のフィールド名が "laserid" (int32) — LIO-SAM は "ring" (uint16) を要求
#        (velodyne モードの ring 自前計算は VLP-16 の等間隔 2° 前提で R-Fans-16 には不正確)
#     3) トピック名 /rfans_driver/rfans_points — LIO-SAM 既定は /points
#   per-point "time" (float32, スキャン先頭からの相対秒) は velodyne 形式そのままなので流用できる。
#
# 処理: /rfans_driver/rfans_points → x,y,z,intensity,ring,time の 22 byte/点に詰め替え、
#       無効点 (xyz 全 0 / 非有限) を除去して is_dense=true、/points として書き出す。
#       /imu/data はシリアライズ済みバイト列をそのまま素通し。他トピック (/tf 等) は
#       LIO-SAM 自身の TF と衝突するため意図的に落とす。
#
# 実行 (rerobot_env に stdin で流す — tools/ は main コンテナに mount されていないため):
#   docker exec -i rerobot_env bash -c \
#     'source /opt/ros/jazzy/setup.bash && python3 - <in_dir> <out_dir>' \
#     < tools/rfans_bag_to_liosam.py
#   in_dir / out_dir はコンテナ内パス (例: /workspace/bags/.../2026-08-14_0919)

import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import PointCloud2, PointField

CLOUD_IN = "/rfans_driver/rfans_points"
CLOUD_OUT = "/points"
IMU = "/imu/data"
MIN_VALID_POINTS = 100  # ゴミフレームガード (surestar_rfans_ros2 2026-09-02 と同基準)

OUT_DTYPE = np.dtype(
    [
        ("x", np.float32),
        ("y", np.float32),
        ("z", np.float32),
        ("intensity", np.float32),
        ("ring", np.uint16),
        ("time", np.float32),
    ]
)  # packed 22 byte/点

OUT_FIELDS = [
    PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
    PointField(name="ring", offset=16, datatype=PointField.UINT16, count=1),
    PointField(name="time", offset=18, datatype=PointField.FLOAT32, count=1),
]


def convert_cloud(msg: PointCloud2) -> PointCloud2 | None:
    off = {f.name: f.offset for f in msg.fields}
    n = msg.width * msg.height
    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(n, msg.point_step)

    def f32(name):
        o = off[name]
        return raw[:, o : o + 4].copy().view(np.float32).ravel()

    x, y, z, inten, tim = f32("x"), f32("y"), f32("z"), f32("intensity"), f32("time")
    lid = raw[:, off["laserid"] : off["laserid"] + 4].copy().view(np.int32).ravel()

    xyz = np.stack([x, y, z], axis=1)
    keep = np.isfinite(xyz).all(axis=1) & (np.abs(xyz).sum(axis=1) > 0.0)
    k = int(keep.sum())
    if k < MIN_VALID_POINTS:
        return None  # 測距異常のゴミフレームは丸ごと捨てる

    out = np.empty(k, dtype=OUT_DTYPE)
    out["x"], out["y"], out["z"] = x[keep], y[keep], z[keep]
    out["intensity"] = inten[keep]
    out["ring"] = lid[keep].astype(np.uint16)
    out["time"] = tim[keep]

    dst = PointCloud2()
    dst.header = msg.header
    dst.height = 1
    dst.width = k
    dst.fields = OUT_FIELDS
    dst.is_bigendian = False
    dst.point_step = OUT_DTYPE.itemsize
    dst.row_step = dst.point_step * k
    dst.data = out.tobytes()
    dst.is_dense = True  # 無効点は上で除去済み
    return dst


def make_topic(name: str, type_name: str) -> rosbag2_py.TopicMetadata:
    # Jazzy の TopicMetadata は第 1 引数に id を要求する (Humble 以前は無い)
    try:
        return rosbag2_py.TopicMetadata(
            0, name=name, type=type_name, serialization_format="cdr"
        )
    except TypeError:
        return rosbag2_py.TopicMetadata(
            name=name, type=type_name, serialization_format="cdr"
        )


def main(in_dir: str, out_dir: str) -> None:
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=in_dir, storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=out_dir, storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    writer.create_topic(make_topic(CLOUD_OUT, "sensor_msgs/msg/PointCloud2"))
    writer.create_topic(make_topic(IMU, "sensor_msgs/msg/Imu"))

    n_cloud = n_imu = n_dropped = 0
    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic == IMU:
            writer.write(IMU, data, t)
            n_imu += 1
        elif topic == CLOUD_IN:
            dst = convert_cloud(deserialize_message(data, PointCloud2))
            if dst is None:
                n_dropped += 1
                continue
            writer.write(CLOUD_OUT, serialize_message(dst), t)
            n_cloud += 1
    print(
        f"done: cloud {n_cloud} msgs (garbage dropped {n_dropped}), imu {n_imu} msgs\n"
        f"  -> {out_dir}"
    )


if __name__ == "__main__":
    if len(sys.argv) != 3:
        sys.exit(f"usage: {sys.argv[0]} <in_bag_dir> <out_bag_dir>")
    main(sys.argv[1], sys.argv[2])
