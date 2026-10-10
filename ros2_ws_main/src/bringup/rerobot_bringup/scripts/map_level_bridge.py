#!/usr/bin/env python3
# claude: 3D 自己位置推定の「3D の世界 → 2D の世界」橋渡し (2026-10-10 追加)。
#   設計と図: docs/features/2026-10-10_lidar_localization_3d.md §2.1〜2.3。
#
#   TF の木:  map ──(このノード)──▶ map_3d ──(lidar_localization)──▶ odom ──(EKF)──▶ base_link
#
#   lidar_localization は 3D 地図の座標系 map_3d で姿勢を解く。level_map_to_odom: true で
#   map_3d->odom の roll / pitch は 0 になるが、z (= 3D 地図上のロボットの高さ) は残る。
#   Nav2 の global costmap は map 座標の z で障害物の高さを判定する (min/max_obstacle_height) ので、
#   z が残ると坂の上下や GLIM の z ドリフト区間で障害物が捨てられる。そこで
#     ① TF map->map_3d = 並進 (0, 0, −z) を出して z を打ち消す → map から見た odom / base_link は z ≈ 0
#     ② RViz の /initialpose (map 座標、z = 0) を map_3d 座標に直して /initialpose_3d へ中継する
#        (lidar_localization は frame_id が global_frame_id と違う初期位置を捨てる)。
#        z は GLIM 軌跡 traj_lidar.txt の最寄り点 (= 地図を作ったときにその場所を通った高さ) から決める。
#   x, y, yaw は map と map_3d で同じ (2D 地図と 3D 地図を同じ GLIM dump から作る前提)。
#
#   ① は /tf_static で出す (z は坂に沿ってゆっくりしか変わらない)。z が z_update_threshold 以上
#   変わったときだけ出し直す。⚠ rclpy の StaticTransformBroadcaster は同じ child_frame_id の 2 回目以降を
#   捨てて最初の値を再送する (Jazzy tf2_ros_py 0.36.22 で確認) ので使わず、/tf_static へ自前で publish する
#   (受信側 tf2 の static は上書きとして扱われる)。
#
#   起動: loc_lidar3d.launch.py に同梱 (単体なら ros2 run rerobot_bringup map_level_bridge.py)。
import math
import os

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from rclpy.time import Time
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformException, TransformListener


def load_traj(path):
    """GLIM の traj_lidar.txt (TUM 形式: stamp x y z qx qy qz qw、LiDAR の姿勢、map_3d 座標) から (x, y, z) を読む。"""
    pts = []
    with open(path) as f:
        for line in f:
            v = line.split()
            if len(v) >= 4 and not line.startswith("#"):
                pts.append((float(v[1]), float(v[2]), float(v[3])))
    return pts


class MapLevelBridge(Node):
    def __init__(self):
        super().__init__("map_level_bridge")
        self.map_frame = self.declare_parameter("map_frame", "map").value  # Nav2 / 2D 地図の座標系
        self.map3d_frame = self.declare_parameter("map3d_frame", "map_3d").value  # 3D 地図の座標系
        self.odom_frame = self.declare_parameter("odom_frame", "odom").value
        self.base_frame = self.declare_parameter("base_frame", "base_link").value
        self.lidar_frame = self.declare_parameter("lidar_frame", "rfans").value  # traj_lidar.txt が表す frame
        self.rate = self.declare_parameter("rate", 20.0).value  # [Hz] map_3d->odom を読みに行く周期
        self.z_thr = self.declare_parameter("z_update_threshold", 0.01).value  # [m] これ以上変わったら出し直す
        # [m] traj が無いときに使う「スタート地点の base_link の高さ (map_3d 座標)」
        self.z = self.declare_parameter("initial_z", 0.0).value
        traj_path = self.declare_parameter("traj_path", "").value  # 空 or 無ければ initial_z だけで動く
        self.traj_max_dist = self.declare_parameter("traj_max_dist", 5.0).value  # [m] これより遠い最寄り点は使わない

        self.traj = []
        if traj_path and os.path.isfile(traj_path):
            self.traj = load_traj(traj_path)
            self.get_logger().info(f"GLIM 軌跡 {len(self.traj)} 点を読込: {traj_path}")
        elif traj_path:
            self.get_logger().warn(f"traj_path が見つからない: {traj_path} — 初期位置の z は initial_z を使う")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # /tf_static と同じ QoS (transient_local) — 後から起動したノードや RViz も最新の 1 本を受け取れる
        self.static_pub = self.create_publisher(
            TFMessage, "/tf_static",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST))

        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, "initialpose_3d", 10)
        self.create_subscription(PoseWithCovarianceStamped, "initialpose", self.on_initialpose, 10)
        self.create_timer(1.0 / self.rate, self.on_timer)

        self.lidar_height = None  # base_link→LiDAR の高さ (URDF 由来、TF から取得)
        self.have_estimate = False  # map_3d->odom を一度でも読めたか
        self.reset_time = None  # 最後に初期位置を中継した時刻。これより古い map_3d->odom は旧トラックなので使わない
        self.publish_level(self.z)  # 推定が出る前から TF の木を繋いでおく (map->map_3d が無いと map が孤立する)
        self.get_logger().info(
            f"ready: TF {self.map_frame}->{self.map3d_frame} で z を打ち消す (初期 z={self.z:+.2f} m)、"
            f"/initialpose → /initialpose_3d を中継")

    def publish_level(self, z):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.map_frame
        t.child_frame_id = self.map3d_frame
        t.transform.translation.z = -z  # map から見て map_3d は z だけ下 → map_3d 上の高さ z が map では 0 になる
        t.transform.rotation.w = 1.0
        self.static_pub.publish(TFMessage(transforms=[t]))

    def ground_z_from_traj(self, x, y):
        """(x, y) に最も近い軌跡点の LiDAR 高さ − LiDAR 取付高さ = その場所の base_link の高さ。使えなければ None。"""
        if not self.traj or self.lidar_height is None:
            return None, None
        px, py, pz = min(self.traj, key=lambda p: (p[0] - x) ** 2 + (p[1] - y) ** 2)
        d = math.hypot(px - x, py - y)
        if d > self.traj_max_dist:
            return None, d
        return pz - self.lidar_height, d

    def on_timer(self):
        if self.lidar_height is None and self.traj:
            try:
                tf = self.tf_buffer.lookup_transform(self.base_frame, self.lidar_frame, Time())
                self.lidar_height = tf.transform.translation.z
                # まだ推定も初期位置も無ければ、軌跡の始点 (= 地図を撮り始めた場所) の高さを仮の z にする
                if not self.have_estimate and self.reset_time is None:
                    x0, y0, _ = self.traj[0]
                    z0, _ = self.ground_z_from_traj(x0, y0)
                    self.z = z0
                    self.publish_level(z0)
                    self.get_logger().info(f"仮の z = 軌跡始点の高さ {z0:+.2f} m (LiDAR 取付高 {self.lidar_height:.3f} m)")
            except TransformException:
                pass  # robot_state_publisher がまだ。次の周期で再試行

        try:
            tf = self.tf_buffer.lookup_transform(self.map3d_frame, self.odom_frame, Time())
        except TransformException:
            return  # lidar_localization がまだ推定を出していない (初期位置待ち等)。前の値を保持
        if self.reset_time is not None and Time.from_msg(tf.header.stamp) < self.reset_time:
            return  # 初期位置を置き直す前の推定。新しい推定が出るまで初期位置の z を保つ
        z = tf.transform.translation.z
        if not self.have_estimate:
            self.have_estimate = True
            self.get_logger().info(f"{self.map3d_frame}->{self.odom_frame} を受信 (z={z:+.2f} m)")
        if abs(z - self.z) >= self.z_thr:
            self.z = z
            self.publish_level(z)

    def on_initialpose(self, msg):
        frame = msg.header.frame_id
        if frame == self.map3d_frame:
            self.pose_pub.publish(msg)  # すでに map_3d 座標ならそのまま
            return
        if frame not in (self.map_frame, ""):
            self.get_logger().warn(
                f"/initialpose の frame_id '{frame}' は {self.map_frame} / {self.map3d_frame} 以外 — 無視 "
                f"(RViz の Fixed Frame を {self.map_frame} にする)")
            return
        # map 座標 → map_3d 座標: x, y, 姿勢は共通。z は「その場所の 3D 地図上の高さ」を入れる
        p = msg.pose.pose.position
        z, d = self.ground_z_from_traj(p.x, p.y)
        if z is not None:
            src = f"軌跡の最寄り点 ({d:.1f} m 先)"
        else:
            z = self.z + p.z  # 軌跡が無い / 遠い: いまの打ち消し量 (= 直近の推定 or initial_z) を使う
            src = "直近の推定" if self.have_estimate else "initial_z"
            if d is not None:
                src += f" (軌跡の最寄り点が {d:.1f} m 先で遠い)"
        msg.header.frame_id = self.map3d_frame
        p.z = z
        self.reset_time = self.get_clock().now()
        self.z = z
        self.publish_level(z)  # 推定が出る前から map 上の z ≈ 0 にしておく
        self.pose_pub.publish(msg)
        self.get_logger().info(f"/initialpose → /initialpose_3d: ({p.x:.2f}, {p.y:.2f}, z={z:+.2f} m ← {src})")


def main():
    rclpy.init()
    node = MapLevelBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
