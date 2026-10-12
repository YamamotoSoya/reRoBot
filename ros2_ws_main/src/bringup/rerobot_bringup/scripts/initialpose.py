#!/usr/bin/env python3
# claude: 2026-10-12 新設 — RViz の「2D Pose Estimate」の代わりに /initialpose を端末から送る (AMCL / emcl2 共通)。
#   使い方 (rerobot_env 内、nav_*.launch.py 起動後):
#     ros2 run rerobot_bringup initialpose.py <x> <y> <yaw_deg> [--frame map] [--cov-xy 0.25] [--cov-yaw 0.068]
#   例: コース起点 (waypoints/course.yaml の waypoint0) なら  ros2 run rerobot_bringup initialpose.py 3.60 -0.07 -8
#   ⚠️ AMCL は最初のスキャン後に尤度場を作る間 (0.05 m 地図で ≈13 s) /initialpose を受けると bond が切れて再起動する
#      (tools/loc_eval/README.md 10-11)。nav 起動から 20 s 以上待ってから送る。
#   covariance 既定は RViz と同じ (xy 0.25 m², yaw (15°)² = 0.068 rad²)。
import argparse
import math
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node


def main():
    ap = argparse.ArgumentParser(description="/initialpose を 1 回送る")
    ap.add_argument("x", type=float)
    ap.add_argument("y", type=float)
    ap.add_argument("yaw_deg", type=float, help="map 座標の向き [deg]。+x 向きが 0、反時計回りが正")
    ap.add_argument("--frame", default="map")
    ap.add_argument("--cov-xy", type=float, default=0.25, help="x, y の分散 [m²]")
    ap.add_argument("--cov-yaw", type=float, default=0.068, help="yaw の分散 [rad²]")
    ap.add_argument("--repeat", type=int, default=3, help="取りこぼし対策の送信回数 (0.5 s 間隔)")
    a = ap.parse_args()

    rclpy.init()
    node = Node("initialpose_cli")
    pub = node.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = a.frame
    msg.pose.pose.position.x = a.x
    msg.pose.pose.position.y = a.y
    yaw = math.radians(a.yaw_deg)
    msg.pose.pose.orientation.z = math.sin(yaw / 2)
    msg.pose.pose.orientation.w = math.cos(yaw / 2)
    cov = [0.0] * 36
    cov[0] = cov[7] = a.cov_xy
    cov[35] = a.cov_yaw
    msg.pose.covariance = cov
    # claude: 購読側 (amcl / emcl2) との接続確立を待ってから送る。接続前に publish すると消える。
    t0 = time.time()
    while pub.get_subscription_count() == 0 and time.time() - t0 < 5.0:
        rclpy.spin_once(node, timeout_sec=0.1)
    if pub.get_subscription_count() == 0:
        node.get_logger().warn("/initialpose の購読者が見つからない (nav が起動していない?)。送るだけ送る")
    for i in range(max(1, a.repeat)):
        msg.header.stamp = node.get_clock().now().to_msg()
        pub.publish(msg)
        node.get_logger().info(f"sent initialpose x={a.x:.2f} y={a.y:.2f} yaw={a.yaw_deg:.1f}° ({i + 1}/{a.repeat})")
        rclpy.spin_once(node, timeout_sec=0.5)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
