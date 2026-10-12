#!/usr/bin/env python3
# claude: 2026-10-12 新設 — /rfans/scan_all (pointcloud_to_laserscan は best_effort で出す) を reliable で /rfans/scan_all_rel に中継する。
#   emcl2 (reliable 購読) に届かせるため。AMCL (best_effort 購読) も reliable publisher から受け取れる。
#   使い方 (コンテナ内、再生時): python3 tools/loc_eval/scan_relay.py --ros-args -p use_sim_time:=true  (README「2026-10-12 追試」参照)
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
class Relay(Node):
    def __init__(self):
        super().__init__("scan_relay")
        be = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        rel = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
        self.pub = self.create_publisher(LaserScan, "/rfans/scan_all_rel", rel)
        self.sub = self.create_subscription(LaserScan, "/rfans/scan_all", self.pub.publish, be)
rclpy.init(); rclpy.spin(Relay())
