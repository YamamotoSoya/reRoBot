#!/usr/bin/env python3
# claude: 2026-10-01 新設。
# bag の /initialpose (再生時に /initialpose_bag へ remap) を受けて /initialpose に REPEAT 回送り直す。
# bag は 1 回しか publish しないため、発見が間に合わなかった AMCL 変種が初期姿勢を取りこぼす (smoke で 6 中 3)。
# 同じ姿勢の再送は AMCL を同じ姿勢に再初期化するだけなので副作用はない。
import rclpy, threading, time
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped

REPEAT, GAP = 4, 0.5

class Relay(Node):
    def __init__(self):
        super().__init__("initialpose_relay")
        self.pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        self.create_subscription(PoseWithCovarianceStamped, "/initialpose_bag", self.cb, 10)

    def cb(self, m):
        p = m.pose.pose.position
        self.get_logger().info(f"relay initialpose ({p.x:.2f}, {p.y:.2f}) x{REPEAT}")
        def burst():
            for _ in range(REPEAT):
                self.pub.publish(m); time.sleep(GAP)
        threading.Thread(target=burst, daemon=True).start()

rclpy.init(); rclpy.spin(Relay())
