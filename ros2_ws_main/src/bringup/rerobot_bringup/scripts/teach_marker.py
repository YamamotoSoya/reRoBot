#!/usr/bin/env python3
# claude: ティーチング用マーカー (2026-10-09 追加)。
#   地図作成用の bag を撮りながら、停止線など「止まりたい場所」に印を付ける。
#   joy のボタン (既定 Y) を押した瞬間に /teach_marker (visualization_msgs/Marker) を 1 本出す。
#
#   bag に残すのは「押した時刻 (header.stamp) + ラベル (text)」だけ。座標は地図を作った後に
#   GLIM dump の軌跡 (traj_lidar.txt) からその時刻の姿勢を引いて決める (抽出ツールは別途)。
#   → 地図と停止点が同じ最適化結果から出るので、地図を作り直しても抽出し直すだけで点が追従する。
#
#   手順: 停止位置に車体を合わせて止める → ボタン → そのまま 2 s 以上静止 → 走行再開。
#     押した時点で直前 still_time 秒静止していなければ WARN し、text に " (moving)" を付ける
#     (記録はする。抽出側で前後の静止区間を探す)。/odom が来ていなければ " (no-odom)"。
#   Marker の pose は /odom (frame odom) の値で、RViz で押した場所を見るための表示用。座標の正には使わない。
#
#   起動: joy_teleop.launch.py に同梱 (単体なら ros2 run rerobot_bringup teach_marker.py)。
#   記録: ros2 bag record の topic に /teach_marker を足す (docs/manual 第5章)。
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Joy
from visualization_msgs.msg import Marker


class TeachMarker(Node):
    def __init__(self):
        super().__init__("teach_marker")
        self.button = self.declare_parameter("button", 3).value  # Xbox Y (A=0 B=1 X=2 Y=3 LB=4 RB=5)
        self.prefix = self.declare_parameter("label_prefix", "teach").value
        self.still_time = self.declare_parameter("still_time", 1.0).value  # [s] 押す前に必要な静止時間
        self.still_lin = self.declare_parameter("still_lin", 0.02).value  # [m/s] これ未満を静止とみなす
        self.still_ang = self.declare_parameter("still_ang", 0.02).value  # [rad/s]

        self.pub = self.create_publisher(Marker, "teach_marker", 10)
        self.create_subscription(Joy, "joy", self.on_joy, 10)
        self.create_subscription(Odometry, "odom", self.on_odom, 10)

        self.prev_pressed = False  # joy は autorepeat で同じ状態を再送するので立ち上がりだけ拾う
        self.count = 0
        self.odom = None  # 最新の Odometry
        self.odom_time = None  # その受信時刻
        self.last_moving = None  # 最後に「動いている」と判定した時刻
        self.get_logger().info(f"ready: button {self.button} で /teach_marker を出す")

    def on_odom(self, msg):
        now = self.get_clock().now()
        self.odom, self.odom_time = msg, now
        t = msg.twist.twist
        if abs(t.linear.x) >= self.still_lin or abs(t.angular.z) >= self.still_ang:
            self.last_moving = now

    def on_joy(self, msg):
        pressed = len(msg.buttons) > self.button and msg.buttons[self.button] == 1
        if pressed and not self.prev_pressed:
            self.mark()
        self.prev_pressed = pressed

    def mark(self):
        now = self.get_clock().now()
        label = f"{self.prefix}_{self.count:02d}"
        flags = ""
        if self.odom_time is None or (now - self.odom_time).nanoseconds > 0.5e9:
            flags = " (no-odom)"
            self.get_logger().warn(f"{label}: /odom が来ていない — 静止を確認できない")
        elif self.last_moving is not None and (now - self.last_moving).nanoseconds < self.still_time * 1e9:
            flags = " (moving)"
            self.get_logger().warn(f"{label}: 動いている / 止まって {self.still_time:.1f} s 未満 — 止まってから押し直す")

        m = Marker()
        m.header.stamp = now.to_msg()
        m.header.frame_id = self.odom.header.frame_id if self.odom is not None else "odom"
        m.ns, m.id = "teach", self.count
        m.type, m.action = Marker.TEXT_VIEW_FACING, Marker.ADD
        if self.odom is not None:  # 参照代入すると受信メッセージを書き換えるので値だけ写す
            p = self.odom.pose.pose.position
            m.pose.position.x, m.pose.position.y, m.pose.position.z = p.x, p.y, p.z
        m.pose.position.z += 1.0  # 車体の上に浮かせて表示
        m.pose.orientation.w = 1.0
        m.scale.z = 0.5
        m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.8, 0.0, 1.0
        m.text = label + flags
        self.pub.publish(m)
        self.get_logger().info(f"marked {m.text}  stamp={m.header.stamp.sec}.{m.header.stamp.nanosec:09d}")
        self.count += 1


def main():
    rclpy.init()
    node = TeachMarker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
