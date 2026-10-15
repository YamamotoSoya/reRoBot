#!/usr/bin/env python3
# claude: map→odom TF の見張り番 (2026-10-16 追加)。
#   AMCL は scan が来たときだけ map→odom を出す (タイマー再送なし)。R-Fans の回転ディップで scan が 5〜7 s
#   止まると map→odom が transform_tolerance 秒で失効し、controller が TF を引けず区間失敗 (error_code 102)
#   になる (10-14 bag 757〜768 s)。transform_tolerance を伸ばして耐えると、Nav2 (Time(0) で TF を引く) には
#   その秒数だけ古い補正が返る副作用がある (10-15 bag 1802、8.0 s で 2.97 m 跳びが 7.8 s 遅れて届き abort)。
#   → tolerance は短く (2.0) しておき、本ノードが「AMCL が黙っている間だけ」最後の map→odom を今の日付で
#     出し続ける。その間ロボットは odom (EKF) の積み上げで走り、scan 復帰後は AMCL の値に戻る。
#
#   動作:
#     /tf を購読し、parent_frame→child_frame (既定 map→odom) の最新値と受信時刻を覚える。
#     自分が出したものは stamp で見分けて無視する (同じ topic に流れるため)。
#     最後の受信から silence_sec (既定 0.5 s) 以上経ったら、rate_hz (既定 20 Hz) で最後の値に
#     stamp = now + stamp_ahead (既定 0.5 s) を付けて出す。AMCL の次の 1 発で即やめる。
#     stamp_ahead は AMCL の transform_tolerance より短くてよい: tf2 バッファは時刻順に挿入するので、
#     AMCL の最後の stamp より前に入っても「今」を挟む 2 点が同じ値なら補間結果は同じ。
#   ⚠ AMCL と同じ親子ペアを 2 か所から出すのは「AMCL 無音のとき」だけ。常時出すと値が違うとき補間で混ざる。
#
#   使い方 (loc_amcl.launch.py が起動する。単体なら):
#     ros2 run rerobot_bringup map_odom_holder.py --ros-args -p silence_sec:=0.5
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.time import Time
from rclpy.duration import Duration
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped


class MapOdomHolder(Node):
    def __init__(self):
        super().__init__("map_odom_holder")
        self.declare_parameter("parent_frame", "map")
        self.declare_parameter("child_frame", "odom")
        self.declare_parameter("silence_sec", 0.5)   # これ以上 AMCL が黙ったら保持を始める
        self.declare_parameter("stamp_ahead", 0.5)   # 出す TF の未来側日付 [s]
        self.declare_parameter("rate_hz", 20.0)
        self.parent = self.get_parameter("parent_frame").value
        self.child = self.get_parameter("child_frame").value
        self.silence = Duration(seconds=float(self.get_parameter("silence_sec").value))
        self.ahead = Duration(seconds=float(self.get_parameter("stamp_ahead").value))
        rate = float(self.get_parameter("rate_hz").value)

        self.last: TransformStamped | None = None  # AMCL から最後に受けた map→odom
        self.last_rx: Time | None = None           # その受信時刻 (ROS clock)
        self.own_stamps = set()                    # 自分が出した stamp (nanosec) — 自己受信の除外用
        self.holding = False
        self.hold_count = 0

        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE,
                         history=HistoryPolicy.KEEP_LAST, depth=100)
        self.sub = self.create_subscription(TFMessage, "/tf", self.on_tf, qos)
        self.pub = self.create_publisher(TFMessage, "/tf", qos)
        self.timer = self.create_timer(1.0 / rate, self.tick)
        self.get_logger().info(f"見張り番開始: {self.parent}→{self.child}, 無音 {self.silence.nanoseconds / 1e9:.2f} s で保持, "
                               f"stamp +{self.ahead.nanoseconds / 1e9:.2f} s, {rate:.0f} Hz")

    def on_tf(self, msg: TFMessage):
        for t in msg.transforms:
            if t.header.frame_id != self.parent or t.child_frame_id != self.child:
                continue
            key = Time.from_msg(t.header.stamp).nanoseconds
            if key in self.own_stamps:
                self.own_stamps.discard(key)
                continue
            self.last = t
            self.last_rx = self.get_clock().now()
            if self.holding:
                self.holding = False
                self.get_logger().warn(f"{self.parent}→{self.child} 復帰 (保持 {self.hold_count} 発)")

    def tick(self):
        if self.last is None or self.last_rx is None:
            return
        now = self.get_clock().now()
        if now - self.last_rx < self.silence:
            return
        if not self.holding:
            self.holding = True
            self.hold_count = 0
            self.get_logger().warn(f"{self.parent}→{self.child} が {self.silence.nanoseconds / 1e9:.2f} s 途絶 → 最後の値を保持して配信")
        out = TransformStamped()
        out.header.frame_id = self.parent
        out.header.stamp = (now + self.ahead).to_msg()
        out.child_frame_id = self.child
        out.transform = self.last.transform
        self.own_stamps.add(Time.from_msg(out.header.stamp).nanoseconds)
        if len(self.own_stamps) > 200:  # 自己受信が届かなかった分の掃除
            self.own_stamps = set(sorted(self.own_stamps)[-100:])
        self.pub.publish(TFMessage(transforms=[out]))
        self.hold_count += 1
        if self.hold_count % 100 == 0:
            self.get_logger().warn(f"保持中 {self.hold_count / 20:.0f} s — AMCL の入力 scan を確認", throttle_duration_sec=5.0)


def main():
    rclpy.init()
    node = MapOdomHolder()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):  # launch からの SIGINT/SIGTERM で静かに終わる
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
