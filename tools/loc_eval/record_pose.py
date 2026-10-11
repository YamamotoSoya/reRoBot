#!/usr/bin/env python3
# claude: 2026-10-11 新設 — bag 再生中に自己位置推定の結果を記録する (手法共通)。
#
#   記録するもの (すべて sim 時刻 = bag 時刻):
#     est.tum         TF map→base_link を 20 Hz で lookup (AMCL / emcl2 / lidar3d で共通の出力)
#     cov.txt         /amcl_pose・/mcl_pose・/pcl_pose の共分散 (xx, yy, yaw-yaw) — 手法の自己申告
#     alpha.txt       emcl2 の /alpha (平均尤度)
#     alignment.jsonl lidar_localization の /alignment_status (level 0=OK と主要値)
#     events.txt      initialpose 送信などのイベント
#   さらに /clock が動き始めて initialpose_delay 秒後から /initialpose (frame map) を送る。
#   「全手法に同じ初期姿勢を同じタイミングで与える」ための仕組みで、bag の再生開始前に起動しておく。
#
#   claude: 2026-10-11 初期姿勢は「bag 開始時の姿勢 (initialpose.yaml) に、それ以降の odom→base_link の移動量を
#   足した現在姿勢」として送り、TF map→base_link がそれと一致する (tol 以内) まで resend_every 秒ごとに送り直す。
#   AMCL は大きな地図の尤度場構築で 10 s 以上固まり、その間に届いた initialpose を robot が動き出した後に
#   odom 補正なしで適用していた (1611 bag で 4 m ずれて開始)。補正つきなら遅れて処理されても正しい位置になる。
#   一致したら以後は送らない (手法の見失いをこちらが直してしまわないため)。
#
# 使い方 (コンテナ内、use_sim_time で動く):
#   python3 record_pose.py <run_dir> --init <initialpose.yaml> [--initialpose_delay 2.0] [--resend_every 1.0]
#                          [--resend_max 60] [--tol_xy 1.0] [--tol_yaw_deg 15]
import argparse
import json
import math
import os
import sys

import rclpy
import yaml
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float32
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException

ALIGNMENT_KEYS = ("fitness_score", "correction_translation_m", "consecutive_rejected_updates",
                  "recovery_state", "failure_category", "bad_match_active", "weak_overlap_active")


def _stamp(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def _yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def _xyyaw(tf):
    return tf.transform.translation.x, tf.transform.translation.y, _yaw(tf.transform.rotation)


class Recorder(Node):
    def __init__(self, run_dir, init, delay, rate, resend_every, resend_max, tol_xy, tol_yaw):
        super().__init__("loc_eval_recorder", parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", value=True)])
        self.init = init
        self.delay = delay
        self.resend_every, self.resend_left = resend_every, resend_max
        self.tol_xy, self.tol_yaw = tol_xy, tol_yaw
        self.odom0 = None       # odom→base_link at initialpose.yaml の stamp (bag 開始時の姿勢の基準)
        self.init_done = False  # TF map→base_link が送った初期姿勢と一致したら True
        self.clock_t0 = None
        self.last_clock = None
        self.sent_at = None
        self.n_est = 0
        os.makedirs(run_dir, exist_ok=True)
        self.f_est = open(os.path.join(run_dir, "est.tum"), "w")
        self.f_cov = open(os.path.join(run_dir, "cov.txt"), "w")
        self.f_alpha = open(os.path.join(run_dir, "alpha.txt"), "w")
        self.f_align = open(os.path.join(run_dir, "alignment.jsonl"), "w")
        self.f_ev = open(os.path.join(run_dir, "events.txt"), "w")
        self.f_lat = open(os.path.join(run_dir, "latency.txt"), "w")
        self.last_est_stamp = None

        self.buf = Buffer(cache_time=rclpy.duration.Duration(seconds=120.0))  # bag 開始時の odom 姿勢を後から引くため長め
        self.tl = TransformListener(self.buf, self)
        self.init_pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        self.create_subscription(Clock, "/clock", self.on_clock, QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        for topic in ("/amcl_pose", "/mcl_pose", "/pcl_pose"):
            self.create_subscription(PoseWithCovarianceStamped, topic, lambda m, t=topic: self.on_pose(m, t), 10)
        self.create_subscription(Float32, "/alpha", self.on_alpha, 10)
        self.create_subscription(DiagnosticArray, "/alignment_status", self.on_align, 10)
        self.create_timer(1.0 / rate, self.on_tick)
        self.get_logger().info(f"recording to {run_dir}; initialpose after {delay:.1f} s of /clock, "
                               f"resend every {resend_every:.1f} s (max {resend_max}) until TF agrees "
                               f"(tol {tol_xy:.2f} m / {math.degrees(tol_yaw):.0f} deg)")

    def now_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def event(self, text):
        line = f"{self.now_sec():.3f} {text}"
        self.f_ev.write(line + "\n"); self.f_ev.flush()
        self.get_logger().info(line)

    def on_clock(self, msg):
        t = msg.clock.sec + msg.clock.nanosec * 1e-9
        if self.clock_t0 is None:
            self.clock_t0 = t
            self.event(f"clock_start {t:.3f}")
        self.last_clock = t

    def odom_at(self, when):
        """odom→base_link を (x, y, yaw, stamp) で。when=None なら最新"""
        tf = self.buf.lookup_transform("odom", "base_link", when if when is not None else rclpy.time.Time())
        x, y, yaw = _xyyaw(tf)
        return x, y, yaw, _stamp(tf.header)

    def expected_pose(self, odom_now):
        """bag 開始時の姿勢 init に、odom0 → odom_now の相対移動を足した map 座標の姿勢"""
        x0, y0, yaw0 = float(self.init["x"]), float(self.init["y"]), float(self.init["yaw"])
        ox0, oy0, oyaw0, _ = self.odom0
        ox, oy, oyaw, _ = odom_now
        dx, dy = ox - ox0, oy - oy0
        c, s = math.cos(-oyaw0), math.sin(-oyaw0)
        bx, by = c * dx - s * dy, s * dx + c * dy  # 相対移動 (開始時の body 座標)
        c, s = math.cos(yaw0), math.sin(yaw0)
        return x0 + c * bx - s * by, y0 + s * bx + c * by, yaw0 + (oyaw - oyaw0)

    def send_initialpose(self):
        if self.odom0 is None:
            # bag 開始時の odom 姿勢: initialpose.yaml の stamp で引く。無理なら最新 (開始直後は静止している前提)
            try:
                st = float(self.init.get("stamp", 0.0))
                self.odom0 = self.odom_at(rclpy.time.Time(seconds=st))
            except (LookupException, ConnectivityException, ExtrapolationException):
                try:
                    self.odom0 = self.odom_at(None)
                    self.event(f"warning: odom at init stamp unavailable, using latest ({self.odom0[3]:.3f})")
                except (LookupException, ConnectivityException, ExtrapolationException):
                    return  # odom がまだ無い → 次の tick で
        try:
            odom_now = self.odom_at(None)
        except (LookupException, ConnectivityException, ExtrapolationException):
            return
        x, y, yaw = self.expected_pose(odom_now)
        m = PoseWithCovarianceStamped()
        m.header.frame_id = "map"
        m.header.stamp = rclpy.time.Time(seconds=odom_now[3]).to_msg()  # 補正に使った odom の時刻 (未来側外挿を避ける)
        m.pose.pose.position.x = x
        m.pose.pose.position.y = y
        m.pose.pose.position.z = 0.0  # map 座標では z=0 (3D 側は map_level_bridge が traj から z を引く)
        m.pose.pose.orientation.z = math.sin(yaw / 2)
        m.pose.pose.orientation.w = math.cos(yaw / 2)
        cov = [0.0] * 36
        cov[0] = cov[7] = 0.25  # RViz の 2D Pose Estimate と同じ既定 (0.5 m)^2
        cov[35] = (math.pi / 12) ** 2  # (15 deg)^2
        m.pose.covariance = cov
        self.init_pub.publish(m)
        self.sent_at = self.now_sec()
        self.resend_left -= 1
        self.event(f"initialpose x={x:.3f} y={y:.3f} yaw={math.degrees(yaw):.1f}deg (odom moved "
                   f"{math.hypot(odom_now[0] - self.odom0[0], odom_now[1] - self.odom0[1]):.2f} m since start)")

    def check_init(self, tf):
        """送った初期姿勢に TF map→base_link が一致したか (その TF の時刻の odom で補正して比べる)"""
        try:
            odom_then = self.odom_at(rclpy.time.Time(seconds=_stamp(tf.header)))
        except (LookupException, ConnectivityException, ExtrapolationException):
            return
        ex, ey, eyaw = self.expected_pose(odom_then)
        x, y, yaw = _xyyaw(tf)
        dyaw = math.atan2(math.sin(yaw - eyaw), math.cos(yaw - eyaw))
        if math.hypot(x - ex, y - ey) <= self.tol_xy and abs(dyaw) <= self.tol_yaw:
            self.init_done = True
            self.event(f"init_ok tf=({x:.3f},{y:.3f},{math.degrees(yaw):.1f}deg) expected=({ex:.3f},{ey:.3f},{math.degrees(eyaw):.1f}deg)")

    def on_tick(self):
        if self.clock_t0 is None:
            return
        t = self.now_sec()
        # initialpose: /clock 開始から delay 秒後に送り、TF が一致するまで resend_every 秒ごとに送り直す
        if not self.init_done and self.resend_left > 0 and t - self.clock_t0 >= self.delay:
            if self.sent_at is None or t - self.sent_at >= self.resend_every:
                self.send_initialpose()
        try:
            tf = self.buf.lookup_transform("map", "base_link", rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException):
            return
        if not self.init_done and self.sent_at is not None:
            self.check_init(tf)
        p = tf.transform.translation
        q = tf.transform.rotation
        # stamp は TF 連鎖の「最新の共通時刻」(= map→odom と odom→base_link の古い方)。姿勢はその時刻のものなので
        # 基準との照合もその時刻で行う。「いま」との差 = 推定の鮮度 (latency.txt)。Nav2 も同じ lookup で姿勢を得るので、
        # 鮮度が悪い = 実運用で古い姿勢を使わされる、という別の指標になる (10-11: lidar3d で 10 s 周期の鋸歯状誤差を見て分離)
        ts = _stamp(tf.header)
        self.f_lat.write(f"{t:.3f} {t - ts:.3f}\n")
        if ts != self.last_est_stamp:
            self.f_est.write(f"{ts:.9f} {p.x:.6f} {p.y:.6f} {p.z:.6f} {q.x:.9f} {q.y:.9f} {q.z:.9f} {q.w:.9f}\n")
            self.last_est_stamp = ts
            self.n_est += 1
            if self.n_est % 200 == 0:
                self.f_est.flush()

    def on_pose(self, m, topic):
        c = m.pose.covariance
        self.f_cov.write(f"{_stamp(m.header):.9f} {topic} {c[0]:.6g} {c[7]:.6g} {c[35]:.6g}\n")

    def on_alpha(self, m):
        self.f_alpha.write(f"{self.now_sec():.9f} {m.data:.6g}\n")

    def on_align(self, m):
        for s in m.status:
            level = s.level[0] if isinstance(s.level, (bytes, bytearray)) else int(s.level)
            rec = {"stamp": _stamp(m.header), "level": level, "message": s.message}
            for kv in s.values:
                if kv.key in ALIGNMENT_KEYS:
                    rec[kv.key] = kv.value
            self.f_align.write(json.dumps(rec) + "\n")

    def close(self):
        for f in (self.f_est, self.f_cov, self.f_alpha, self.f_align, self.f_ev, self.f_lat):
            f.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run_dir")
    ap.add_argument("--init", required=True, help="make_reference.py の initialpose.yaml")
    ap.add_argument("--initialpose_delay", type=float, default=2.0)
    ap.add_argument("--resend_every", type=float, default=1.0)
    ap.add_argument("--resend_max", type=int, default=60)
    ap.add_argument("--tol_xy", type=float, default=1.0, help="初期姿勢が入ったとみなす TF との差 [m]")
    ap.add_argument("--tol_yaw_deg", type=float, default=15.0)
    ap.add_argument("--rate", type=float, default=20.0)
    args = ap.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    with open(args.init) as f:
        init = yaml.safe_load(f)
    rclpy.init()
    node = Recorder(args.run_dir, init, args.initialpose_delay, args.rate, args.resend_every, args.resend_max,
                    args.tol_xy, math.radians(args.tol_yaw_deg))
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
