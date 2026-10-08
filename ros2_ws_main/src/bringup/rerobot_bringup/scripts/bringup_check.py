#!/usr/bin/env python3
# claude: bringup 起動結果のサマリ表示ノード (2026-10-01 追加)。
#   rerobot_bringup.launch.py の最後に起動され、各コンポーネントが立ち上がったかを
#   1 回だけ判定して表にし、終了する (launch 全体は止めない)。読み取り専用:
#   トピックは raw 購読で周波数だけ数え、EPOS4 は SDO read で statusword (0x6041) を読むだけ。
#
#   判定:
#     EPOS4    ... statusword & 0x6F == 0x27 (Operation enabled) かつ joint_states が流れている
#     センサ類 ... 測定窓内の受信周波数が期待値の 50% 以上で OK、0 < 50% 未満は LOW、0 は NG
#
#   単体でも使える (稼働中スタックの再確認):
#     ros2 run rerobot_bringup bringup_check.py --ros-args -p lidar_2d:=false -p lidar_3d:=true
import time
import unicodedata

import rclpy
from canopen_interfaces.srv import CORead
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, JointState, LaserScan, PointCloud2

GREEN, YELLOW, RED, BOLD, RESET = "\033[32m", "\033[33m", "\033[31m", "\033[1m", "\033[0m"

# claude: CiA402 statusword (0x6041) の状態判定 (mask 0x6F)。stack-health skill と同じ表。
CIA402_STATES = {
    0x27: "Operation enabled",
    0x23: "Switched on",
    0x21: "Ready to switch on",
    0x40: "Switch on disabled",
    0x60: "Switch on disabled",
    0x07: "Quick stop active",
}


def _pad(text, width):
    # claude: 全角を 2 桁として数えて右詰め (日本語混じりでも列が揃うように)
    w = sum(2 if unicodedata.east_asian_width(c) in "WF" else 1 for c in text)
    return text + " " * max(0, width - w)


class BringupCheck(Node):
    def __init__(self):
        super().__init__("bringup_check")
        self.declare_parameter("lidar_2d", True)
        self.declare_parameter("lidar_3d", True)
        self.declare_parameter("rfans_scan", True)       # claude: 2026-10-08
        self.declare_parameter("rfans_scan_all", False)  # claude: 2026-10-08
        self.declare_parameter("imu", True)
        self.declare_parameter("imu_wit", True)
        self.declare_parameter("ekf", False)
        self.declare_parameter("imu_rate", 200.0)
        self.declare_parameter("rps", 10)
        self.declare_parameter("measure_sec", 4.0)
        p = lambda n: self.get_parameter(n).value  # noqa: E731

        # claude: (表示名, topic, 型, 期待 Hz, NG 時のヒント)。有効なものだけ並べる。
        self.topics = [
            ("EPOS4 右 (motor1) joint_states", "/motor1/cia402_device_1/joint_states",
             JointState, 20.0, "can0 / EPOS 電源 / bus.yml を確認 (./scripts/can_up.sh)"),
            ("EPOS4 左 (motor2) joint_states", "/motor2/cia402_device_2/joint_states",
             JointState, 20.0, "can0 / EPOS 電源 / bus.yml を確認 (./scripts/can_up.sh)"),
            ("車輪オドメトリ /odom", "/odom", Odometry, 20.0,
             "epos4_odometry が落ちていないか (params の ros__parameters 綴り)"),
        ]
        if p("lidar_2d"):
            # claude: 2026-10-08 /scan → /urg_front/scan (裸の /scan は誰も出さない規約)
            self.topics.append(("2D LiDAR (UTM-30LX) /urg_front/scan", "/urg_front/scan", LaserScan, 40.0,
                                "USB 接続 / /dev/ttyUSB-utm-30lx の有無"))
        if p("lidar_3d"):
            self.topics.append(("3D LiDAR (R-Fans-16) 点群", "/rfans_driver/rfans_points",
                                PointCloud2, float(p("rps")),
                                "LiDAR 電源 / Ethernet / device_ip (192.168.0.3)"))
            # claude: 2026-10-08 3D→2D (rfans_scan.launch.py)
            if p("rfans_scan"):
                self.topics.append(("3D→2D /rfans/scan", "/rfans/scan", LaserScan, float(p("rps")),
                                    "点群が出ているか / TF base_link->rfans"))
                if p("rfans_scan_all"):
                    self.topics.append(("3D→2D 全点 /rfans/scan_all", "/rfans/scan_all", LaserScan,
                                        float(p("rps")), "点群が出ているか / TF base_link->rfans"))
        if p("imu"):
            self.topics.append(("IMU (BNO086) /imu/data", "/imu/data", Imu,
                                float(p("imu_rate")), "USB 接続 / imu_port (/dev/ttyACM0)"))
            if p("imu_wit"):
                self.topics.append(("IMU (WT901C) /imu_wit/data", "/imu_wit/data", Imu, 200.0,
                                    "USB-TTL 接続 / imu_wit_port (ttyUSB-wt901)"))
        if p("ekf"):
            self.topics.append(("EKF /odometry/filtered", "/odometry/filtered", Odometry, 30.0,
                                "imu:=true か / config/ekf.yaml"))

        self.counts = {t[1]: 0 for t in self.topics}
        self.subs = []
        for _, topic, msg_type, _, _ in self.topics:
            # claude: raw=True でデシリアライズを省略 (点群・200 Hz IMU でも軽い)
            self.subs.append(self.create_subscription(
                msg_type, topic, lambda _m, t=topic: self._count(t),
                qos_profile_sensor_data, raw=True))

        self.sdo_clients = {
            name: self.create_client(CORead, f"{ns}/sdo_read")
            for name, ns in [("EPOS4 右 (motor1)", "/motor1/cia402_device_1"),
                             ("EPOS4 左 (motor2)", "/motor2/cia402_device_2")]
        }

    def _count(self, topic):
        self.counts[topic] += 1

    def _spin_for(self, sec):
        end = time.monotonic() + sec
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def _read_statusword(self, client):
        if not client.wait_for_service(timeout_sec=2.0):
            return None, "sdo_read サービス無し"
        fut = client.call_async(CORead.Request(index=0x6041, subindex=0))
        end = time.monotonic() + 3.0
        while not fut.done() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        if not fut.done() or not fut.result().success:
            return None, "SDO read 失敗"
        return fut.result().data, None

    def run(self):
        measure = float(self.get_parameter("measure_sec").value)
        self.get_logger().info(f"起動チェック中... ({measure:.0f} 秒間トピックを計測)")
        # claude: 先に購読を張ってから計測窓を開ける (discovery 待ちで頭の受信を落とさない)
        self._spin_for(1.0)
        self.counts = {k: 0 for k in self.counts}
        self._spin_for(measure)
        rates = {k: v / measure for k, v in self.counts.items()}

        rows = []  # (判定, 名前, 詳細, ヒント)
        for name, client in self.sdo_clients.items():
            sw, err = self._read_statusword(client)
            if sw is None:
                rows.append(("NG", f"{name} 状態", err, "bus_config の起動ログ (NMT START) を確認"))
                continue
            state = "Fault" if sw & 0x08 else CIA402_STATES.get(sw & 0x6F, "不明")
            ok = (sw & 0x6F) == 0x27
            rows.append(("OK" if ok else "NG", f"{name} 状態",
                         f"{state} (0x{sw:04X})",
                         "" if ok else "epos4_controller の init/enable 失敗 — bringup を再起動"))

        for name, topic, _, expect, hint in self.topics:
            hz = rates[topic]
            verdict = "OK" if hz >= 0.5 * expect else ("LOW" if hz > 0 else "NG")
            rows.append((verdict, name, f"{hz:6.1f} Hz (期待 {expect:.0f})",
                         "" if verdict == "OK" else hint))

        self._print(rows)

    def _print(self, rows):
        mark = {"OK": f"{GREEN}✔ OK {RESET}", "LOW": f"{YELLOW}▲ LOW{RESET}",
                "NG": f"{RED}✘ NG {RESET}"}
        n_ok = sum(r[0] == "OK" for r in rows)
        bar = "=" * 72
        lines = ["", bar, f"{BOLD} reRoBot bringup 起動結果{RESET}", bar]
        for verdict, name, detail, hint in rows:
            lines.append(f" {mark[verdict]}  {_pad(name, 34)} {detail}")
            if hint:
                lines.append(f"          → {hint}")
        lines.append(bar)
        color = GREEN if n_ok == len(rows) else RED
        lines.append(f"{color}{BOLD} {n_ok}/{len(rows)} 項目 OK"
                     f"{'' if n_ok == len(rows) else ' — NG / LOW の項目を確認してください'}{RESET}")
        lines.append(bar)
        print("\n".join(lines), flush=True)


def main():
    rclpy.init()
    node = BringupCheck()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
