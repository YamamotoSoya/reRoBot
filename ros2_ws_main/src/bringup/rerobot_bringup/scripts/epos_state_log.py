#!/usr/bin/env python3
# claude: 2026-11-06 EPOS の状態表示をログに残す (auto_run が起動する)。
#   epos4_controller の /epos4_controller/state (走行許可・自動復帰の状態機械) が変わるたびに JST 時刻付きで 1 行、
#   /diagnostics の epos4_controller/motor* が Fault になったらコード・履歴・Vcc を 1 行出す。
#   出力先 = 画面 (auto_run を tee しているログに入る) + log_file パラメータのファイル (追記)。
#   bringup を別端末で上げる運用 (auto_run bringup:=false) でも、EPOS の出来事が auto_run 側のログに揃う。
#   設計 docs/features/2026-11-05_epos_auto_recovery_plan.md
import datetime
import zoneinfo

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

JST = zoneinfo.ZoneInfo("Asia/Tokyo")
RED, YELLOW, GREEN, RESET = "\033[31m", "\033[33m", "\033[32m", "\033[0m"


class EposStateLog(Node):
    def __init__(self):
        super().__init__("epos_state_log")
        self.log_file = self.declare_parameter("log_file", "").value
        self.no_state_warn_s = float(self.declare_parameter("no_state_warn_s", 30.0).value)
        self.fp = open(self.log_file, "a", encoding="utf-8") if self.log_file else None
        self.state = None
        self.motor_fault = {}  # diagnostics の名前 → 直前の Fault メッセージ (None = 正常)
        self.link = None
        tl = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                        history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(String, "/epos4_controller/state", self.on_state, tl)
        self.create_subscription(DiagnosticArray, "/diagnostics", self.on_diag, 50)
        self.t0 = self.get_clock().now()
        self.alive_timer = self.create_timer(self.no_state_warn_s, self.check_alive)
        self.emit(f"EPOS 状態ログ開始 (file: {self.log_file or 'なし'})", "")

    def emit(self, text, color):
        stamp = datetime.datetime.now(JST).strftime("%H:%M:%S.%f")[:-3]
        line = f"[EPOS {stamp}] {text}"
        print(f"{color}{line}{RESET}" if color else line, flush=True)
        if self.fp:
            self.fp.write(line + "\n")
            self.fp.flush()

    def on_state(self, msg):
        if msg.data == self.state:
            return
        self.state = msg.data
        if msg.data.startswith("RECOVERING Fault 検知"):
            self.motor_fault.clear()  # 新しい Fault のたびに輪ごとの詳細を出し直す (同じコードの連続でも省かない)
        color = GREEN if msg.data.startswith("RUNNING") else (RED if msg.data.startswith("WAIT_POWER") else YELLOW)
        self.emit(f"状態 {msg.data}", color)

    def on_diag(self, msg):
        for s in msg.status:
            if not s.name.startswith("epos4_controller/"):
                continue
            kv = {v.key: v.value for v in s.values}
            if s.name == "epos4_controller/can_link":
                link = f"{s.message} (emcy_socket={kv.get('emcy_socket')})"
                if link != self.link:
                    prev, self.link = self.link, link
                    bad = s.message != "OK" or kv.get("emcy_socket") == "down"
                    if bad:
                        self.emit(f"CAN リンク: {link}", RED)
                    elif prev is not None:
                        self.emit(f"CAN リンク: 回復 {link}", "")
                continue
            if not s.name.startswith("epos4_controller/motor"):
                continue
            fault = s.message if s.message.startswith("FAULT") else None
            if fault != self.motor_fault.get(s.name, None):
                self.motor_fault[s.name] = fault
                motor = s.name.split("/", 1)[1]
                if fault:
                    self.emit(f"{motor} {fault}  history=[{kv.get('error_history', '').strip()}]  "
                              f"Vcc={kv.get('supply_voltage_V', '?')} V  statusword={kv.get('statusword', '?')}", RED)
                else:
                    self.emit(f"{motor} Fault 解除 ({s.message})", "")

    def check_alive(self):
        self.alive_timer.cancel()  # 1 回だけ
        if self.state is None:
            self.emit(f"⚠ /epos4_controller/state が {self.no_state_warn_s:.0f} s 届かない "
                      "(bringup が上がっていない / epos4_controller が旧版で未再ビルド)", YELLOW)

    def destroy_node(self):
        if self.fp:
            self.fp.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = EposStateLog()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
