#!/usr/bin/env python3
# claude 2026-11-06: epos4_controller の自動復帰・走行許可 (DISARMED) の結合試験。モータも CAN 実機も要らない。
#   擬似 EPOS ×2 (cia402 サービス + sdo_read + joint_states) を立て、EMCY は vcan0 に cansend で流す。
#   設計 docs/features/2026-11-05_epos_auto_recovery_plan.md §3、試験項目は同 §7 のうちソフトで再現できるもの。
#
#   実行 (rerobot_env コンテナ内。実機スタックと混ざらないよう別ドメインで):
#     ip link add dev vcan0 type vcan 2>/dev/null; ip link set up vcan0
#     source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash
#     ROS_DOMAIN_ID=177 python3 src/app/epos4_controller/test/mock_recovery_test.py [controller の実行ファイル]
#   既定の実行ファイルは /workspace/install/epos4_controller/lib/epos4_controller/epos4_controller。
import math
import os
import subprocess
import sys
import threading
import time

import rclpy
from canopen_interfaces.msg import COData
from canopen_interfaces.srv import CORead
from geometry_msgs.msg import Twist
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import String
from std_srvs.srv import Trigger

CTRL = sys.argv[1] if len(sys.argv) > 1 else "/workspace/install/epos4_controller/lib/epos4_controller/epos4_controller"
PARAMS = "/workspace/src/bringup/rerobot_bringup/config/params.yaml"
CAN = "vcan0"


class MockMotor:
    """擬似 EPOS4 1 台。fault / code / vcc を試験側から書き換える。"""

    def __init__(self, node, i):
        self.i, self.fault, self.code, self.vcc = i, False, 0, 24.0
        self.target, self.pos, self.recover_calls = 0.0, 0.0, 0
        ns = f"/motor{i}/cia402_device_{i}"
        for name in ["init", "enable", "cyclic_velocity_mode", "disable", "halt", "shutdown", "velocity_mode"]:
            node.create_service(Trigger, f"{ns}/{name}", self.on_ok)
        node.create_service(Trigger, f"{ns}/recover", self.on_recover)
        node.create_service(CORead, f"{ns}/sdo_read", self.on_sdo)
        node.create_subscription(COData, f"{ns}/tpdo", self.on_tpdo, 10)
        self.js_pub = node.create_publisher(JointState, f"{ns}/joint_states", 10)

    def on_ok(self, req, res):
        res.success = True
        return res

    def on_recover(self, req, res):
        self.recover_calls += 1
        res.success = self.vcc >= 20.0  # 電源が無ければ Fault は解けない
        if res.success:
            self.fault, self.code = False, 0
        return res

    def on_sdo(self, req, res):
        res.success = True
        idx, sub = req.index, req.subindex
        if idx == 0x6041:
            res.data = 0x0208 if self.fault else 0x1237
        elif idx == 0x6061:
            res.data = 9
        elif idx == 0x603F:
            res.data = self.code if self.fault else 0
        elif idx == 0x2200:
            res.data = int(self.vcc * 10)
        elif idx == 0x1003:
            res.data = (1 if self.fault else 0) if sub == 0 else self.code
        else:
            res.data = 0
        return res

    def on_tpdo(self, m):
        if m.index == 0x60FF:
            d = m.data
            self.target = float(d - (1 << 32) if d >= (1 << 31) else d)

    def step(self, dt):
        if not self.fault:  # Fault 中は車輪が止まっている
            self.pos += self.target * 2 * math.pi / 60.0 * dt
        js = JointState()
        js.name, js.position, js.velocity, js.effort = ["j"], [self.pos], [0.0], [0.0]
        self.js_pub.publish(js)

    def set_fault(self, code, vcc=None):
        self.code, self.fault = code, True
        if vcc is not None:
            self.vcc = vcc


class Harness(Node):
    def __init__(self):
        super().__init__("mock_recovery_test")
        self.m = [MockMotor(self, 1), MockMotor(self, 2)]
        self.state = ""
        tl = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, "/epos4_controller/state", self.on_state, tl)
        self.cmd_pub = self.create_publisher(Twist, "/robot_speed_cmd", 10)
        self.joy_pub = self.create_publisher(Joy, "/joy", 10)
        self.joyvel_pub = self.create_publisher(Twist, "/joy_vel", 10)
        self.nav_v = 0.0
        self.joy_on, self.lb, self.rb, self.start, self.joy_v = False, False, False, False, 0.0
        self.create_timer(0.05, self.tick)

    def on_state(self, msg):
        if msg.data != self.state:
            print(f"      [state] {msg.data}", flush=True)
        self.state = msg.data

    def tick(self):
        for m in self.m:
            m.step(0.05)
        t = Twist()
        t.linear.x = self.nav_v
        self.cmd_pub.publish(t)
        if self.joy_on:
            j = Joy()
            j.axes = [0.0] * 8
            j.buttons = [0] * 11
            j.buttons[4] = int(self.lb)
            j.buttons[5] = int(self.rb)
            j.buttons[7] = int(self.start)
            self.joy_pub.publish(j)
            if self.lb or self.rb:  # teleop_twist_joy は LB でも RB (turbo) 単独でも指令を出す
                tv = Twist()
                tv.linear.x = self.joy_v
                self.joyvel_pub.publish(tv)

    def targets(self):
        return [abs(m.target) for m in self.m]


def emcy(motor, code):
    data = f"{code & 0xFF:02X}{(code >> 8) & 0xFF:02X}000000000000"
    subprocess.run(["cansend", CAN, f"{0x080 + motor:03X}#{data}"], check=True)


results = []


def check(name, ok, detail=""):
    results.append((name, ok, detail))
    print(f"  {'PASS' if ok else 'FAIL'}  {name}  {detail}", flush=True)


def wait_state(h, prefix, timeout):
    end = time.time() + timeout
    while time.time() < end:
        if h.state.startswith(prefix):
            return True
        time.sleep(0.02)
    return False


def press_start(h):
    h.start = True
    time.sleep(0.15)
    h.start = False
    time.sleep(0.15)


def main():
    rclpy.init()
    h = Harness()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(h)
    threading.Thread(target=ex.spin, daemon=True).start()
    log = open("/tmp/mock_recovery_ctrl.log", "w")
    ctrl = subprocess.Popen([CTRL, "--ros-args", "--params-file", PARAMS, "-p", f"can_interface:={CAN}"],
                            stdout=log, stderr=subprocess.STDOUT)
    try:
        print("0. 起動 → DISARMED", flush=True)
        check("起動直後は DISARMED", wait_state(h, "DISARMED 起動完了", 40), h.state)

        print("1. DISARMED 中の Nav2 指令は通らない", flush=True)
        h.nav_v = 0.2
        time.sleep(1.0)
        check("Nav2 0.2 m/s でも出力 0", max(h.targets()) == 0.0, str(h.targets()))

        print("2. LB を握った joy は通る", flush=True)
        h.joy_on, h.lb, h.joy_v = True, True, 0.1
        time.sleep(0.8)
        check("LB + joy_vel で出力あり", min(h.targets()) > 100, str(h.targets()))
        h.lb, h.rb = False, True
        time.sleep(0.8)
        check("RB (turbo) 単独 + joy_vel でも出力あり", min(h.targets()) > 100, str(h.targets()))
        h.lb, h.rb = True, False

        print("3. パッド瞬断 (LB を握ったまま /joy と /joy_vel が止まる) → 0", flush=True)
        h.joy_on = False
        time.sleep(1.0)
        check("joy が途絶えたら出力 0 (Nav2 は通らない)", max(h.targets()) == 0.0, str(h.targets()))

        print("4. 再接続直後に Start が押されていても走行許可しない → 離して押せば許可", flush=True)
        h.lb, h.rb, h.start, h.joy_on = False, False, True, True
        time.sleep(0.6)
        check("再接続直後の押しっぱなし Start では DISARMED のまま", h.state.startswith("DISARMED"), h.state)
        h.start = False
        time.sleep(0.2)
        press_start(h)
        check("Start で RUNNING", wait_state(h, "RUNNING", 3), h.state)
        time.sleep(0.6)
        check("RUNNING で Nav2 指令が出る", min(h.targets()) > 100, str(h.targets()))

        print("5. 段差ブラウンアウト (motor1 だけ 0x3220、Vcc 24 V) → 自動復帰", flush=True)
        rc = h.m[0].recover_calls
        h.m[0].set_fault(0x3220, 24.0)
        emcy(1, 0x3220)
        check("RECOVERING を経て RUNNING", wait_state(h, "RECOVERING", 1) and wait_state(h, "RUNNING", 4), h.state)
        check("motor1 に recover が呼ばれた", h.m[0].recover_calls > rc, f"{rc}→{h.m[0].recover_calls}")

        print("6. 両輪 0x3220・Vcc 24 V (段差で両輪、ユーザ指示で自動復帰)", flush=True)
        h.m[0].set_fault(0x3220, 24.0)
        h.m[1].set_fault(0x3220, 24.0)
        emcy(1, 0x3220)
        emcy(2, 0x3220)
        check("両輪の段差も自動で RUNNING", wait_state(h, "RECOVERING", 1) and wait_state(h, "RUNNING", 4), h.state)

        print("7. 0x8130 (heartbeat) 両輪 → 自動復帰", flush=True)
        h.m[0].set_fault(0x8130)
        h.m[1].set_fault(0x8130)
        emcy(1, 0x8130)
        emcy(2, 0x8130)
        check("0x8130 は自動で RUNNING", wait_state(h, "RECOVERING", 1) and wait_state(h, "RUNNING", 4), h.state)

        print("8. 非常停止 (両輪 0x3220、Vcc 8 V) → WAIT_POWER → 解除後は DISARMED で走らない", flush=True)
        h.m[0].set_fault(0x3220, 8.0)
        h.m[1].set_fault(0x3220, 8.0)
        emcy(1, 0x3220)
        emcy(2, 0x3220)
        check("WAIT_POWER に入る", wait_state(h, "WAIT_POWER", 2), h.state)
        time.sleep(3.0)  # ラッチ式: 解除まで 3 s
        check("非常停止中は出力 0", max(h.targets()) == 0.0, str(h.targets()))
        h.m[0].vcc = h.m[1].vcc = 24.0
        check("電源が戻ったら DISARMED", wait_state(h, "DISARMED", 5), h.state)
        check("enable 済み (Fault 解除)", not h.m[0].fault and not h.m[1].fault)
        time.sleep(1.0)
        check("解除後も Nav2 指令は通らない", max(h.targets()) == 0.0, str(h.targets()))
        press_start(h)
        check("Start で RUNNING", wait_state(h, "RUNNING", 3), h.state)

        print("9. 予算: 0x8130 を短時間に 4 回 → 4 回目は DISARMED", flush=True)
        for k in range(4):
            h.m[0].set_fault(0x8130)
            emcy(1, 0x8130)
            time.sleep(1.5)
        check("4 回目で DISARMED (予算超過)", h.state.startswith("DISARMED") and "予算" in h.state, h.state)
        h.m[0].fault = False
        press_start(h)
        check("Start で RUNNING", wait_state(h, "RUNNING", 3), h.state)

        print("10. 0x8250 → 自動復帰、30 s 以内の再発は DISARMED", flush=True)
        h.m[0].set_fault(0x8250)
        h.m[1].set_fault(0x8250)
        emcy(1, 0x8250)
        emcy(2, 0x8250)
        check("0x8250 は自動で RUNNING", wait_state(h, "RECOVERING", 1) and wait_state(h, "RUNNING", 4), h.state)
        time.sleep(1.0)
        h.m[0].set_fault(0x8250)
        emcy(1, 0x8250)
        check("すぐ再発したら DISARMED", wait_state(h, "DISARMED", 3), h.state)
        h.m[0].fault = False
        press_start(h)
        check("Start で RUNNING", wait_state(h, "RUNNING", 3), h.state)

        print("11. EMCY なしの Fault (取りこぼし) → 物理トリガで検出", flush=True)
        time.sleep(1.0)
        h.m[1].set_fault(0x8130)
        check("物理トリガ → RECOVERING → RUNNING", wait_state(h, "RECOVERING", 3) and wait_state(h, "RUNNING", 4), h.state)

        print("12. EMCY なしの 0x3220 (時刻不明) → 安全側で非常停止扱い", flush=True)
        time.sleep(1.0)
        h.m[0].set_fault(0x3220, 24.0)
        check("WAIT_POWER (時刻不明)", wait_state(h, "WAIT_POWER", 3), h.state)
        check("Vcc が正常なので DISARMED へ", wait_state(h, "DISARMED", 5), h.state)
    finally:
        ctrl.send_signal(2)
        try:
            ctrl.wait(timeout=5)
        except subprocess.TimeoutExpired:
            ctrl.kill()
        log.close()
        n_ok = sum(1 for _, ok, _ in results if ok)
        print(f"\n結果: {n_ok}/{len(results)} PASS (controller のログ: /tmp/mock_recovery_ctrl.log)", flush=True)
        ex.shutdown()
        rclpy.shutdown()
        sys.exit(0 if n_ok == len(results) else 1)


if __name__ == "__main__":
    main()
