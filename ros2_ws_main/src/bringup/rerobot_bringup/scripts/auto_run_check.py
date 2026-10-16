#!/usr/bin/env python3
# claude: 2026-10-16 自律走行 (auto_run.launch.py) の全部品の起動結果を 1 枚の表で出す。
#   auto_run は bringup / joy / Nav2 / waypoint_runner / RViz / bag を時間差で起動するので、個々のログが
#   流れていくと「どのセンサが落ちたか」「Nav2 は active か」「runner は待機に入ったか」が追えなくなる
#   (10-16 ユーザ指摘)。本ノードは auto_run の最後 (55 s) に 1 回走って表を出し、終了する。
#   単体でも何度でも再確認できる:
#     ros2 run rerobot_bringup auto_run_check.py            # 既定 = auto_run の構成 (joy/rviz/record/runner すべて on)
#     ros2 run rerobot_bringup auto_run_check.py --ros-args -p rviz:=false -p record:=false
#
#   判定の中身 (読み取り専用):
#     bringup   ... bringup_check.py と同じ (EPOS4 statusword SDO + センサ周波数)。同じディレクトリから import して再利用
#     速度指令   ... /robot_speed_cmd の publisher が twist_mux 1 つだけか (2 つ以上 = 旧配線が残っている)
#     パッド     ... joy_node / teleop_twist_joy_node が居るか + /dev/input/js0 の有無
#     Nav2      ... lifecycle 7 ノードの get_state が active か / /map と /keepout_filter_mask (transient_local) を受け取れるか
#     自己位置   ... map→odom TF が流れているか (初期位置を置く前は「未」で正常 → ▲ で表示)
#     走行       ... waypoint_runner ノード + navigate_through_poses action + runner の最新状態 (/waypoint_runner/status)
#     表示・記録 ... rviz2 / rosbag2_recorder ノードの有無
import os
import sys
import time

import rclpy
from lifecycle_msgs.srv import GetState
from rclpy.action.graph import get_action_server_names_and_types_by_node
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))  # bringup_check.py は同じ install ディレクトリにある
import bringup_check as bc  # noqa: E402

GREEN, YELLOW, RED, BOLD, RESET = bc.GREEN, bc.YELLOW, bc.RED, bc.BOLD, bc.RESET


def _width(text):
    return sum(2 if bc.unicodedata.east_asian_width(c) in "WF" else 1 for c in text)  # 全角 2 桁
STATE_NAMES = {0: "unknown", 1: "unconfigured", 2: "inactive", 3: "active", 4: "finalized"}
# (表示名, ノード名, NG 時のヒント)
LIFECYCLE_NODES = [
    ("地図 map_server", "map_server", "map_dir の nav2/map.yaml を確認"),
    ("keepout filter_mask_server", "filter_mask_server", "keep_out/ のマスクを確認"),
    ("自己位置 amcl", "amcl", "loc_amcl.launch.py のログ / loc_map_yaml"),
    ("Nav2 controller_server", "controller_server", "nav2_params.yaml / navigation.launch.py のログ"),
    ("Nav2 planner_server", "planner_server", "同上"),
    ("Nav2 behavior_server", "behavior_server", "同上"),
    ("Nav2 bt_navigator", "bt_navigator", "同上 (plugin_lib_names を列挙すると segfault)"),
]
TL_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                    history=HistoryPolicy.KEEP_LAST, depth=1)


class AutoRunCheck(Node):
    def __init__(self):
        super().__init__("auto_run_check")
        for name in ("joy", "rviz", "record", "runner"):
            self.declare_parameter(name, True)
        self.declare_parameter("measure_sec", 3.0)
        self.flag = lambda n: bool(self.get_parameter(n).value)  # noqa: E731

        self.got_map = self.got_mask = self.got_map_odom = False
        self.runner_status = None
        self.create_subscription(OccupancyGrid, "/map", lambda _m: setattr(self, "got_map", True), TL_QOS, raw=True)
        self.create_subscription(OccupancyGrid, "/keepout_filter_mask",
                                 lambda _m: setattr(self, "got_mask", True), TL_QOS, raw=True)
        self.create_subscription(String, "/waypoint_runner/status", self._on_status, TL_QOS)
        self.create_subscription(TFMessage, "/tf", self._on_tf, 50)

    def _on_status(self, msg):
        self.runner_status = msg.data

    def _on_tf(self, msg):
        for t in msg.transforms:
            if t.header.frame_id == "map" and t.child_frame_id == "odom":
                self.got_map_odom = True

    def _spin_for(self, sec):
        end = time.monotonic() + sec
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def _lifecycle_state(self, node_name):
        """(state_label | None, err)"""
        client = self.create_client(GetState, f"/{node_name}/get_state")
        if not client.wait_for_service(timeout_sec=1.0):
            return None, "get_state サービス無し (ノード未起動?)"
        fut = client.call_async(GetState.Request())
        end = time.monotonic() + 2.0
        while not fut.done() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        if not fut.done():
            return None, "get_state 応答なし (負荷で詰まっている?)"
        return STATE_NAMES.get(fut.result().current_state.id, "?"), None

    def collect(self):
        rows = []  # ("SEC", title) か (判定, 名前, 詳細, ヒント)
        nodes = set(self.get_node_names())

        # --- bringup (bringup_check を再利用。auto_run の bringup 構成 = 既定 + rfans_scan_all + ekf) ---
        rows.append(("SEC", "bringup (EPOS4 / センサ / EKF)"))
        bu = bc.BringupCheck(node_name="auto_run_check_bringup", parameter_overrides=[
            Parameter("rfans_scan_all", value=True), Parameter("ekf", value=True),
            Parameter("measure_sec", value=float(self.get_parameter("measure_sec").value)),
        ])
        try:
            rows += bu.collect_rows()
        finally:
            bu.destroy_node()

        # --- 速度指令 ---
        rows.append(("SEC", "速度指令"))
        pubs = sorted(p.node_name for p in self.get_publishers_info_by_topic("/robot_speed_cmd"))
        if pubs == ["twist_mux"]:
            rows.append(("OK", "twist_mux → /robot_speed_cmd", "publisher = twist_mux のみ", ""))
        elif "twist_mux" in pubs:
            rows.append(("LOW", "twist_mux → /robot_speed_cmd", f"publisher が複数: {pubs}",
                         "joy_teleop / Nav2 が直接書いている (旧配線の launch が動いていないか)"))
        else:
            rows.append(("NG", "twist_mux → /robot_speed_cmd", f"publisher: {pubs or 'なし'}",
                         "bringup の twist_mux が落ちている (ros-jazzy-twist-mux 未インストール?)"))
        if self.flag("joy"):
            js = os.path.exists("/dev/input/js0")
            have = {n for n in ("joy_node", "teleop_twist_joy_node", "teach_marker") if n in nodes}
            miss = {"joy_node", "teleop_twist_joy_node"} - have
            if not miss and js:
                rows.append(("OK", "パッド (joy_node / teleop_twist_joy)", "ノード起動済み, /dev/input/js0 あり",
                             ""))
            elif not miss:
                rows.append(("LOW", "パッド (joy_node / teleop_twist_joy)", "ノードは居るが /dev/input/js0 が無い",
                             "パッドの USB を挿し直す (joy_node は自動で再オープンする)"))
            else:
                rows.append(("NG", "パッド (joy_node / teleop_twist_joy)", f"未起動: {sorted(miss)}",
                             "joy_teleop.launch.py のログを確認"))

        # --- Nav2 ---
        rows.append(("SEC", "Nav2 / 地図 / 自己位置"))
        for name, node_name, hint in LIFECYCLE_NODES:
            state, err = self._lifecycle_state(node_name)
            if state is None:
                rows.append(("NG", name, err, hint))
            elif state == "active":
                rows.append(("OK", name, "active", ""))
            else:
                rows.append(("LOW", name, state, "lifecycle_manager が activate できていない — " + hint))
        rows.append(("OK" if self.got_map else "NG", "/map 受信 (transient_local)",
                     "受信" if self.got_map else "未受信", "" if self.got_map else "map_server が active か"))
        rows.append(("OK" if self.got_mask else "NG", "/keepout_filter_mask 受信",
                     "受信" if self.got_mask else "未受信", "" if self.got_mask else "filter_mask_server が active か"))
        rows.append(("OK" if "map_odom_holder" in nodes else "NG", "map_odom_holder (TF 保持)",
                     "起動済み" if "map_odom_holder" in nodes else "未起動", "" if "map_odom_holder" in nodes else "loc_amcl.launch.py"))
        rows.append(("OK" if self.got_map_odom else "LOW", "自己位置 map→odom TF",
                     "AMCL が出力中" if self.got_map_odom else "未 (初期位置がまだ)",
                     "" if self.got_map_odom else "RViz の 2D Pose Estimate で置く (置く前はこれで正常)"))

        # --- 走行 ---
        rows.append(("SEC", "走行 / 表示 / 記録"))
        if self.flag("runner"):
            # claude: サービス名一覧は action **クライアント** (runner 自身) でも見えてしまうので、bt_navigator ノードが
            #   持つアクションサーバで判定する (ノードが居なければ例外 → なし)
            try:
                servers = get_action_server_names_and_types_by_node(self, "bt_navigator", "/")
                action_ok = any(n == "/navigate_through_poses" for n, _ in servers)
            except Exception:
                action_ok = False
            if "waypoint_runner" not in nodes:
                rows.append(("NG", "waypoint_runner", "ノード未起動",
                             "course.yaml が無い / 起動ログの traceback を確認。端末から ros2 run rerobot_bringup waypoint_runner.py"))
            elif self.runner_status is None:
                rows.append(("LOW", "waypoint_runner", "起動済み・状態未発行 (Nav2 サーバ待ち?)",
                             "" if action_ok else "navigate_through_poses action が無い (bt_navigator 未 active)"))
            else:
                rows.append(("OK", "waypoint_runner", self.runner_status, ""))
            rows.append(("OK" if action_ok else "NG", "Nav2 action navigate_through_poses",
                         "あり" if action_ok else "なし", "" if action_ok else "bt_navigator が active か"))
        if self.flag("rviz"):
            rows.append(("OK" if "rviz2" in nodes else "NG", "RViz (nav2.rviz)",
                         "起動済み" if "rviz2" in nodes else "未起動 (閉じた後なら正常)", ""))
        if self.flag("record"):
            rec = [n for n in nodes if "rosbag2_recorder" in n]
            rows.append(("OK" if rec else "NG", "bag record", "記録中" if rec else "未起動",
                         "" if rec else "auto_run の bag_record のログ (bags/raw の空き容量)"))
        return rows

    def print_rows(self, rows):
        mark = {"OK": f"{GREEN}✔ OK {RESET}", "LOW": f"{YELLOW}▲    {RESET}", "NG": f"{RED}✘ NG {RESET}"}
        bar = "=" * 76
        lines = ["", bar, f"{BOLD} auto_run 起動結果 (全部品)  — 再確認: ros2 run rerobot_bringup auto_run_check.py{RESET}", bar]
        n_ok = n_all = 0
        for r in rows:
            if r[0] == "SEC":
                lines.append(f"{BOLD}── {r[1]} {'─' * max(0, 72 - _width(r[1]))}{RESET}")
                continue
            verdict, name, detail, hint = r
            n_all += 1
            n_ok += verdict == "OK"
            lines.append(f" {mark[verdict]}  {bc._pad(name, 36)} {detail}")
            if hint:
                lines.append(f"          → {hint}")
        lines.append(bar)
        n_ng = sum(r[0] == "NG" for r in rows)
        color = GREEN if n_ng == 0 else RED
        lines.append(f"{color}{BOLD} {n_ok}/{n_all} 項目 OK"
                     f"{'' if n_ng == 0 else f' — ✘ {n_ng} 件を確認してください'}"
                     f"{'  (▲ は初期位置を置く前なら正常)' if n_ng == 0 and n_ok < n_all else ''}{RESET}")
        lines.append(bar)
        print("\n".join(lines), flush=True)


def main():
    rclpy.init()
    node = AutoRunCheck()
    try:
        node._spin_for(1.0)  # discovery + transient_local の初回受信
        node.print_rows(node.collect())
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
