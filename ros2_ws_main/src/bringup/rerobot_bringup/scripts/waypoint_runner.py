#!/usr/bin/env python3
# claude: 停止点つき waypoint 走行ノード (2026-10-04 追加)。
#   waypoint YAML (tools/waypoint_editor / RViz Nav2 パネル「Load WPs」と同じ形式) を読み、
#   `stop: true` が付いた点で止まり、キーボードの再開キーが押されるまで待つ。
#
#   仕組み: コースを停止点で区間に切り、区間ごとに NavigateThroughPoses を 1 本ずつ送る。
#     waypoint0 .. 停止点A | 停止点A の次 .. 停止点B | ... | .. 最後の点
#   区間の最後の点 = 停止点なので、Nav2 はそこで goal checker の許容内まで寄せて旋回し止まる
#   (途中の点は既定 BT の RemovePassedGoals で通過扱い — RViz の Start Nav Through Poses と同じ挙動)。
#   停止点の向きは YAML の orientation (waypoint_editor 既定 = 次の点の方向) が効く。
#   ⚠️ RViz パネルの「Start Nav Through Poses」は stop を読まない (全点を 1 本で送る)。止めたいときは本ノードで走らせる。
#
#   キー (端末で 1 文字。Enter 不要):
#     待機中 : 再開キー (既定 g) = 次の区間へ / q = 終了
#     走行中 : p = 一時停止 (ゴールをキャンセルしてその場で待機、再開キーで残りの点から続ける)
#     Ctrl-C : 走行中ならゴールをキャンセルして終了
#   claude: 2026-10-16 ゲームパッドのボタンでも同じ操作ができる (/joy を購読。class Input):
#     start_button (既定 7 = Start) = 再開キー / pause_button (既定 6 = Back) = p
#     start_requires_enable (既定 false) を true にすると LB (enable_button 4) を握りながら Start でないと開始しない (誤爆防止オプション)
#     ボタン番号は config/joy_teleop.yaml の waypoint_runner 節 (--ros-args --params-file で渡す。auto_run.launch.py が渡す)
#     端末が無い (launch 配下) ときはキーボードは無効で joy だけ。端末があれば両方使える。
#     本番の運用フロー (待機エリアで起動 → joy → RViz で initialpose → スタート列 → Start) は
#     docs/features/2026-10-16_auto_run_joy_mux.md
#   区間が失敗 (ABORTED) したときは、まず --retry-wait 秒待って残りの点を自動再送する (--auto-retry 回まで、
#   既定 1 回)。それでも失敗したら待機に入り、再開キーで残りの点から再送する。
#   claude: 2026-10-16 自動再送を追加。本番は人が g を押せないので、一時的な失敗 (TF 失効 102・AMCL 跳びで
#     経路が空 103・planner の一瞬の失敗) は自分で続きを始める。1 回だけなのは、本当に通れない場所で
#     送り直し続けても意味がないから (10-15 bag 1802 T=426 の 103 は数十秒後の再送で到達した)。
#
#   使い方 (rerobot_env 内、nav2.launch.py 起動 + AMCL 初期位置合わせ済みで。キー入力を使うなら -it):
#     ros2 run rerobot_bringup waypoint_runner.py <course.yaml> [--resume-key g] [--start N] [--auto-retry 1] [--retry-wait 5] \
#       [--ros-args --params-file <share>/config/joy_teleop.yaml]   # joy ボタンの割り当てを読む (省略時は既定値)
#     --start N : waypointN から始める (途中で止めて再起動したとき用)
#     --auto-retry N : 区間失敗の自動再送回数 (0 で無効 = 旧挙動)
#     --retry-wait S : 再送までの待ち秒数 (R-Fans ディップは 5〜7 s なので既定 5)
import argparse
import collections
import math
import select
import sys
import termios
import time
import tty

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Joy  # claude: 2026-10-16 ゲームパッド入力
from std_msgs.msg import String
from visualization_msgs.msg import Marker

GREEN, YELLOW, RED, BOLD, RESET = "\033[32m", "\033[33m", "\033[31m", "\033[1m", "\033[0m"


def load_course(path):
    """戻り値: [(name, x, y, qz, qw, stop), ...] (記述順)。orientation は [w, x, y, z] (panel 形式)。"""
    d = yaml.safe_load(open(path)) or {}
    wps = []
    for name, wp in (d.get("waypoints") or {}).items():  # PyYAML は記述順を保つ
        x, y = float(wp["pose"][0]), float(wp["pose"][1])
        w, _, _, z = (float(v) for v in wp["orientation"])
        wps.append((name, x, y, z, w, bool(wp.get("stop", False))))
    return wps


def split_segments(n, stops, start):
    """index start..n-1 を停止点 (と最後の点) で区切った [(i0, i1), ...] (両端含む)。"""
    segs, i0 = [], start
    for i in range(start, n):
        if stops[i] or i == n - 1:
            segs.append((i0, i))
            i0 = i + 1
    return segs


class Keyboard:
    """端末を cbreak にして 1 文字ずつ非ブロッキングで読む。終了時に必ず元に戻す。"""

    def __enter__(self):
        self.fd = sys.stdin.fileno()
        self.old = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        return self

    def __exit__(self, *exc):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)

    def get(self):
        if select.select([sys.stdin], [], [], 0)[0]:
            return sys.stdin.read(1)
        return None


class Input:
    """キーボードと /joy のボタンを同じ「1 文字イベント」に正規化する (2026-10-16)。
    - Start ボタンの立ち上がり → 再開キー、Back ボタンの立ち上がり → "p"。既存のキー処理がそのまま使える。
    - stdin が端末のときだけ Keyboard も有効 (launch 配下では None)。
    - joy_node は autorepeat で同じ状態を 20 Hz で再送するので、ボタンごとに前回状態を覚えて立ち上がりだけ拾う。
    - button_debounce 秒以内の連続イベントは捨てる (スタート列で 2 度押しになっても 1 回扱い)。
    ボタン番号はノードパラメータ (config/joy_teleop.yaml の waypoint_runner 節)。"""

    def __init__(self, node, resume_key):
        self.node, self.resume_key = node, resume_key
        self.kb = Keyboard() if sys.stdin.isatty() else None
        self.queue = collections.deque()
        self.prev = {}  # button index → 前回押下状態
        self.last_push = -1e9  # 直前にイベントを積んだ時刻 (monotonic)
        p = node.declare_parameter
        self.start_button = int(p("start_button", 7).value)
        self.pause_button = int(p("pause_button", 6).value)
        self.enable_button = int(p("enable_button", 4).value)  # teleop_twist_joy の LB と同じ番号にする
        self.start_requires_enable = bool(p("start_requires_enable", False).value)  # 10-16 ユーザ指示: 既定は Start 単独
        self.debounce = float(p("button_debounce", 1.0).value)
        node.create_subscription(Joy, "joy", self.on_joy, 10)

    def describe(self):
        guard = f"LB({self.enable_button})+" if self.start_requires_enable else ""
        return (f"joy: {guard}button {self.start_button} = 開始/再開, button {self.pause_button} = 一時停止"
                + ("、キーボードも有効" if self.kb else "、キーボードは無効 (端末なし)"))

    def __enter__(self):
        if self.kb:
            self.kb.__enter__()
        return self

    def __exit__(self, *exc):
        if self.kb:
            self.kb.__exit__(*exc)

    def on_joy(self, msg):
        def pressed(b):
            return 0 <= b < len(msg.buttons) and msg.buttons[b] == 1

        def rising(b):
            now, was = pressed(b), self.prev.get(b, False)
            self.prev[b] = now
            return now and not was

        start, pause = rising(self.start_button), rising(self.pause_button)  # 両方とも毎回評価して状態を更新
        if start:
            if self.start_requires_enable and not pressed(self.enable_button):
                self.node.log(f"{YELLOW}Start は LB を握りながら押す (誤爆防止: start_requires_enable){RESET}")
            else:
                self.push(self.resume_key)
        if pause:
            self.push("p")

    def push(self, ch):
        t = time.monotonic()
        if t - self.last_push < self.debounce:
            return
        self.last_push = t
        self.queue.append(ch)

    def get(self):
        """1 文字 (joy 由来を優先) か None。既存コードの kb.get() と同じ契約。"""
        if self.queue:
            return self.queue.popleft()
        return self.kb.get() if self.kb else None


class WaypointRunner(Node):
    def __init__(self, wps, resume_key, auto_retry=1, retry_wait=5.0):
        super().__init__("waypoint_runner")
        self.wps, self.resume_key = wps, resume_key
        self.auto_retry, self.retry_wait = auto_retry, retry_wait  # claude: 区間失敗の自動再送 (2026-10-16)
        self.client = ActionClient(self, NavigateThroughPoses, "navigate_through_poses")
        self.remaining = 0  # 走行中区間の未到達点数 (feedback の number_of_poses_remaining)
        self.goal_handle = None  # 走行中ゴールのハンドル (Ctrl-C 時のキャンセル用)
        # claude: 2026-10-16 現在状態を外に出す (端末ログは auto_run の他ノード出力に流されて追えないため)。
        #   /waypoint_runner/status (String) と /waypoint_runner/marker (車体上 1.6 m のテキスト、nav2.rviz に表示あり)。
        #   transient_local なので後から購読した auto_run_check / RViz にも最新が届く。
        tl = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                        history=HistoryPolicy.KEEP_LAST, depth=1)
        self.status_pub = self.create_publisher(String, "~/status", tl)
        self.marker_pub = self.create_publisher(Marker, "~/marker", tl)

    def log(self, msg):
        # claude: rclpy のロガーは cbreak 中に行頭が崩れるので print で直接出す
        print(msg, flush=True)

    def set_status(self, text, rgb=(1.0, 0.9, 0.0)):
        """現在状態を /waypoint_runner/status と RViz マーカーに出す (ログは呼び出し側で別に出す)。"""  # claude: 2026-10-16
        self.status_pub.publish(String(data=text))
        m = Marker()
        m.header.frame_id = "base_link"
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns, m.id = "waypoint_runner", 0
        m.type, m.action = Marker.TEXT_VIEW_FACING, Marker.ADD
        m.pose.position.z = 1.6
        m.pose.orientation.w = 1.0
        m.scale.z = 0.35
        m.color.r, m.color.g, m.color.b, m.color.a = rgb[0], rgb[1], rgb[2], 1.0
        m.text = text
        self.marker_pub.publish(m)

    def make_goal(self, idx):
        goal = NavigateThroughPoses.Goal()
        now = self.get_clock().now().to_msg()
        for i in idx:
            _, x, y, qz, qw, _ = self.wps[i]
            p = PoseStamped()
            p.header.frame_id = "map"
            p.header.stamp = now
            p.pose.position.x, p.pose.position.y = x, y
            p.pose.orientation.z, p.pose.orientation.w = qz, qw
            goal.poses.append(p)
        return goal

    def spin_until(self, future, kb, on_key=None):
        """future 完了まで spin。走行中のキーは on_key に渡す。"""
        while rclpy.ok() and not future.done():
            rclpy.spin_once(self, timeout_sec=0.05)
            k = kb.get()
            if k is not None and on_key is not None:
                on_key(k)
        return future.result()

    def wait_key(self, kb, prompt):
        """再開キー → True / q → False。待機中も spin して DDS を回しておく。"""
        self.log(f"{YELLOW}{BOLD}■ {prompt}{RESET}  [Start / {self.resume_key}] 再開 / [q] 終了")
        self.set_status(f"■ 待機: {prompt}  → Start で再開")
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            k = kb.get()
            if k == self.resume_key:
                return True
            if k == "q":
                return False
        return False

    def wait_seconds(self, kb, sec, prompt):
        """sec 秒 spin しながら待つ。q が押されたら False (中断)。"""
        self.log(f"{YELLOW}… {prompt} ({sec:.0f} s 待って自動再送)  [q] 終了{RESET}")
        self.set_status(f"… {prompt} ({sec:.0f} s 後に自動再送)", (1.0, 0.5, 0.0))
        end = self.get_clock().now().nanoseconds + int(sec * 1e9)
        while rclpy.ok() and self.get_clock().now().nanoseconds < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if kb.get() == "q":
                return False
        return rclpy.ok()

    def run_segment(self, idx, kb):
        """idx の点列を 1 本の NavigateThroughPoses で走る。
        戻り値: ("ok"|"paused"|"failed", 未到達の idx 末尾)。"""
        self.remaining = len(idx)
        head, tail = self.wps[idx[0]][0], self.wps[idx[-1]][0]

        def on_fb(fb):
            if fb.feedback.number_of_poses_remaining != self.remaining:  # claude: 残り点数が変わったときだけ状態更新
                self.remaining = fb.feedback.number_of_poses_remaining
                self.set_status(f"▶ 走行中 {head} → {tail}  残り {self.remaining} 点  (LB で介入 / Back で一時停止)",
                                (0.3, 1.0, 0.3))

        send = self.client.send_goal_async(self.make_goal(idx), feedback_callback=on_fb)
        handle = self.spin_until(send, kb)
        if handle is None or not handle.accepted:
            self.log(f"{RED}✘ ゴールが拒否された (bt_navigator が active か確認){RESET}")
            return "failed", idx
        self.goal_handle = handle
        paused = []

        def on_key(k):
            if k == "p" and not paused:
                paused.append(True)
                self.log(f"{YELLOW}… 一時停止: ゴールをキャンセル中{RESET}")
                handle.cancel_goal_async()

        res = self.spin_until(handle.get_result_async(), kb, on_key)
        self.goal_handle = None
        left = idx[len(idx) - max(1, self.remaining):]  # 最低でも区間の終点 (停止点) は残す
        if res is None:
            return "failed", left
        if res.status == GoalStatus.STATUS_SUCCEEDED:
            return "ok", []
        if res.status == GoalStatus.STATUS_CANCELED and paused:
            return "paused", left
        r = res.result
        self.log(f"{RED}✘ 区間失敗 (status {res.status}, error_code {r.error_code}: {r.error_msg}){RESET}")
        self.set_status(f"✘ 区間失敗 error_code {r.error_code}", (1.0, 0.2, 0.2))
        return "failed", left

    def cancel_active(self):
        """走行中ゴールがあればキャンセルし、受理されるまで最大 2 s 待つ。"""
        if self.goal_handle is None:
            return
        fut = self.goal_handle.cancel_goal_async()
        rclpy.spin_until_future_complete(self, fut, timeout_sec=2.0)
        self.log("ゴールをキャンセルした" if fut.done() else f"{RED}✘ キャンセル応答なし — RViz で停止を確認{RESET}")

    def run(self, segs, kb):
        self.log("navigate_through_poses サーバ待ち…")
        self.set_status("navigate_through_poses サーバ待ち (Nav2 起動中)", (0.7, 0.7, 0.7))
        waited = 0
        while rclpy.ok() and not self.client.wait_for_server(timeout_sec=1.0):
            waited += 1
            if waited % 10 == 0:  # claude: 2026-10-16 auto_run では Nav2 より先に起動するので 10 s に 1 回だけ (ログを流さない)
                self.log(f"  … まだ ({waited} s。nav の起動 / bt_navigator の activate を待っている)")
        n = len(self.wps)
        for s, (i0, i1) in enumerate(segs):
            idx = list(range(i0, i1 + 1))
            name = self.wps[i1][0]
            last = i1 == n - 1
            if not self.wait_key(kb, f"区間 {s + 1}/{len(segs)}: {self.wps[i0][0]} → {name} "
                                     f"({len(idx)} 点) を開始する?"):
                return
            retries = 0  # claude: この区間で使った自動再送回数 (2026-10-16)
            while idx:
                self.log(f"{GREEN}▶ 走行中 {self.wps[idx[0]][0]} → {name}{RESET}  [Back / p] 一時停止")
                self.set_status(f"▶ 走行中 {self.wps[idx[0]][0]} → {name}  残り {len(idx)} 点  (LB で介入 / Back で一時停止)",
                                (0.3, 1.0, 0.3))
                st, idx = self.run_segment(idx, kb)
                if st == "ok":
                    break
                head = self.wps[idx[0]][0]
                if st == "failed" and retries < self.auto_retry:
                    retries += 1
                    if not self.wait_seconds(kb, self.retry_wait,
                                             f"区間失敗 → 自動再送 {retries}/{self.auto_retry}: {head} から"):
                        return
                    continue
                msg = "一時停止中" if st == "paused" else "失敗 (自動再送も失敗)" if retries else "失敗"
                if not self.wait_key(kb, f"{msg} — 再開すると {head} から続ける"):
                    return
            if not rclpy.ok():
                return
            self.log(f"{GREEN}{BOLD}✔ {name} に到着{'' if last else ' (停止点)'}{RESET}")
            self.set_status(f"✔ {name} に到着{'' if last else ' (停止点)'}", (0.3, 1.0, 0.3))
        self.log(f"{GREEN}{BOLD}✔ コース完走{RESET}")
        self.set_status("✔ コース完走", (0.3, 1.0, 0.3))


def main():
    # claude: allow_abbrev=False — launch が付ける --ros-args / -r が長い option の前方一致に吸われないように
    ap = argparse.ArgumentParser(description="停止点つき waypoint 走行 (stop: true の点で止まりキー / joy ボタンで再開)",
                                 allow_abbrev=False)
    ap.add_argument("course", help="waypoint YAML (waypoint_editor / Nav2 パネル形式)")
    ap.add_argument("--resume-key", default="g", help="再開キー (1 文字、既定 g)")
    ap.add_argument("--start", type=int, default=0, help="waypointN から始める (index、既定 0)")
    ap.add_argument("--auto-retry", type=int, default=1, help="区間失敗時の自動再送回数 (既定 1、0 で無効)")
    ap.add_argument("--retry-wait", type=float, default=5.0, help="自動再送までの待ち秒数 (既定 5)")
    args, ros_args = ap.parse_known_args()
    if len(args.resume_key) != 1 or args.resume_key in "qp":
        sys.exit("--resume-key は q / p 以外の 1 文字")
    # claude: 2026-10-16 端末なし (launch 配下) でも起動できる — その場合の入力は joy ボタンだけ (class Input)

    wps = load_course(args.course)
    if not 0 <= args.start < len(wps):
        sys.exit(f"--start {args.start} が範囲外 (0..{len(wps) - 1})")
    segs = split_segments(len(wps), [w[5] for w in wps], args.start)
    total = sum(math.dist(a[1:3], b[1:3]) for a, b in zip(wps[args.start:], wps[args.start + 1:]))
    print(f"コース: {args.course} — {len(wps)} 点 (waypoint{args.start} から), {total:.1f} m, "
          f"停止点 {[w[0] for w in wps[args.start:] if w[5]] or 'なし'}", flush=True)  # launch 配下は stdout がブロックバッファ

    # claude: 既定の SIGINT ハンドラは context を即 shutdown してキャンセルを送れなくなるので外し、
    #   KeyboardInterrupt として受けてから走行中ゴールをキャンセルする (走りっぱなし防止)
    rclpy.init(args=ros_args, signal_handler_options=SignalHandlerOptions.NO)
    node = WaypointRunner(wps, args.resume_key, max(0, args.auto_retry), max(0.0, args.retry_wait))
    try:
        with Input(node, args.resume_key) as kb:  # claude: 2026-10-16 キーボード + joy ボタン
            node.log(f"入力: {kb.describe()}")
            node.run(segs, kb)
    except KeyboardInterrupt:
        print(f"\n{YELLOW}Ctrl-C: 走行中のゴールをキャンセルして終了{RESET}")
        node.cancel_active()
        node.set_status("■ waypoint_runner 終了 (Ctrl-C)", (0.7, 0.7, 0.7))
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
