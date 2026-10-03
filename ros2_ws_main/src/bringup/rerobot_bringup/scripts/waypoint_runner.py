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
#   区間が失敗 (ABORTED) したときも待機に入る。再開キーで残りの点から再送する。
#
#   使い方 (rerobot_env 内、nav2.launch.py 起動 + AMCL 初期位置合わせ済みで。キー入力に -it 必須):
#     ros2 run rerobot_bringup waypoint_runner.py <course.yaml> [--resume-key g] [--start N]
#     --start N : waypointN から始める (途中で止めて再起動したとき用)
import argparse
import math
import select
import sys
import termios
import tty

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

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


class WaypointRunner(Node):
    def __init__(self, wps, resume_key):
        super().__init__("waypoint_runner")
        self.wps, self.resume_key = wps, resume_key
        self.client = ActionClient(self, NavigateThroughPoses, "navigate_through_poses")
        self.remaining = 0  # 走行中区間の未到達点数 (feedback の number_of_poses_remaining)
        self.goal_handle = None  # 走行中ゴールのハンドル (Ctrl-C 時のキャンセル用)

    def log(self, msg):
        # claude: rclpy のロガーは cbreak 中に行頭が崩れるので print で直接出す
        print(msg, flush=True)

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
        self.log(f"{YELLOW}{BOLD}■ {prompt}{RESET}  [{self.resume_key}] 再開 / [q] 終了")
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            k = kb.get()
            if k == self.resume_key:
                return True
            if k == "q":
                return False
        return False

    def run_segment(self, idx, kb):
        """idx の点列を 1 本の NavigateThroughPoses で走る。
        戻り値: ("ok"|"paused"|"failed", 未到達の idx 末尾)。"""
        self.remaining = len(idx)

        def on_fb(fb):
            self.remaining = fb.feedback.number_of_poses_remaining

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
        while rclpy.ok() and not self.client.wait_for_server(timeout_sec=1.0):
            self.log("  … まだ (nav2.launch.py は起動している?)")
        n = len(self.wps)
        for s, (i0, i1) in enumerate(segs):
            idx = list(range(i0, i1 + 1))
            name = self.wps[i1][0]
            last = i1 == n - 1
            if not self.wait_key(kb, f"区間 {s + 1}/{len(segs)}: {self.wps[i0][0]} → {name} "
                                     f"({len(idx)} 点) を開始する?"):
                return
            while idx:
                self.log(f"{GREEN}▶ 走行中 {self.wps[idx[0]][0]} → {name}{RESET}  [p] 一時停止")
                st, idx = self.run_segment(idx, kb)
                if st == "ok":
                    break
                head = self.wps[idx[0]][0]
                msg = "一時停止中" if st == "paused" else "失敗"
                if not self.wait_key(kb, f"{msg} — 再開すると {head} から続ける"):
                    return
            if not rclpy.ok():
                return
            self.log(f"{GREEN}{BOLD}✔ {name} に到着{'' if last else ' (停止点)'}{RESET}")
        self.log(f"{GREEN}{BOLD}✔ コース完走{RESET}")


def main():
    ap = argparse.ArgumentParser(description="停止点つき waypoint 走行 (stop: true の点で止まりキーで再開)")
    ap.add_argument("course", help="waypoint YAML (waypoint_editor / Nav2 パネル形式)")
    ap.add_argument("--resume-key", default="g", help="再開キー (1 文字、既定 g)")
    ap.add_argument("--start", type=int, default=0, help="waypointN から始める (index、既定 0)")
    args, ros_args = ap.parse_known_args()
    if len(args.resume_key) != 1 or args.resume_key in "qp":
        sys.exit("--resume-key は q / p 以外の 1 文字")
    if not sys.stdin.isatty():
        sys.exit("キー入力に端末が必要 (docker exec -it で起動する)")

    wps = load_course(args.course)
    if not 0 <= args.start < len(wps):
        sys.exit(f"--start {args.start} が範囲外 (0..{len(wps) - 1})")
    segs = split_segments(len(wps), [w[5] for w in wps], args.start)
    total = sum(math.dist(a[1:3], b[1:3]) for a, b in zip(wps[args.start:], wps[args.start + 1:]))
    print(f"コース: {args.course} — {len(wps)} 点 (waypoint{args.start} から), {total:.1f} m, "
          f"停止点 {[w[0] for w in wps[args.start:] if w[5]] or 'なし'}")

    # claude: 既定の SIGINT ハンドラは context を即 shutdown してキャンセルを送れなくなるので外し、
    #   KeyboardInterrupt として受けてから走行中ゴールをキャンセルする (走りっぱなし防止)
    rclpy.init(args=ros_args, signal_handler_options=SignalHandlerOptions.NO)
    node = WaypointRunner(wps, args.resume_key)
    try:
        with Keyboard() as kb:
            node.run(segs, kb)
    except KeyboardInterrupt:
        print(f"\n{YELLOW}Ctrl-C: 走行中のゴールをキャンセルして終了{RESET}")
        node.cancel_active()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
