#!/usr/bin/env python3
# claude: 2026-10-15 新設 — 自律走行の起動を 1 本にまとめる (bringup → nav → RViz → bag record)。
#   10-14 の走行で「当日の起動引数が分からない」「bag record の開始が遅れる」「帯の引数を間違える」が
#   起きたので、人が打つのは map_dir だけにする。
# claude: 2026-10-16 joy_teleop と waypoint_runner も同居させ、起動後は端末を一切触らない運用にした
#   (設計: docs/features/2026-10-16_auto_run_joy_mux.md)。速度指令は bringup 常駐の twist_mux が
#   joy (/joy_vel 優先) と Nav2 (/cmd_vel) を調停する。
#
#   当日の流れ (rerobot_env 内):
#     待機エリアで起動:
#     ros2 launch rerobot_bringup auto_run.launch.py map_dir:=/workspace/maps/2d/glim/2026-10-09_1643_5goukan_chal \
#       2>&1 | tee /workspace/log/auto_$(TZ=Asia/Tokyo date +%F_%H%M).log
#     → LB を握って joy でスタート付近へ (Nav2 はゴールを持たないので邪魔しない)
#     → 蓋を開け RViz の 2D Pose Estimate で初期位置を目視で置く (RViz は閉じても残してもよい。他ノードは落ちない)
#        (AMCL は起動時から course.yaml の waypoint0 を仮の初期姿勢として持つ — 下記 initial_pose)
#     → joy でスタート列へ。AMCL はスキャンで追従し続ける
#     → スタート位置で Start → waypoint_runner が区間 1 を送る。LB を離していれば 0.5 s 以内に Nav2 の指令が通る
#        停止点・一時停止からの再開も Start。Back = 一時停止 (ゴールをキャンセル)。走行中に LB を握ればいつでも止められる
#     Ctrl-C 1 回で全部止まる (bag record も SIGINT で正常終了し mcap が閉じる。runner は走行中ゴールをキャンセル)。
#
# claude: 2026-11-07 bringup / waypoint_runner を別端末で起動する使い方を追加 (bringup:=false runner:=false)。
#   1 本にまとめると bringup (EPOS4・センサ) と runner (区間の進み・失敗) のログが Nav2 のログに流されて
#   リアルタイムに追えないため。端末の分け方:
#     端末 1: ros2 launch rerobot_bringup rerobot_bringup.launch.py rfans_scan_all:=true
#             (bringup_check の表が出てから端末 2 へ。先に上げると nav が TF 待ちで activate に失敗しうる)
#     端末 2: ros2 launch rerobot_bringup auto_run.launch.py bringup:=false runner:=false \
#               map_dir:=/workspace/maps/2d/glim/2026-10-09_1643_5goukan_chal_fixed2_filtered loc_band:=0.3-100.0_r180 \
#               2>&1 | tee /workspace/log/auto_$(TZ=Asia/Tokyo date +%F_%H%M).log
#             (0.025 m 地図など loc_band 規約に乗らない地図は loc_map_yaml:=<yaml> で直接指定)
#     端末 3: ros2 run rerobot_bringup waypoint_runner.py <map_dir>/waypoints/course.yaml [--start N]
#   bringup:=false のときの時間割: nav 0 s / rviz 15 s / record 20 s / 全部品チェック 35 s
#
# claude: 2026-11-07 初期姿勢: AMCL は初期姿勢を知るまで map->odom を出さず、人が RViz で置くのが遅れると
#   global_costmap が TF 待ちで activate に失敗する。そこで起動時から AMCL に仮の初期姿勢 (set_initial_pose) を
#   持たせ、最初のスキャンから TF が出るようにした。正しい姿勢は従来どおり RViz の 2D Pose Estimate で上書きする。
#   initial_pose:= 空 (既定) = course.yaml の waypoint0 / "x,y,yaw_deg" = 指定値 / none = 与えない (旧挙動)。
#   (旧仕様の「60 s 後に initialpose.py で /initialpose を送る」は、RViz で置いた姿勢を上書きしうるので廃止)
#
#   時間割 (bringup の EPOS enable 5 s・bringup_check 15 s の後に nav。nav + RViz 同時起動は CPU 負荷で
#   CAN 途絶の実績があるので RViz は 15 s ずらす。AMCL は最初のスキャン後 ≈13 s 尤度場を作るので initialpose は 60 s):
#     0 s  rerobot_bringup.launch.py rfans_scan_all:=true   (帯は launch 既定: /rfans/scan 0.3〜2.0, /rfans/scan_all 0.3〜100)
#     0 s  joy_teleop.launch.py   (joy_node + teleop_twist_joy → /joy_vel + teach_marker)   joy:=false で省略
#    20 s  nav_amcl_scanall.launch.py map_dir loc_map_yaml=<map_dir>/raw_traj/<loc_band>/map.yaml
#    30 s  waypoint_runner.py <map_dir>/waypoints/course.yaml  (course.yaml が無ければ起動しない。サーバ待ちは自前)
#    35 s  RViz (nav2.rviz)   rviz:=false で省略
#    40 s  ros2 bag record -s mcap -o <bag_dir>/<JST 日時>_<tag>   record:=false で省略
#    (60 s の initialpose.py 送信は 2026-11-07 廃止 → AMCL の set_initial_pose に置き換え)
#
#   ⚠ 新規ファイルなので初回は rerobot_bringup の再ビルド (scripts/build.sh main) が要る
#     (install(DIRECTORY launch) はファイル単位の symlink で、後から増えたファイルは拾わない)。
import datetime
import math
import os
import re
import zoneinfo

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, GroupAction, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml

# 記録トピック (1 か所で管理)。packets は回転ディップ解析と点群の作り直し用。
#   claude: 2026-10-16 /joy /joy_vel /cmd_vel を追加 — 「誰の指令が /robot_speed_cmd に通ったか」「いつボタンを押したか」を後から追える
RECORD_TOPICS = [
    "/rfans_driver/rfans_packets", "/rfans_driver/rfans_points", "/urg_front/scan", "/rfans/scan", "/rfans/scan_all",
    "/imu/data", "/odom", "/odometry/filtered", "/joint_states", "/tf", "/tf_static", "/diagnostics", "/robot_speed_cmd",
    "/joy", "/joy_vel", "/cmd_vel", "/cmd_vel_nav",  # claude: 2026-10-30 /cmd_vel_nav = velocity_smoother 前 (平滑化の前後比較用)
    "/behavior_tree_log", "/rosout", "/plan", "/local_plan", "/lookahead_point", "/amcl_pose", "/particle_cloud",
    "/lookahead_collision_arc", "/is_rotating_to_heading",  # claude: 2026-11-04 RPP の衝突掃引の当たり姿勢と旋回モード (後端振り出しラッチの直接証拠用)
    "/initialpose", "/goal_pose", "/map", "/map_loc", "/keepout_filter_mask",
    "/global_costmap/costmap", "/global_costmap/costmap_updates",
    "/local_costmap/costmap", "/local_costmap/costmap_updates", "/local_costmap/published_footprint", "/teach_marker",
]
T_NAV, T_RUNNER, T_RVIZ, T_RECORD, T_CHECK = 20.0, 30.0, 35.0, 40.0, 55.0


def first_waypoint_pose(course_yaml):
    """course.yaml の先頭 waypoint を (x, y, yaw_rad) で返す。無ければ None。orientation は [w, x, y, z]。"""
    try:
        wps = (yaml.safe_load(open(course_yaml)) or {}).get("waypoints") or {}
        wp = next(iter(wps.values()))
        w, _, _, z = wp["orientation"]
        return float(wp["pose"][0]), float(wp["pose"][1]), 2.0 * math.atan2(float(z), float(w))
    except (OSError, StopIteration, KeyError, TypeError, ValueError, yaml.YAMLError):
        return None


def setup(context):
    pkg_share = get_package_share_directory("rerobot_bringup")
    launch_dir = os.path.join(pkg_share, "launch")
    cfg = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    flag = lambda name: cfg(name).strip().lower() in ("true", "1", "yes")  # noqa: E731

    map_dir = cfg("map_dir").rstrip("/")
    if not map_dir:
        raise RuntimeError("map_dir:=<地図ディレクトリ> は必須 (例 /workspace/maps/2d/glim/2026-10-09_1643_5goukan_chal)")
    # claude: 2026-11-07 loc_map_yaml 直指定 (0.025 m 版 map_r0025.yaml など loc_band 規約に乗らない地図用)
    loc_map_yaml = cfg("loc_map_yaml").strip() or os.path.join(map_dir, "raw_traj", cfg("loc_band"), "map.yaml")
    nav_map_yaml = os.path.join(map_dir, "nav2", "map.yaml")
    course_yaml = os.path.join(map_dir, "waypoints", "course.yaml")
    missing = [p for p in (nav_map_yaml, loc_map_yaml) if not os.path.isfile(p)]
    if missing:
        raise RuntimeError("見つからない: " + ", ".join(missing))

    # bag 名: <JST 日時>_<tag>。tag 既定は map_dir 末尾から先頭の日時を外したもの + _auto (例 5goukan_chal_auto)
    tag = cfg("tag") or re.sub(r"^\d{4}-\d{2}-\d{2}_\d{4}_", "", os.path.basename(map_dir)) + "_auto"
    stamp = datetime.datetime.now(zoneinfo.ZoneInfo("Asia/Tokyo")).strftime("%Y-%m-%d_%H%M")
    bag_path = os.path.join(cfg("bag_dir"), f"{stamp}_{tag}")

    def include(rel, launch_arguments):
        # scoped=True: 子に渡した引数が後続 include に漏れないように (nav_*.launch.py と同じ約束)
        return GroupAction([IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, rel)),
            launch_arguments=launch_arguments.items(),
        )], scoped=True)

    have_course = os.path.isfile(course_yaml)
    use_bringup, use_runner = flag("bringup"), flag("runner")
    # claude: 2026-11-07 bringup を外で上げるときは bringup 待ちの 20 s が要らないので全体を前に詰める (間隔は維持)
    shift = 0.0 if use_bringup else T_NAV
    t_nav, t_runner, t_rviz, t_record, t_check = (t - shift for t in (T_NAV, T_RUNNER, T_RVIZ, T_RECORD, T_CHECK))

    # claude: 2026-11-07 AMCL に起動時から持たせる仮の初期姿勢 (set_initial_pose)。空 = waypoint0 / "x,y,yaw_deg" / none
    init_arg = cfg("initial_pose").strip()
    init_pose, init_src = None, "なし (RViz で置くまで map->odom が出ない)"
    if init_arg.lower() == "none":
        pass
    elif init_arg:
        try:
            x, y, yaw_deg = (float(v) for v in init_arg.split(","))
        except ValueError:
            raise RuntimeError('initial_pose は "x,y,yaw_deg" (例 "3.60,-0.07,-8") か none')
        init_pose, init_src = (x, y, math.radians(yaw_deg)), "指定値"
    elif have_course:
        init_pose = first_waypoint_pose(course_yaml)
        init_src = "course.yaml の先頭 waypoint" if init_pose else "なし (course.yaml の先頭 waypoint が読めない)"
    init_str = ",".join(f"{v:.4f}" for v in init_pose) if init_pose else ""
    init_desc = (f"x={init_pose[0]:.2f} y={init_pose[1]:.2f} yaw={math.degrees(init_pose[2]):.1f}° ({init_src})"
                 if init_pose else init_src)
    joy_yaml = os.path.join(pkg_share, "config", "joy_teleop.yaml")
    actions = [
        LogInfo(msg=(
            "\n===== auto_run =====\n"
            f"  map_dir      : {map_dir}\n"
            f"  nav map      : {nav_map_yaml}\n"
            f"  loc map      : {loc_map_yaml}\n"
            f"  course       : {course_yaml if have_course else '(無い → waypoint_runner は起動しない)'}\n"
            f"  bringup      : {'この launch で起動' if use_bringup else '外部 (bringup:=false — 別端末で起動済みの前提)'}\n"
            f"  runner       : {'この launch で起動' if use_runner and have_course else '起動しない (端末から ros2 run)'}\n"
            f"  仮の初期姿勢 : {init_desc} → 正しい姿勢は RViz の 2D Pose Estimate で\n"
            f"  bag          : {bag_path if flag('record') else '(record:=false)'}\n"
            f"  joy          : {'on' if flag('joy') else 'off'}  rviz: {'on' if flag('rviz') else 'off'}\n"
            f"  時間割       : nav {t_nav:.0f} s{f' / runner {t_runner:.0f} s' if use_runner and have_course else ''}"
            f" / rviz {t_rviz:.0f} s / record {t_record:.0f} s"
            f" / 全部品チェック {t_check:.0f} s\n"
            "  操作 (パッド): LB 押下中 = joy 走行 (Nav2 より優先)  /  Start = 区間開始・再開  /  Back = 一時停止\n"
            f"  {t_check:.0f} s 後に全部品の起動結果の表が出ます。再確認: ros2 run rerobot_bringup auto_run_check.py\n"
            "  runner の状態: RViz の車体上テキスト / ros2 topic echo /waypoint_runner/status\n"
            "  端末から動かす場合: ros2 run rerobot_bringup waypoint_runner.py <course.yaml> (-it)\n"
            "====================\n")),
    ]
    if use_bringup:
        actions.append(include("rerobot_bringup.launch.py", {"rfans_scan_all": "true"}))
    nav = include(os.path.join("nav", "nav_amcl_scanall.launch.py"), {
        "map_dir": map_dir, "loc_map_yaml": loc_map_yaml, "use_rviz": "false", "initial_pose": init_str,
    })
    actions.append(TimerAction(period=t_nav, actions=[nav]) if t_nav > 0 else nav)
    # claude: 2026-10-16 ゲームパッド (joy_node + teleop_twist_joy → /joy_vel + teach_marker)。bringup の twist_mux が受ける
    if flag("joy"):
        actions.append(include("joy_teleop.launch.py", {}))
    # claude: 2026-10-16 waypoint_runner を launch 配下で起動 (端末なし → 入力は joy ボタンだけ)。
    #   navigate_through_poses サーバ待ちは runner 自身がループするので nav の後なら時刻は厳密でなくてよい。
    #   joy_teleop.yaml の waypoint_runner 節 (ボタン割り当て) をパラメータで渡す。
    if have_course and use_runner:
        actions.append(TimerAction(period=t_runner, actions=[Node(
            package="rerobot_bringup", executable="waypoint_runner.py", name="waypoint_runner",
            arguments=[course_yaml], parameters=[joy_yaml], output="screen",
        )]))
    if flag("rviz"):
        actions.append(TimerAction(period=t_rviz, actions=[include(os.path.join("parts", "rviz_nav.launch.py"), {
            "rviz_config": os.path.join(pkg_share, "rviz", "nav2.rviz"),
        })]))
    if flag("record"):
        actions.append(TimerAction(period=t_record, actions=[ExecuteProcess(
            cmd=["ros2", "bag", "record", "-s", "mcap", "-o", bag_path] + RECORD_TOPICS,
            name="bag_record", output="screen",
        )]))
    if not have_course:
        actions.append(LogInfo(msg=f"⚠ waypoint ファイルが無い: {course_yaml} → waypoint_runner は起動しない "
                                   "(端末から別パスで ros2 run rerobot_bringup waypoint_runner.py <yaml>)"))
    # claude: 2026-10-16 全部品の起動結果を 1 枚の表で (bringup_check は 15 s で流れてしまうので、全部出揃った後にもう一度まとめて)
    actions.append(TimerAction(period=t_check, actions=[
        LogInfo(msg="===== auto_run: 全部品の起動チェック ====="),
        Node(package="rerobot_bringup", executable="auto_run_check.py", name="auto_run_check", output="screen",
             emulate_tty=True,
             parameters=[{"joy": flag("joy"), "rviz": flag("rviz"), "record": flag("record"),
                         "runner": have_course and use_runner}]),
    ]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("map_dir", default_value="",
                              description="地図一式の親ディレクトリ (nav2/, keep_out/, raw_traj/<loc_band>/, waypoints/)。必須"),
        DeclareLaunchArgument("loc_band", default_value="0.3-100.0_r100",
                              description="AMCL 用地図 raw_traj/<loc_band>/map.yaml。bringup の scan_all 帯 (既定 0.3〜100) と揃える"),
        DeclareLaunchArgument("loc_map_yaml", default_value="",
                              description="AMCL 用地図 yaml の直接指定。空なら raw_traj/<loc_band>/map.yaml"),
        DeclareLaunchArgument("bringup", default_value="true",
                              description="rerobot_bringup.launch.py をこの launch で起動するか。false = 別端末で起動済み"),
        DeclareLaunchArgument("runner", default_value="true",
                              description="waypoint_runner をこの launch で起動するか。false = 端末から ros2 run"),
        DeclareLaunchArgument("record", default_value="true", description="bag を記録するか"),
        DeclareLaunchArgument("bag_dir", default_value="/workspace/bags/raw", description="bag の保存先"),
        DeclareLaunchArgument("tag", default_value="",
                              description="bag 名の後半。空なら map_dir 名から日時を外して _auto を付ける"),
        DeclareLaunchArgument("rviz", default_value="true", description="RViz (nav2.rviz) を 35 s 後に起動するか"),
        DeclareLaunchArgument("joy", default_value="true",
                              description="ゲームパッド (joy_teleop.launch.py) を起動するか。false なら waypoint_runner は端末からのみ"),
        DeclareLaunchArgument("initial_pose", default_value="",
                              description='AMCL の仮の初期姿勢。空 = course.yaml の先頭 waypoint / "x,y,yaw_deg" / none'),
        OpaqueFunction(function=setup),
    ])
