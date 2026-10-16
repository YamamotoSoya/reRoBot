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
#        (initial_pose:="x,y,yaw_deg" は再起動時の保険。60 s 後に自動送信)
#     → joy でスタート列へ。AMCL はスキャンで追従し続ける
#     → スタート位置で Start → waypoint_runner が区間 1 を送る。LB を離していれば 0.5 s 以内に Nav2 の指令が通る
#        停止点・一時停止からの再開も Start。Back = 一時停止 (ゴールをキャンセル)。走行中に LB を握ればいつでも止められる
#     Ctrl-C 1 回で全部止まる (bag record も SIGINT で正常終了し mcap が閉じる。runner は走行中ゴールをキャンセル)。
#
#   時間割 (bringup の EPOS enable 5 s・bringup_check 15 s の後に nav。nav + RViz 同時起動は CPU 負荷で
#   CAN 途絶の実績があるので RViz は 15 s ずらす。AMCL は最初のスキャン後 ≈13 s 尤度場を作るので initialpose は 60 s):
#     0 s  rerobot_bringup.launch.py rfans_scan_all:=true   (帯は launch 既定: /rfans/scan 0.3〜2.0, /rfans/scan_all 0.3〜100)
#     0 s  joy_teleop.launch.py   (joy_node + teleop_twist_joy → /joy_vel + teach_marker)   joy:=false で省略
#    20 s  nav_amcl_scanall.launch.py map_dir loc_map_yaml=<map_dir>/raw_traj/<loc_band>/map.yaml
#    30 s  waypoint_runner.py <map_dir>/waypoints/course.yaml  (course.yaml が無ければ起動しない。サーバ待ちは自前)
#    35 s  RViz (nav2.rviz)   rviz:=false で省略
#    40 s  ros2 bag record -s mcap -o <bag_dir>/<JST 日時>_<tag>   record:=false で省略
#    60 s  initialpose.py x y yaw   initial_pose:="" (既定) なら送らない
#
#   ⚠ 新規ファイルなので初回は rerobot_bringup の再ビルド (scripts/build.sh main) が要る
#     (install(DIRECTORY launch) はファイル単位の symlink で、後から増えたファイルは拾わない)。
import datetime
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

# 記録トピック (1 か所で管理)。packets は回転ディップ解析と点群の作り直し用。
#   claude: 2026-10-16 /joy /joy_vel /cmd_vel を追加 — 「誰の指令が /robot_speed_cmd に通ったか」「いつボタンを押したか」を後から追える
RECORD_TOPICS = [
    "/rfans_driver/rfans_packets", "/rfans_driver/rfans_points", "/urg_front/scan", "/rfans/scan", "/rfans/scan_all",
    "/imu/data", "/odom", "/odometry/filtered", "/joint_states", "/tf", "/tf_static", "/diagnostics", "/robot_speed_cmd",
    "/joy", "/joy_vel", "/cmd_vel",
    "/behavior_tree_log", "/rosout", "/plan", "/local_plan", "/lookahead_point", "/amcl_pose", "/particle_cloud",
    "/initialpose", "/goal_pose", "/map", "/map_loc", "/keepout_filter_mask",
    "/global_costmap/costmap", "/global_costmap/costmap_updates",
    "/local_costmap/costmap", "/local_costmap/costmap_updates", "/local_costmap/published_footprint", "/teach_marker",
]
T_NAV, T_RUNNER, T_RVIZ, T_RECORD, T_CHECK, T_INITPOSE = 20.0, 30.0, 35.0, 40.0, 55.0, 60.0


def setup(context):
    pkg_share = get_package_share_directory("rerobot_bringup")
    launch_dir = os.path.join(pkg_share, "launch")
    cfg = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    flag = lambda name: cfg(name).strip().lower() in ("true", "1", "yes")  # noqa: E731

    map_dir = cfg("map_dir").rstrip("/")
    if not map_dir:
        raise RuntimeError("map_dir:=<地図ディレクトリ> は必須 (例 /workspace/maps/2d/glim/2026-10-09_1643_5goukan_chal)")
    loc_map_yaml = os.path.join(map_dir, "raw_traj", cfg("loc_band"), "map.yaml")
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
    joy_yaml = os.path.join(pkg_share, "config", "joy_teleop.yaml")
    actions = [
        LogInfo(msg=(
            "\n===== auto_run =====\n"
            f"  map_dir      : {map_dir}\n"
            f"  nav map      : {nav_map_yaml}\n"
            f"  loc map      : {loc_map_yaml}\n"
            f"  course       : {course_yaml if have_course else '(無い → waypoint_runner は起動しない)'}\n"
            f"  bag          : {bag_path if flag('record') else '(record:=false)'}\n"
            f"  joy          : {'on' if flag('joy') else 'off'}  rviz: {'on' if flag('rviz') else 'off'}"
            f"  initial_pose: {cfg('initial_pose') or '(RViz で手動)'}\n"
            f"  時間割       : nav {T_NAV:.0f} s / runner {T_RUNNER:.0f} s / rviz {T_RVIZ:.0f} s / record {T_RECORD:.0f} s"
            f" / 全部品チェック {T_CHECK:.0f} s / initialpose {T_INITPOSE:.0f} s\n"
            "  操作 (パッド): LB 押下中 = joy 走行 (Nav2 より優先)  /  Start = 区間開始・再開  /  Back = 一時停止\n"
            f"  {T_CHECK:.0f} s 後に全部品の起動結果の表が出ます。再確認: ros2 run rerobot_bringup auto_run_check.py\n"
            "  runner の状態: RViz の車体上テキスト / ros2 topic echo /waypoint_runner/status\n"
            "  端末から動かす場合: ros2 run rerobot_bringup waypoint_runner.py <course.yaml> (-it)\n"
            "====================\n")),
        include("rerobot_bringup.launch.py", {"rfans_scan_all": "true"}),
        TimerAction(period=T_NAV, actions=[include(os.path.join("nav", "nav_amcl_scanall.launch.py"), {
            "map_dir": map_dir, "loc_map_yaml": loc_map_yaml, "use_rviz": "false",
        })]),
    ]
    # claude: 2026-10-16 ゲームパッド (joy_node + teleop_twist_joy → /joy_vel + teach_marker)。bringup の twist_mux が受ける
    if flag("joy"):
        actions.append(include("joy_teleop.launch.py", {}))
    # claude: 2026-10-16 waypoint_runner を launch 配下で起動 (端末なし → 入力は joy ボタンだけ)。
    #   navigate_through_poses サーバ待ちは runner 自身がループするので nav の後なら時刻は厳密でなくてよい。
    #   joy_teleop.yaml の waypoint_runner 節 (ボタン割り当て) をパラメータで渡す。
    if have_course:
        actions.append(TimerAction(period=T_RUNNER, actions=[Node(
            package="rerobot_bringup", executable="waypoint_runner.py", name="waypoint_runner",
            arguments=[course_yaml], parameters=[joy_yaml], output="screen",
        )]))
    if flag("rviz"):
        actions.append(TimerAction(period=T_RVIZ, actions=[include(os.path.join("parts", "rviz_nav.launch.py"), {
            "rviz_config": os.path.join(pkg_share, "rviz", "nav2.rviz"),
        })]))
    if flag("record"):
        actions.append(TimerAction(period=T_RECORD, actions=[ExecuteProcess(
            cmd=["ros2", "bag", "record", "-s", "mcap", "-o", bag_path] + RECORD_TOPICS,
            name="bag_record", output="screen",
        )]))
    if cfg("initial_pose").strip():
        try:
            x, y, yaw = (s.strip() for s in cfg("initial_pose").split(","))
            float(x), float(y), float(yaw)
        except ValueError:
            raise RuntimeError('initial_pose は "x,y,yaw_deg" (例 "3.60,-0.07,-8")')
        actions.append(TimerAction(period=T_INITPOSE, actions=[ExecuteProcess(
            cmd=["ros2", "run", "rerobot_bringup", "initialpose.py", x, y, yaw], name="initialpose", output="screen",
        )]))
    if not have_course:
        actions.append(LogInfo(msg=f"⚠ waypoint ファイルが無い: {course_yaml} → waypoint_runner は起動しない "
                                   "(端末から別パスで ros2 run rerobot_bringup waypoint_runner.py <yaml>)"))
    # claude: 2026-10-16 全部品の起動結果を 1 枚の表で (bringup_check は 15 s で流れてしまうので、全部出揃った後にもう一度まとめて)
    actions.append(TimerAction(period=T_CHECK, actions=[
        LogInfo(msg="===== auto_run: 全部品の起動チェック ====="),
        Node(package="rerobot_bringup", executable="auto_run_check.py", name="auto_run_check", output="screen",
             emulate_tty=True,
             parameters=[{"joy": flag("joy"), "rviz": flag("rviz"), "record": flag("record"), "runner": have_course}]),
    ]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("map_dir", default_value="",
                              description="地図一式の親ディレクトリ (nav2/, keep_out/, raw_traj/<loc_band>/, waypoints/)。必須"),
        DeclareLaunchArgument("loc_band", default_value="0.3-100.0_r100",
                              description="AMCL 用地図 raw_traj/<loc_band>/map.yaml。bringup の scan_all 帯 (既定 0.3〜100) と揃える"),
        DeclareLaunchArgument("record", default_value="true", description="bag を記録するか"),
        DeclareLaunchArgument("bag_dir", default_value="/workspace/bags/raw", description="bag の保存先"),
        DeclareLaunchArgument("tag", default_value="",
                              description="bag 名の後半。空なら map_dir 名から日時を外して _auto を付ける"),
        DeclareLaunchArgument("rviz", default_value="true", description="RViz (nav2.rviz) を 35 s 後に起動するか"),
        DeclareLaunchArgument("joy", default_value="true",
                              description="ゲームパッド (joy_teleop.launch.py) を起動するか。false なら waypoint_runner は端末からのみ"),
        DeclareLaunchArgument("initial_pose", default_value="",
                              description='起動 60 s 後に送る初期姿勢 "x,y,yaw_deg" (map 座標)。空なら RViz で手動'),
        OpaqueFunction(function=setup),
    ])
