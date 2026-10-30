# claude: 2026-10-08 部品 — Nav2 サーバ群 (走行に必要な最小セット)。nav2.launch.py から切り出し。
#   controller_server / planner_server / behavior_server / bt_navigator + lifecycle_manager_navigation
#
# 前提 (自己位置推定の方式に依らない約束): TF map->odom が誰か 1 つから出ていること、/map が出ていること。
#
# 速度司令の配線 (2026-10-16 変更): Nav2 は既定の /cmd_vel にそのまま出す。rerobot_bringup.launch.py が
# 常駐させる twist_mux が /cmd_vel (優先 10) と joy の /joy_vel (優先 100) を調停して /robot_speed_cmd へ流す。
# それ以前はここで /cmd_vel → /robot_speed_cmd に remap していた (joy と publisher を取り合っていた)。
# Twist 化は params 側の enable_stamped_cmd_vel: false で行う (twist_mux も use_stamped: false)。
#
# 平滑化 2 段 (claude: 2026-10-30、経路追従のカクつき対策。設計は docs/features/2026-10-30_nav2_smoothing.md):
#   - smoother_server (SimpleSmoother): planner の折れ線経路を BT の SmoothPath で滑らかにする (追従 BT のみ)
#   - velocity_smoother: controller/behavior の速度指令を加速度制限して出し直す
#     配線: controller_server / behavior_server → /cmd_vel_nav → velocity_smoother → /cmd_vel → twist_mux
#     (Nav2 公式 navigation_launch.py と同じ。公式は collision_monitor が最後段だがここでは無し)
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("rerobot_bringup")

    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")

    args = [
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument(
            "params_file", default_value=os.path.join(pkg_share, "config", "nav2_params.yaml"),
            description="Nav2 パラメータファイルへのパス"),
    ]

    controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
        # claude: 2026-10-30 /cmd_vel_nav へ逃がし、/cmd_vel は velocity_smoother だけが出す (twist_mux が /robot_speed_cmd へ中継)
        remappings=[("cmd_vel", "cmd_vel_nav")],
    )
    smoother_server = Node(
        package="nav2_smoother",
        executable="smoother_server",
        name="smoother_server",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
        # claude: 2026-10-30 経路平滑化 (global_costmap/costmap_raw を既定で読む。追従窓でも可)
    )
    planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
    )
    behavior_server = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
        # claude: 2026-10-30 復帰行動 (Wait/BackUp) の速度司令も velocity_smoother を通す
        remappings=[("cmd_vel", "cmd_vel_nav")],
    )
    velocity_smoother = Node(
        package="nav2_velocity_smoother",
        executable="velocity_smoother",
        name="velocity_smoother",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
        # claude: 2026-10-30 入力 cmd_vel → /cmd_vel_nav、出力 cmd_vel_smoothed → /cmd_vel (twist_mux の Nav2 入力)
        remappings=[("cmd_vel", "cmd_vel_nav"), ("cmd_vel_smoothed", "cmd_vel")],
    )
    bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=[params_file, {
            "use_sim_time": use_sim_time,
            # claude: 2026-10-16 NavigateThroughPoses (waypoint_runner) は自作の追従型 BT を使う。
            #   計画失敗時に旧経路を保持し、前方の遅いロボットに stop-and-go で追従する。Spin/BackUp なし。
            #   設計は XML 冒頭コメント。NavigateToPose (RViz の単発ゴール) は Nav2 既定 BT のまま。
            "default_nav_through_poses_bt_xml": os.path.join(
                pkg_share, "behavior_trees", "navigate_through_poses_follow.xml"),
        }],
    )
    lifecycle_manager_navigation = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "autostart": True,  # 起動時に自動で全ノードを activate
            # claude: 2026-10-30 smoother_server / velocity_smoother を追加 (どちらも lifecycle ノード、未登録だと無音)
            "node_names": ["controller_server", "smoother_server", "planner_server", "behavior_server",
                           "bt_navigator", "velocity_smoother"],
        }],
    )

    return LaunchDescription(args + [
        controller_server,
        smoother_server,
        planner_server,
        behavior_server,
        bt_navigator,
        velocity_smoother,
        lifecycle_manager_navigation,
    ])
