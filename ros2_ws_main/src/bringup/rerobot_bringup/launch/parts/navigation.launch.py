# claude: 2026-10-08 部品 — Nav2 サーバ群 (走行に必要な最小セット)。nav2.launch.py から切り出し。
#   controller_server / planner_server / behavior_server / bt_navigator + lifecycle_manager_navigation
#
# 前提 (自己位置推定の方式に依らない約束): TF map->odom が誰か 1 つから出ていること、/map が出ていること。
#
# 速度司令の配線 (2026-10-16 変更): Nav2 は既定の /cmd_vel にそのまま出す。rerobot_bringup.launch.py が
# 常駐させる twist_mux が /cmd_vel (優先 10) と joy の /joy_vel (優先 100) を調停して /robot_speed_cmd へ流す。
# それ以前はここで /cmd_vel → /robot_speed_cmd に remap していた (joy と publisher を取り合っていた)。
# Twist 化は params 側の enable_stamped_cmd_vel: false で行う (twist_mux も use_stamped: false)。
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
        # claude: 2026-10-16 /cmd_vel のまま (twist_mux が /robot_speed_cmd へ中継)
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
        # claude: 2026-10-16 復帰行動の速度司令も /cmd_vel のまま (twist_mux 経由)
    )
    bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
    )
    lifecycle_manager_navigation = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "autostart": True,  # 起動時に自動で全ノードを activate
            "node_names": ["controller_server", "planner_server", "behavior_server", "bt_navigator"],
        }],
    )

    return LaunchDescription(args + [
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        lifecycle_manager_navigation,
    ])
