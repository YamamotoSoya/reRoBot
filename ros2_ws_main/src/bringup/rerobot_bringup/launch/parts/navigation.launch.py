# claude: 2026-10-08 部品 — Nav2 サーバ群 (走行に必要な最小セット)。nav2.launch.py から切り出し。
#   controller_server / planner_server / behavior_server / bt_navigator + lifecycle_manager_navigation
#
# 前提 (自己位置推定の方式に依らない約束): TF map->odom が誰か 1 つから出ていること、/map が出ていること。
#
# 速度司令の配線: Nav2 既定の /cmd_vel を本機の /robot_speed_cmd (素の Twist) へリマップする。
# Twist 化は params 側の enable_stamped_cmd_vel: false で行う。
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

    cmd_vel_remap = ("/cmd_vel", "/robot_speed_cmd")

    controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
        remappings=[cmd_vel_remap],  # /cmd_vel -> /robot_speed_cmd
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
        remappings=[cmd_vel_remap],  # 復帰行動の速度司令も /robot_speed_cmd へ
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
