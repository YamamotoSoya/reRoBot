# claude: 2026-10-08 部品 — 自己位置推定: emcl2 (ryuichiueda/emcl2_ros2、膨張リセット付き MCL)。
#
# 全 localization 部品の共通の約束 (loc_amcl.launch.py 冒頭と同じ):
#   出力 = TF map->odom / 入力 = /initialpose + TF odom->base_link + 地図 (/map か /map_loc)
# emcl2 固有の出力: /mcl_pose, /particlecloud (PoseArray), /alpha (平均尤度)
#
# ⚠ emcl2 は lifecycle ノードではない — 起動直後から initial_pose_{x,y,a} (emcl2.yaml) で動き TF を出す。
#   そのため manager は持たない。RViz の Navigation 2 パネルの "Localization" 表示は
#   lifecycle_manager_localization が居ないので unknown のままになる (走行には影響しない想定、要実機確認)。
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("rerobot_bringup")

    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    scan_topic = LaunchConfiguration("scan_topic")
    loc_map_yaml = LaunchConfiguration("loc_map_yaml")
    map_topic = PythonExpression(["'/map_loc' if '", loc_map_yaml, "' != '' else '/map'"])

    args = [
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument(
            "params_file", default_value=os.path.join(pkg_share, "config", "localization", "emcl2.yaml"),
            description="emcl2 節を持つ yaml"),
        DeclareLaunchArgument("scan_topic", default_value="/rfans/scan", description="入力 LaserScan"),
        DeclareLaunchArgument(
            "loc_map_yaml", default_value="",
            description="emcl2 専用地図 yaml (/map_loc)。空なら /map を読む"),
    ]

    emcl2 = Node(
        package="emcl2",
        executable="emcl2_node",
        name="emcl2",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
        remappings=[("scan", scan_topic), ("map", map_topic), ("initialpose", "/initialpose")],
    )

    # claude: include は GroupAction(scoped=True) で包む — 子 launch に渡した引数 (params_file 等) が
    #   親の launch configuration を上書きして後続 include に漏れるのを防ぐ (2026-10-08 実際に
    #   nav2.launch.py で loc_amcl の params_file が navigation 側に漏れ controller_server が落ちた)。
    loc_map = GroupAction([IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, "launch", "parts", "localization", "loc_map.launch.py")),
        launch_arguments={"use_sim_time": use_sim_time, "loc_map_yaml": loc_map_yaml}.items(),
    )], scoped=True)

    return LaunchDescription(args + [emcl2, loc_map])
