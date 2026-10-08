# claude: 2026-10-08 部品 — 経路計画用の地図 (/map) と keepout (進入禁止帯) の配信。
#   nav2.launch.py から切り出し。自己位置推定の方式には依存しない (どの nav 構成でも共通)。
#
#   map_server                 : /map (global costmap の static_layer・RViz・AMCL/emcl2 の既定入力)
#   filter_mask_server         : /keepout_filter_mask   (use_keepout:=true のときだけ)
#   costmap_filter_info_server : /costmap_filter_info   (同上)
#
# lifecycle_manager は「地図」と「keepout」で分ける。旧 nav2.launch.py は 1 つの manager に
# amcl・keepout まで束ねていたため、keepout あり/なしで同名 manager を 2 定義していた。
# 分けたことで条件付き起動が manager 単位で済む。
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("rerobot_bringup")

    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    map_yaml = LaunchConfiguration("map_yaml")
    keepout_yaml = LaunchConfiguration("keepout_yaml")
    use_keepout = LaunchConfiguration("use_keepout")

    args = [
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument(
            "params_file", default_value=os.path.join(pkg_share, "config", "nav2_params.yaml"),
            description="map_server / filter_mask_server / costmap_filter_info_server の節を持つ yaml"),
        DeclareLaunchArgument("map_yaml", description="本体地図 yaml へのフルパス (/map)"),
        DeclareLaunchArgument("keepout_yaml", default_value="",
                              description="keepout マスク yaml へのフルパス。use_keepout:=false なら未使用"),
        DeclareLaunchArgument("use_keepout", default_value="true",
                              description="keepout マスクを配信するか。マスク未作成の地図では false"),
    ]

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time, "yaml_filename": map_yaml}],
    )
    lifecycle_manager_map = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_map",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time, "autostart": True, "node_names": ["map_server"]}],
    )

    # keepout マスク配信サーバ (map_server 実体を別名・別トピックで起動)。
    # マスク yaml が無いと configure が失敗するので、未作成の地図では use_keepout:=false で丸ごと外す。
    filter_mask_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="filter_mask_server",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time, "yaml_filename": keepout_yaml}],
        condition=IfCondition(use_keepout),
    )
    # マスク値→コスト変換則の配信サーバ。
    costmap_filter_info_server = Node(
        package="nav2_map_server",
        executable="costmap_filter_info_server",
        name="costmap_filter_info_server",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
        condition=IfCondition(use_keepout),
    )
    lifecycle_manager_keepout = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_keepout",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "autostart": True,
            "node_names": ["filter_mask_server", "costmap_filter_info_server"],
        }],
        condition=IfCondition(use_keepout),
    )

    return LaunchDescription(args + [
        map_server,
        lifecycle_manager_map,
        filter_mask_server,
        costmap_filter_info_server,
        lifecycle_manager_keepout,
    ])
