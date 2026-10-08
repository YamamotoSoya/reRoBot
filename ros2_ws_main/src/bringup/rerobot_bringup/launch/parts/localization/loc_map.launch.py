# claude: 2026-10-08 部品 — 自己位置推定専用の地図 (/map_loc)。nav2.launch.py (2026-10-04 追加分) から切り出し。
#   loc_map_yaml が空なら何も起動しない (= 自己位置推定も /map を読む)。
#   amcl / emcl2 どちらの部品からも include される共通部品。
#
# 自己位置推定用地図は屋根・軒・建物上部まで含めた広い帯で作り、rfans_scan.launch.py の
# amcl_min/max_height (/scan_all・/scan_far) を同じ帯にする。/map (経路計画・RViz) と keepout は従来どおり。
# lifecycle_manager の node_names は条件で切り替えられないので専用 manager を持つ。
# 自己位置推定ノードは transient_local で /map_loc を購読するので、起動順が前後しても受け取れる。
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    loc_map_yaml = LaunchConfiguration("loc_map_yaml")
    use_loc_map = PythonExpression(["'", loc_map_yaml, "' != ''"])

    map_server_loc = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server_loc",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time, "yaml_filename": loc_map_yaml, "topic_name": "map_loc"}],
        remappings=[("map", "/map_loc")],
        condition=IfCondition(use_loc_map),
    )
    lifecycle_manager_loc_map = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_loc_map",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time, "autostart": True, "node_names": ["map_server_loc"]}],
        condition=IfCondition(use_loc_map),
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("loc_map_yaml", default_value="",
                              description="自己位置推定専用地図 yaml (/map_loc で配信)。空なら起動しない"),
        map_server_loc,
        lifecycle_manager_loc_map,
    ])
