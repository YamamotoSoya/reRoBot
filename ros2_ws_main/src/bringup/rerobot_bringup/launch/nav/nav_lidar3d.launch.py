# claude: 2026-10-08 nav 構成 (仮置き) — 3D 自己位置推定 + Nav2 + keepout + RViz。方式は未定。
#   現状は parts/localization/loc_lidar3d.launch.py がエラーを出して launch 全体を止める。
#
# robot 側: rerobot_bringup.launch.py (全部入り、既定のままで可)。 入力は R-Fans 点群 /rfans_driver/rfans_points。
# 経路計画用 /map は glim_dump_to_2dmap 由来の 2D 地図 (3D 地図と同じ map 座標系)。
#
# 構成ファイルの約束: nav_amcl.launch.py 冒頭を参照。
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    pkg_share = get_package_share_directory("rerobot_bringup")
    parts = os.path.join(pkg_share, "launch", "parts")
    nav2_params = os.path.join(pkg_share, "config", "nav2_params.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")
    map_dir = LaunchConfiguration("map_dir")
    map_yaml = LaunchConfiguration("map_yaml")
    keepout_yaml = LaunchConfiguration("keepout_yaml")
    use_keepout = LaunchConfiguration("use_keepout")

    args = [
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("map_dir", default_value="/workspace/maps/2d/slam_toolbox/2026_6_9__22:00",
                              description="地図一式 (nav2/, keep_out/) を含む親ディレクトリ"),
        DeclareLaunchArgument("map_yaml", default_value=PathJoinSubstitution([map_dir, "nav2", "map.yaml"]),
                              description="本体地図 yaml。指定すると map_dir 規約より優先"),
        DeclareLaunchArgument("keepout_yaml",
                              default_value=PathJoinSubstitution([map_dir, "keep_out", "keep_out.yaml"]),
                              description="keepout マスク yaml"),
        DeclareLaunchArgument("use_keepout", default_value="true",
                              description="keepout マスクを配信するか (マスク未作成の地図では false)"),
    ]

    # claude: include は GroupAction(scoped=True) で包む — 子 launch に渡した引数 (params_file 等) が
    #   親の launch configuration を上書きして後続 include に漏れるのを防ぐ (2026-10-08 実際に
    #   nav2.launch.py で loc_amcl の params_file が navigation 側に漏れ controller_server が落ちた)。
    def include(rel, launch_arguments):
        return GroupAction([IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(parts, rel)),
            launch_arguments=launch_arguments.items(),
        )], scoped=True)

    return LaunchDescription(args + [
        # 未実装の間は先頭で include して、他の部品が上がる前に Shutdown させる
        include("localization/loc_lidar3d.launch.py", {}),
        include("map_keepout.launch.py", {
            "use_sim_time": use_sim_time, "params_file": nav2_params,
            "map_yaml": map_yaml, "keepout_yaml": keepout_yaml, "use_keepout": use_keepout,
        }),
        include("navigation.launch.py", {"use_sim_time": use_sim_time, "params_file": nav2_params}),
        include("rviz_nav.launch.py", {
            "use_sim_time": use_sim_time, "rviz_config": os.path.join(pkg_share, "rviz", "nav2.rviz"),
        }),
    ])
