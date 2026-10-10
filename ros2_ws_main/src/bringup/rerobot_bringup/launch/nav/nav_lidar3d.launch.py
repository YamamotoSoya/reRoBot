# claude: 2026-10-08 nav 構成 (仮置き) → 2026-10-10 実装 — 3D 自己位置推定 (lidar_localization_ros2) + Nav2 + keepout + RViz。
#   設計: docs/features/2026-10-10_lidar_localization_3d.md
#
# robot 側: rerobot_bringup.launch.py (全部入り、既定のままで可 — R-Fans 点群・EKF・IMU を使う)。
# 地図は同じ GLIM dump から 2 枚焼いたものを map_name で対にして渡す (x, y, yaw を一致させるため):
#   2D (経路計画・keepout): /workspace/maps/2d/glim/<map_name>/{nav2/map.yaml, keep_out/keep_out.yaml}
#   3D (自己位置推定)     : /workspace/maps/3d/<map_name>/{map.pcd, traj_lidar.txt}  (traj = 初期位置の z を引く GLIM 軌跡)
#   別名の地図を使うときは map_dir / map_yaml / map_path を直接指定する。
# map->odom を出すのは loc_lidar3d だけ (AMCL / emcl2 は include しない)。
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
    map_name = LaunchConfiguration("map_name")
    map_dir = LaunchConfiguration("map_dir")
    map_yaml = LaunchConfiguration("map_yaml")
    keepout_yaml = LaunchConfiguration("keepout_yaml")
    use_keepout = LaunchConfiguration("use_keepout")
    map_path = LaunchConfiguration("map_path")

    args = [
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("map_name", default_value="",
                              description="地図名 (2D と 3D で共通のディレクトリ名。同じ GLIM dump 由来であること)"),
        DeclareLaunchArgument("map_dir", default_value=PathJoinSubstitution(["/workspace/maps/2d/glim", map_name]),
                              description="2D 地図一式 (nav2/, keep_out/) を含む親ディレクトリ"),
        DeclareLaunchArgument("map_yaml", default_value=PathJoinSubstitution([map_dir, "nav2", "map.yaml"]),
                              description="2D 本体地図 yaml。指定すると map_dir 規約より優先"),
        DeclareLaunchArgument("keepout_yaml",
                              default_value=PathJoinSubstitution([map_dir, "keep_out", "keep_out.yaml"]),
                              description="keepout マスク yaml"),
        DeclareLaunchArgument("use_keepout", default_value="true",
                              description="keepout マスクを配信するか (マスク未作成の地図では false)"),
        DeclareLaunchArgument("map_path", default_value=PathJoinSubstitution(["/workspace/maps/3d", map_name, "map.pcd"]),
                              description="3D 点群地図 (.pcd / .ply)。指定すると map_name 規約より優先"),
        DeclareLaunchArgument("traj_path",
                              default_value=PathJoinSubstitution(["/workspace/maps/3d", map_name, "traj_lidar.txt"]),
                              description="map.pcd と同じ GLIM dump の traj_lidar.txt (初期位置の z を引く)"),
        DeclareLaunchArgument("initial_z", default_value="0.0",
                              description="traj が無いときのスタート地点の base_link の z [m] (map_3d 座標)"),
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
        # 先頭で include — 3D 地図が無ければ他の部品が上がる前に Shutdown する
        include("localization/loc_lidar3d.launch.py", {
            "use_sim_time": use_sim_time, "map_path": map_path,
            "initial_z": LaunchConfiguration("initial_z"), "traj_path": LaunchConfiguration("traj_path"),
        }),
        include("map_keepout.launch.py", {
            "use_sim_time": use_sim_time, "params_file": nav2_params,
            "map_yaml": map_yaml, "keepout_yaml": keepout_yaml, "use_keepout": use_keepout,
        }),
        include("navigation.launch.py", {"use_sim_time": use_sim_time, "params_file": nav2_params}),
        include("rviz_nav.launch.py", {
            "use_sim_time": use_sim_time, "rviz_config": os.path.join(pkg_share, "rviz", "nav2.rviz"),
        }),
    ])
