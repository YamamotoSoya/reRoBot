# claude: 2026-10-08 部品 — 自己位置推定: AMCL (nav2_amcl)。nav2.launch.py から切り出し。
#
# 全 localization 部品の共通の約束:
#   出力 = TF map->odom (同時に 1 つの部品だけが出す)
#   入力 = /initialpose (RViz 2D Pose Estimate) + TF odom->base_link (EKF) + 地図 (/map か /map_loc)
#   lifecycle 管理は部品の中で完結させる (外の manager に名前を登録させない)
# AMCL 固有の出力: /amcl_pose, /particle_cloud
#
# claude: 2026-10-08 入力スキャンは scan_topic で直接指定する (旧 amcl_scan:=default/all/far の 3 択を廃止)。
#   地図に合うスキャンを選ぶ: GLIM 地図 = /rfans/scan (最近点) か /rfans/scan_all (全点)、slam_toolbox 地図 = /urg_front/scan
#   max_beams: /rfans/scan_all のときは 2000 (39,270 席 → 間隔 19 席、リング数 16 と互いに素)、それ以外は 60
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory("rerobot_bringup")

    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    loc_map_yaml = LaunchConfiguration("loc_map_yaml")
    amcl_map_topic = PythonExpression(["'/map_loc' if '", loc_map_yaml, "' != '' else 'map'"])

    args = [
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument(
            "params_file", default_value=os.path.join(pkg_share, "config", "localization", "amcl.yaml"),
            description="amcl 節を持つ yaml"),
        DeclareLaunchArgument("scan_topic", default_value="/rfans/scan", description="AMCL の入力 LaserScan"),
        DeclareLaunchArgument("max_beams", default_value="60",
                              description="1 スキャンで使うビーム本数 (/rfans/scan_all なら 2000)"),
        DeclareLaunchArgument(
            "loc_map_yaml", default_value="",
            description="AMCL 専用地図 yaml (/map_loc)。空なら AMCL は /map を読む"),
    ]

    amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=[params_file, {
            "use_sim_time": use_sim_time,
            "map_topic": amcl_map_topic,
            "scan_topic": LaunchConfiguration("scan_topic"),
            "max_beams": ParameterValue(LaunchConfiguration("max_beams"), value_type=int),
        }],
    )

    # claude: include は GroupAction(scoped=True) で包む — 子 launch に渡した引数 (params_file 等) が
    #   親の launch configuration を上書きして後続 include に漏れるのを防ぐ (2026-10-08 実際に
    #   nav2.launch.py で loc_amcl の params_file が navigation 側に漏れ controller_server が落ちた)。
    loc_map = GroupAction([IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, "launch", "parts", "localization", "loc_map.launch.py")),
        launch_arguments={"use_sim_time": use_sim_time, "loc_map_yaml": loc_map_yaml}.items(),
    )], scoped=True)

    # 名前は旧 nav2.launch.py と同じ lifecycle_manager_localization のまま —
    # RViz の Navigation 2 パネルがこの名前で localization の状態を問い合わせるため。
    lifecycle_manager_localization = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time, "autostart": True, "node_names": ["amcl"]}],
    )

    return LaunchDescription(args + [
        amcl,
        loc_map,
        lifecycle_manager_localization,
    ])
