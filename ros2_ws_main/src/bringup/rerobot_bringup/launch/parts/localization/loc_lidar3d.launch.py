# claude: 2026-10-08 部品 (仮置き) → 2026-10-10 実装 — 自己位置推定: 3D LiDAR + 3D 点群地図 (lidar_localization_ros2)。
#   設計: docs/features/2026-10-10_lidar_localization_3d.md
#
# 全 localization 部品の共通の約束 (loc_amcl.launch.py 冒頭):
#   出力 = TF map->odom (同時に 1 つの部品だけが出す) / 入力 = /initialpose + TF odom->base_link (EKF)
#   lifecycle 管理は部品の中で完結させる
# この部品では map->odom を 2 段で出す:
#   lidar_localization  : TF map_3d->odom   (3D 地図座標で解いた結果。level_map_to_odom で roll/pitch = 0、z は残る)
#   map_level_bridge.py : TF map->map_3d    (z を打ち消す) + /initialpose (map) → /initialpose_3d (map_3d) の中継
#                         (初期位置の z は traj_path の GLIM 軌跡から引く)
# 固有の出力: /pcl_pose (map_3d 座標), /path, /alignment_status, /reinitialization_requested, /initial_map
#
# 本体付属の launch (lidar_localization.launch.py 等) は include しない — LiDAR / IMU の static TF を自前で出すため、
#   robot_state_publisher (URDF) と二重配信になる。Node を直接書く。
# lifecycle は本体付属の start_lifecycle_node.py で configure → activate する。nav2 lifecycle_manager は使わない
#   (v1.3.0 は use_bond_ をパラメータから有効化する経路が無く bond を張らない → bond 前提の manager と相性が悪い)。
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, OpaqueFunction
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def check_map(context, *args, **kwargs):
    # 地図が無いと lidar_localization は configure で失敗するだけで原因が分かりにくいので、先に止める
    map_path = LaunchConfiguration("map_path").perform(context)
    if map_path and os.path.isfile(map_path):
        return [LogInfo(msg=f"[loc_lidar3d] 3D 地図: {map_path}")]
    return [
        LogInfo(msg=f"[loc_lidar3d] 3D 地図が見つからない: '{map_path}' — map_path:=<.pcd/.ply> を指定する"),
        EmitEvent(event=Shutdown(reason="loc_lidar3d: map_path not found")),
    ]


def generate_launch_description():
    pkg_share = get_package_share_directory("rerobot_bringup")
    upstream_preset = os.path.join(
        get_package_share_directory("lidar_localization_ros2"), "param", "nav2_ndt_urban.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")

    args = [
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("map_path", default_value="", description="3D 点群地図 (.pcd / .ply, map_3d 座標)"),
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(pkg_share, "config", "localization", "lidar_localization.yaml"),
            description="本機の差分パラメータ (上流プリセット nav2_ndt_urban.yaml の後に重ねる)"),
        DeclareLaunchArgument("cloud_topic", default_value="/rfans_driver/rfans_points", description="入力点群"),
        # seed は use_odom_tf_prediction (EKF の TF) で取るので /odom topic は既定構成では読まれない (use_odom: false)。
        #   use_odom を A/B で試すときのために remap だけ残す
        DeclareLaunchArgument("odom_topic", default_value="/odometry/filtered", description="use_odom: true 時の odom"),
        DeclareLaunchArgument("imu_topic", default_value="/imu/data",
                              description="IMU (use_imu_preintegration: true のときだけ使われる)"),
        DeclareLaunchArgument("traj_path", default_value="",
                              description="地図と同じ GLIM dump の traj_lidar.txt。初期位置の z をここから引く (空なら initial_z)"),
        DeclareLaunchArgument("initial_z", default_value="0.0",
                              description="traj が無いときに使うスタート地点の base_link の z [m] (map_3d 座標)"),
        DeclareLaunchArgument("startup_timeout", default_value="120.0",
                              description="configure (地図ロード込み) → activate の待ち時間上限 [s]"),
    ]

    lidar_localization = Node(
        package="lidar_localization_ros2",
        executable="lidar_localization_node",
        name="lidar_localization",
        output="screen",
        parameters=[
            upstream_preset,
            LaunchConfiguration("params_file"),
            {"use_sim_time": use_sim_time, "map_path": LaunchConfiguration("map_path")},
        ],
        remappings=[
            ("/cloud", LaunchConfiguration("cloud_topic")),
            ("/odom", LaunchConfiguration("odom_topic")),
            ("/imu", LaunchConfiguration("imu_topic")),
            ("/initialpose", "/initialpose_3d"),  # RViz の /initialpose は map_level_bridge が map_3d に直して渡す
            # 本体は点群地図を topic で受ける入口 /map (PointCloud2) も購読する (use_pcd_map: true なので未使用)。
            #   Nav2 の /map (OccupancyGrid, map_server) と同名・別型になるので逃がす (2026-10-10 統合試験で発見)
            ("/map", "/lidar_localization/map_cloud_in"),
        ],
    )

    startup = Node(
        package="lidar_localization_ros2",
        executable="start_lifecycle_node.py",
        name="lidar_localization_startup",
        output="screen",
        arguments=["lidar_localization", "--timeout", LaunchConfiguration("startup_timeout")],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    map_level_bridge = Node(
        package="rerobot_bringup",
        executable="map_level_bridge.py",
        name="map_level_bridge",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "map_frame": "map",
            "map3d_frame": "map_3d",  # lidar_localization.yaml の global_frame_id と一致させる
            "odom_frame": "odom",
            # initial_z:=0 のような整数表記でも declare_parameter (float) と型が食い違わないよう固定
            "initial_z": ParameterValue(LaunchConfiguration("initial_z"), value_type=float),
            "traj_path": LaunchConfiguration("traj_path"),
        }],
    )

    return LaunchDescription(args + [
        OpaqueFunction(function=check_map),
        lidar_localization,
        startup,
        map_level_bridge,
    ])
