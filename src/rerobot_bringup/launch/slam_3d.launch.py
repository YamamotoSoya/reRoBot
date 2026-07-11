# claude: 3D mapping (LIO-SAM) 用 launch。
#
# 前提: rerobot_bringup_3d.launch.py を **odom_tf:=false で** 先に起動しておくこと。
#   ros2 launch rerobot_bringup rerobot_bringup_3d.launch.py odom_tf:=false
# 理由: LIO-SAM (imuPreintegration) が odom→base_link TF を自分で出すため、
# epos4_odometry の同 TF と競合する。mapping 中は LIO-SAM が odom→base_link を
# 所有し、epos4_odometry は /odom トピックのみ出す (TF off)。
#
# 本 launch が立ち上げるもの:
#   - realsense_imu.launch.py     : RealSense IMU + madgwick → /imu/data
#   - rfans_ring_converter        : /sdk_could → /points (ring/time 付与)
#   - LIO-SAM 4 ノード             : /points + /imu/data → 地図 (+ odom→base_link TF)
#   - static map→odom (恒等)      : mapping 中は補正なしのため恒等で埋める
#     (LIO-SAM 上流 run.launch.py と同じ構成。map→odom は誰も動的に出さない)
#   - rviz2 (LIO-SAM 同梱ビュー)   : 構築中の 3D 地図を表示
#
# TF ツリー (mapping 時):
#   map ──静的恒等── odom ──LIO-SAM── base_link ──URDF── rfans/laser/...
#   ※ LIO-SAM mapOptimization は odom→"lidar_link" (ハードコード名) も出すが、
#     本機の LiDAR frame は rfans なので孤立フレームになるだけで無害。
#
# 地図の保存 (走行終了後):
#   ros2 service call /lio_sam/save_map lio_sam/srv/SaveMap "{resolution: 0.2, destination: /maps/lio_sam/<日付>}"
#   → $HOME/<destination>/ に PCD 一式が保存される。その後 pcd_to_gridmap で
#     Nav2 用 2D 地図に投影する (docs/features/ の 3D 自律移動文書参照)。
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("rerobot_bringup")
    lio_sam_share = get_package_share_directory("lio_sam")

    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(pkg_share, "config", "lio_sam_params.yaml"),
        description="LIO-SAM parameter file (reRoBot 用に調整済みのもの)",
    )
    params_file = LaunchConfiguration("params_file")

    # RealSense IMU + madgwick → /imu/data (LIO-SAM の imuTopic)
    imu = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "realsense_imu.launch.py")
        ),
    )

    # /sdk_could → /points (laserid→ring, timeflag→time)。LIO-SAM の入力形式に変換
    ring_converter = Node(
        package="rerobot_perception",
        executable="rfans_ring_converter",
        name="rfans_ring_converter",
        parameters=[{"input_topic": "/sdk_could", "output_topic": "/points", "n_scan": 16}],
        output="screen",
        respawn=True,
        respawn_delay=2.0,
    )

    # mapping 中の map→odom は恒等 (LIO-SAM 上流 run.launch.py と同じ)
    static_map_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments="--frame-id map --child-frame-id odom".split(" "),
        output="screen",
    )

    lio_sam_nodes = [
        Node(
            package="lio_sam",
            executable=exe,
            name=exe,
            parameters=[params_file],
            output="screen",
        )
        for exe in (
            "lio_sam_imuPreintegration",
            "lio_sam_imageProjection",
            "lio_sam_featureExtraction",
            "lio_sam_mapOptimization",
        )
    ]

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", os.path.join(lio_sam_share, "config", "rviz2.rviz")],
        output="screen",
    )

    return LaunchDescription([
        params_file_arg,
        imu,
        ring_converter,
        static_map_to_odom,
        *lio_sam_nodes,
        rviz_node,
    ])
