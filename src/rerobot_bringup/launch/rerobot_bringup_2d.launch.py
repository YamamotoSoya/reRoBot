# claude_opt: 2D LiDAR (HOKUYO urg_node) 構成の bringup。
#   共通部 (bus_config + epos4_controller + epos4_odometry + robot_state_publisher) は
#   rerobot_bringup_common.launch.py に抽出済みで、ここでは params/urdf の 2D 用
#   ファイル指定と urg_node の追加のみを行う。
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("rerobot_bringup")

    # claude: HOKUYO シリアルポート。udev rule 整備までは sudo ln -sf /dev/ttyUSB0
    # /dev/HOKUYO-LINK-SAMPLE で実体に紐付けるか、`serial_port:=/dev/ttyUSB0` で上書き。
    serial_port_arg = DeclareLaunchArgument(
        "serial_port",
        default_value="/dev/ttyUSB-utm-30lx",
        description="HOKUYO LiDAR serial device path",
    )

    common = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "rerobot_bringup_common.launch.py")
        ),
        launch_arguments={
            "params_file": os.path.join(pkg_share, "config", "params_2d.yaml"),
            "urdf_file": os.path.join(pkg_share, "urdf", "rerobot_2d.urdf"),
        }.items(),
    )

    # claude: HOKUYO laser driver. frame_id は URDF (rerobot_2d.urdf) の laser link 名と一致。
    # urg_node の executable 名は ROS 2 Jazzy では `urg_node_driver`。
    urg_node_node = Node(
        package="urg_node",
        executable="urg_node_driver",
        name="urg_node",
        parameters=[{
            "serial_port": LaunchConfiguration("serial_port"),
            "serial_baud": 115200,
            "frame_id": "laser",
            "calibrate_time": False,
            "publish_intensity": False,
            "publish_multiecho": False,
            "angle_min": -1.5708,
            "angle_max": 1.5708,
        }],
        output="screen",
        # claude_robust: シリアル断・USB 抜き差しでプロセスが死んだら自動再起動する。
        # LiDAR が止まると SLAM/Nav2 が静かに固まるため、落ちたら露骨に再起動ログを残す。
        respawn=True,
        respawn_delay=2.0,
    )

    return LaunchDescription([
        serial_port_arg,
        common,
        urg_node_node,
    ])
