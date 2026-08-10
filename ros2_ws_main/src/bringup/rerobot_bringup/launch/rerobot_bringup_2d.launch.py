# claude: 2D LiDAR のみ (IMU なし) 互換ラッパ (2026-08-10 に統合 launch 化)。
#   実体は rerobot_bringup.launch.py。scripts/bringup2d.sh 等の既存呼び出しを
#   壊さないために名前を維持している。IMU 込みは rerobot_bringup_2d_imu.launch.py。
#   固定: lidar_2d=true, lidar_3d=false, imu=false
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = get_package_share_directory("rerobot_bringup")

    passthrough_args = [
        DeclareLaunchArgument("serial_port", default_value="/dev/ttyUSB-utm-30lx",
                              description="HOKUYO LiDAR serial device path"),
    ]

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "rerobot_bringup.launch.py")),
        launch_arguments={
            "lidar_2d": "true",
            "lidar_3d": "false",
            "imu": "false",
            "serial_port": LaunchConfiguration("serial_port"),
        }.items(),
    )

    return LaunchDescription(passthrough_args + [bringup])
