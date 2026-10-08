# claude: 3D LiDAR のみ (IMU なし) 互換ラッパ (2026-08-10 に統合 launch 化)。
#   実体は rerobot_bringup.launch.py。scripts/bringup3d.sh / glim3d.sh 等の既存呼び出しを
#   壊さないために名前を維持している。IMU 込みは rerobot_bringup_3d_imu.launch.py。
#   固定: lidar_2d=false, lidar_3d=true, imu=false
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = get_package_share_directory("rerobot_bringup")

    passthrough_args = [
        DeclareLaunchArgument("device_ip", default_value="192.168.0.3",
                              description="R-Fans LiDAR device IP (UDP source)"),
        DeclareLaunchArgument("rps", default_value="10",
                              description="R-Fans scan speed [Hz]: 5 / 10 / 20"),
        DeclareLaunchArgument("model", default_value="R-Fans-16",
                              description="R-Fans model: R-Fans-32 / R-Fans-16 / R-Fans-V6K / C-Fans-128 / C-Fans-32"),
    ]

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "rerobot_bringup.launch.py")),
        launch_arguments={
            "lidar_2d": "false",
            "lidar_3d": "true",
            "imu": "false",
            "ekf": "false",  # claude: 2026-10-08 実体の ekf 既定が true になったため明示 (従来挙動維持)
            "device_ip": LaunchConfiguration("device_ip"),
            "rps": LaunchConfiguration("rps"),
            "model": LaunchConfiguration("model"),
        }.items(),
    )

    return LaunchDescription(passthrough_args + [bringup])
