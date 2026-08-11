# claude: 2D + 3D LiDAR + IMU 全部載せ構成ラッパ (2026-08-10)。実体は rerobot_bringup.launch.py。
#   固定: lidar_2d=true, lidar_3d=true, imu=true
#   接続系引数 (serial_port / device_ip / rps / model / imu_port) はそのまま透過する。
#   2D SLAM/Nav2 と 3D SLAM (GLIM) を同時に走らせる・比較する用途向け。
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
        DeclareLaunchArgument("device_ip", default_value="192.168.0.3",
                              description="R-Fans LiDAR device IP (UDP source)"),
        DeclareLaunchArgument("rps", default_value="10",
                              description="R-Fans scan speed [Hz]: 5 / 10 / 20"),
        DeclareLaunchArgument("model", default_value="R-Fans-16",
                              description="R-Fans model"),
        DeclareLaunchArgument("imu_port", default_value="/dev/ttyACM0",
                              description="BNO086 IMU board serial device path"),
        # claude_ekf: 車輪 odom + IMU の EKF 融合 (robot_localization) を透過 (2026-08-11)
        DeclareLaunchArgument("ekf", default_value="false",
                              description="Fuse wheel odom + IMU with robot_localization EKF"),
    ]

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "rerobot_bringup.launch.py")),
        launch_arguments={
            "lidar_2d": "true",
            "lidar_3d": "true",
            "imu": "true",
            "serial_port": LaunchConfiguration("serial_port"),
            "device_ip": LaunchConfiguration("device_ip"),
            "rps": LaunchConfiguration("rps"),
            "model": LaunchConfiguration("model"),
            "imu_port": LaunchConfiguration("imu_port"),
            "ekf": LaunchConfiguration("ekf"),  # claude_ekf
        }.items(),
    )

    return LaunchDescription(passthrough_args + [bringup])
