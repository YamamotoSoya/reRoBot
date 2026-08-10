# claude: IMU 取付向きの視覚検証用 launch (2026-08-10)。CAN/モータには一切触らない。
#   構成: bno086_imu_driver + robot_state_publisher (rerobot.urdf)
#         + joint_state_publisher (車輪 joint を 0 で埋めて RobotModel を完全表示)
#         + rviz2 (imu_check.rviz — TF 軸 + Imu 表示の加速度矢印)。
#   見方:
#     静止時: 加速度矢印 (赤) が真上を向けば URDF の roll/pitch (裏返し補正) が正しい。
#             下や横を向くなら imu_joint の rpy が実取付と不一致。
#     機首下げ (前を下げて保持): 矢印がロボット後方に傾けば yaw (X↔Y 入替) も正しい。
#             左右に傾くなら yaw の符号が逆 (π/2 → -π/2)。
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("rerobot_bringup")
    urdf_file = os.path.join(pkg_share, "urdf", "rerobot.urdf")
    rviz_file = os.path.join(pkg_share, "rviz", "imu_check.rviz")

    imu_port_arg = DeclareLaunchArgument(
        "imu_port", default_value="/dev/ttyACM0",
        description="BNO086 IMU board serial device path")

    with open(urdf_file, "r") as f:
        robot_description = f.read()

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        output="screen",
    )

    # claude: 実機の epos4_odometry が居ないので車輪 joint を 0 で埋める (見た目専用)
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
    )

    imu_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("bno086_imu_driver"),
                "launch",
                "bno086.launch.py",
            )
        ),
        launch_arguments={"port": LaunchConfiguration("imu_port")}.items(),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_file],
        output="screen",
    )

    return LaunchDescription([
        imu_port_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
        imu_include,
        rviz_node,
    ])
