# claude: 2026-09-23 比較用 IMU WITmotion WT901C-TTL を witmotion_ros で起こす launch。
# BNO086 (rerobot_bringup.launch.py imu:=true → /imu/data) と同時起動して両者を bag に残す用。
#
#   [WT901C-TTL] --USB-TTL(/dev/ttyUSB-wt901, 115200)--> [witmotion_ros_node] --> /imu_wit/data
#                                                                              --> /imu_wit/mag
# 前提:
#   - ros2_ws_main/src/drivers/witmotion_ros (ElettraSciComp/witmotion_IMU_ros, ros2 branch) がビルド済み
#   - センサ側は Windows 公式ソフトで 115200 baud / 200 Hz に変更済み (2026-09-23)
# 設定は config/wt901.yaml (topic / frame_id / use_native_orientation=false の理由はそちら)。
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_arg = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution(
            [FindPackageShare("rerobot_bringup"), "config", "wt901.yaml"]),
        description="witmotion_ros parameter file")
    port_arg = DeclareLaunchArgument(
        "port", default_value="ttyUSB-wt901",
        description="Serial device name under /dev (udev symlink or ttyUSBn)")

    node = Node(
        package="witmotion_ros",
        executable="witmotion_ros_node",
        name="witmotion",
        output="screen",
        parameters=[LaunchConfiguration("params_file"),
                    {"port": LaunchConfiguration("port")}],
    )
    return LaunchDescription([params_arg, port_arg, node])
