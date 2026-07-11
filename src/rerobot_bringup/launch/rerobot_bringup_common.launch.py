# claude_opt: 2D/3D bringup の共通部。rerobot_bringup_2d.launch.py /
# rerobot_bringup_3d.launch.py から launch_arguments (params_file, urdf_file) 付きで
# include される。LiDAR ドライバは各ラッパー側が追加する。
#
# 構成: bus_config (ros2_canopen) + epos4_controller + epos4_odometry
#       + robot_state_publisher
#
# 旧構成にあった 5 秒の TimerAction は撤去した。かつては epos4_controller が
# コンストラクタ内で wait_for_service(1s) を連射していたため bus_config の起動
# (~3-4 s) とレースしたが、現行の controller は初期化を専用スレッドに移し、そこで
# init サービスの出現を最大 20 秒待ってから逐次投入 + SDO 検証するため、launch 側で
# 遅延させる必要がなくなった (src/epos4_controller/src/epos4_controller.cpp の
# run_init_sequence を参照)。
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    urdf_file = LaunchConfiguration("urdf_file")

    params_file_arg = DeclareLaunchArgument(
        "params_file",
        description="車体パラメータ yaml (params_2d.yaml / params_3d.yaml) の絶対パス",
    )
    urdf_file_arg = DeclareLaunchArgument(
        "urdf_file",
        description="robot_state_publisher に渡す URDF の絶対パス",
    )

    bus_config = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("maxon_epos4_ros2"),
                "launch",
                "bus_config_cia402_epos4_vel.launch.py",
            )
        )
    )

    epos4_controller_node = Node(
        package="epos4_controller",
        executable="epos4_controller",
        name="epos4_controller_node",
        parameters=[params_file],
        output="screen",
    )

    epos4_odometry_node = Node(
        package="epos4_controller",
        executable="epos4_odometry",
        name="epos4_odometry_node",
        parameters=[params_file],
        output="screen",
    )

    # claude: base_link -> {m1_wheel_link, m2_wheel_link, <lidar>} の TF を publish。
    # epos4_odometry が /joint_states (m1_wheel, m2_wheel の車輪側角度) を出すので
    # remap は不要。固定 LiDAR joint は /tf_static になる。
    # claude_opt: urdf_file は substitution (起動時に解決) のため、description 生成時の
    # open() では読めない。`cat` の Command substitution で中身を文字列として取り込む。
    robot_description = ParameterValue(Command(["cat ", urdf_file]), value_type=str)
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        output="screen",
    )

    # claude: RViz の起動はここでは行わない。可視化は nav2.launch.py (nav2.rviz) /
    # slam.launch.py (slam.rviz) 側に委譲し、bringup と重ねたときの窓の重複を避ける。

    return LaunchDescription([
        params_file_arg,
        urdf_file_arg,
        bus_config,
        epos4_controller_node,
        epos4_odometry_node,
        robot_state_publisher_node,
    ])
