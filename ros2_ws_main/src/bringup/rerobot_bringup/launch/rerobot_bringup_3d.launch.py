# claude: 3D LiDAR (R-Fans / rfans_driver) 構成用の composite bringup。
#   rerobot_bringup_2d.launch.py を雛形に、urg_node を rfans_driver に差し替えたもの。
#   構成: bus_config -> (5s) -> epos4_controller + epos4_odometry
#         + robot_state_publisher(rerobot_3d.urdf) + rfans_driver。
#   rfans の param は config/params_3d.yaml の rfans_driver セクションから読む。
#   device_ip / rps / model は launch 引数で yaml の値を上書き可能。
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
# claude: launch 引数は文字列で解決されるため、int 等の型付き param は ParameterValue で包む
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory("rerobot_bringup")  # claude
    params_file = os.path.join(pkg_share, "config", "params_3d.yaml")  # claude: 3D 用
    urdf_file = os.path.join(pkg_share, "urdf", "rerobot_3d.urdf")  # claude: 3D 用

    # claude: R-Fans 接続/機種の上書き引数。未指定なら params_3d.yaml の値を使う。
    device_ip_arg = DeclareLaunchArgument(
        "device_ip",
        default_value="192.168.0.3",
        description="R-Fans LiDAR device IP (UDP source)",
    )
    rps_arg = DeclareLaunchArgument(
        "rps",
        default_value="10",
        description="R-Fans scan speed [Hz]: 5 / 10 / 20",
    )
    model_arg = DeclareLaunchArgument(
        "model",
        default_value="R-Fans-16",
        description="R-Fans model: R-Fans-32 / R-Fans-16 / R-Fans-V6K / C-Fans-128 / C-Fans-32",
    )

    with open(urdf_file, "r") as f:
        robot_description = f.read()

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

    # claude: base_link -> {m1_wheel_link, m2_wheel_link, rfans} の TF を publish。
    # 固定 rfans_joint は /tf_static になる。
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        output="screen",
    )

    # claude: RViz の起動はここでは行わない（nav2.launch.py / slam.launch.py に委譲）。

    # claude: R-Fans 3D LiDAR driver。frame_id 等の静的 param は params_3d.yaml
    # (rfans_driver セクション) から読み、device_ip / rps / model のみ launch 引数で
    # 上書きする（後勝ちで yaml の値を置き換え）。出力は PointCloud2 /sdk_could。
    rfans_node = Node(
        package="rfans_driver",
        executable="driver_node",
        name="rfans_driver",  # claude: params_3d.yaml の key と一致必須
        parameters=[
            params_file,
            {
                "device_ip": LaunchConfiguration("device_ip"),
                "rps": ParameterValue(LaunchConfiguration("rps"), value_type=int),
                "model": LaunchConfiguration("model"),
            },
        ],
        # claude: libstar.so (ベンダー blob) が古い C++ 例外ランタイム (__cxa_throw 等) を
        #   export しており、FastDDS がポート衝突時に投げる正常系例外の unwind を横取りして
        #   SIGABRT で即死する (他ノードが同一ドメインにいると 100% 再現)。正規の
        #   libstdc++/libgcc を先に解決させて無効化する (2026-08-01)。
        additional_env={
            "LD_PRELOAD": "/usr/lib/x86_64-linux-gnu/libstdc++.so.6:"
                          "/lib/x86_64-linux-gnu/libgcc_s.so.1"
        },
        output="screen",
    )

    # Delay controller/odometry so the ros2_canopen device_manager has time to
    # advertise /motor*/cia402_device_*/{init,enable,cyclic_velocity_mode}.
    # Without this, the controller's constructor-time wait_for_service(1s) calls
    # race the bus_config launch and silently fail, leaving the EPOS4s disabled.
    delayed_nodes = TimerAction(
        period=5.0,
        actions=[epos4_controller_node, epos4_odometry_node],
    )

    return LaunchDescription([
        device_ip_arg,  # claude
        rps_arg,        # claude
        model_arg,      # claude
        bus_config,
        delayed_nodes,
        robot_state_publisher_node,  # claude
        rfans_node,     # claude
    ])
