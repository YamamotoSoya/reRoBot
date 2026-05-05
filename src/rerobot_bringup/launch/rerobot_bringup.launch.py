import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction  # claude
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration  # claude
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("rerobot_bringup")  # claude
    params_file = os.path.join(pkg_share, "config", "params.yaml")
    urdf_file = os.path.join(pkg_share, "urdf", "rerobot.urdf")

    # claude: HOKUYO シリアルポート。udev rule 整備までは sudo ln -sf /dev/ttyUSB0
    # /dev/HOKUYO-LINK-SAMPLE で実体に紐付けるか、`serial_port:=/dev/ttyUSB0` で上書き。
    serial_port_arg = DeclareLaunchArgument(
        "serial_port",
        default_value="/dev/ttyUSB-utm-30lx",
        description="HOKUYO LiDAR serial device path",
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

    # claude: re-enabled to publish base_link -> {m1_wheel_link, m2_wheel_link, laser} TFs.
    # epos4_odometry already publishes /joint_states (m1_wheel, m2_wheel wheel-side angles),
    # so no remap is needed. The fixed laser joint becomes a /tf_static entry.
    
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        output="screen",
    )
    

    # # claude: auto-start rviz2 with bundled preset (odom fixed frame,
    # RobotModel / TF / Odometry displays). Not wrapped in TimerAction because
    # rviz2 has no CANopen service dependency and its subscribers will pick up
    # /odom and /tf as soon as the delayed nodes come online.
    rviz_config = os.path.join(pkg_share, "rviz", "rerobot.rviz")  # claude
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
    )

    # claude: HOKUYO laser driver. frame_id は URDF (rerobot.urdf) の laser link 名と一致。
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
        serial_port_arg,  # claude
        bus_config,
        delayed_nodes,
        robot_state_publisher_node,  # claude
        rviz_node,
        urg_node_node,  # claude
    ])
