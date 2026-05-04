import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("rerobot_bringup")  # claude
    params_file = os.path.join(pkg_share, "config", "params.yaml")
    urdf_file = os.path.join(pkg_share, "urdf", "rerobot.urdf")

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

    #   robot_state_publisher_node = Node(
        #   package="robot_state_publisher",
        #   executable="robot_state_publisher",
        #   name="robot_state_publisher",
        #   parameters=[{"robot_description": robot_description}],
        #   # claude_tire: removed remap to /robot_encoder_states; epos4_odometry now
        #   # publishes /joint_states directly so RSP can emit dynamic wheel TFs.
        #   output="screen",
    #   )


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

    # Delay controller/odometry so the ros2_canopen device_manager has time to
    # advertise /motor*/cia402_device_*/{init,enable,cyclic_velocity_mode}.
    # Without this, the controller's constructor-time wait_for_service(1s) calls
    # race the bus_config launch and silently fail, leaving the EPOS4s disabled.
    delayed_nodes = TimerAction(
        period=5.0,
        actions=[epos4_controller_node, epos4_odometry_node],
    )

    return LaunchDescription([
        bus_config,
        delayed_nodes,
        # robot_state_publisher_node,
        rviz_node,  
    ])
