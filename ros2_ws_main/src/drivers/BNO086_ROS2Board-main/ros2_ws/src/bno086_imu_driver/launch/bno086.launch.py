from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg = FindPackageShare('bno086_imu_driver')

    args = [
        DeclareLaunchArgument('port', default_value='/dev/ttyACM0',
                              description='Serial device of the board'),
        DeclareLaunchArgument('frame_id', default_value='imu_link',
                              description='frame_id stamped on the published messages'),
        DeclareLaunchArgument('imu_rate_hz', default_value='100.0',
                              description='Requested IMU report rate'),
        DeclareLaunchArgument('params_file',
                              default_value=PathJoinSubstitution([pkg, 'config', 'bno086.yaml']),
                              description='Parameter file to load'),
        DeclareLaunchArgument('namespace', default_value='',
                              description='Namespace for the driver node'),
    ]

    node = Node(
        package='bno086_imu_driver',
        executable='imu_node',
        name='bno086_imu_driver',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {
                # Launch arguments win over the file so the common overrides
                # can be given on the command line.
                'port': LaunchConfiguration('port'),
                'frame_id': LaunchConfiguration('frame_id'),
                'imu_rate_hz': LaunchConfiguration('imu_rate_hz'),
            },
        ],
    )

    return LaunchDescription(args + [node])
