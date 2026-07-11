# claude
# RealSense を 6軸IMU として起動し、imu_filter_madgwick で orientation を
# 合成して /imu/data に出す launch。
#
#   [realsense2_camera_node] --/camera/camera/imu--> [madgwick] --/imu/data--> (LIO-SAM)
#
# - rs_launch.py (realsense2_camera_launch) は IMU 専用 default 済み
#   (color/depth/infra/rgbd 無効、gyro/accel/motion 有効、unite_imu_method=2)。
# - LIO-SAM (ros2 branch) の imuTopic 既定 "/imu/data" と madgwick の既定
#   出力名が一致するため、出力側の remap は不要。
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # IMU レート。0 = デバイス既定。LIO-SAM は 200Hz 以上推奨なので
    # D435i では gyro_fps:=400 accel_fps:=250 を推奨。
    gyro_fps_arg = DeclareLaunchArgument(
        'gyro_fps', default_value='0',
        description='RealSense gyro rate [Hz] (0=device default)')
    accel_fps_arg = DeclareLaunchArgument(
        'accel_fps', default_value='0',
        description='RealSense accel rate [Hz] (0=device default)')

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('realsense2_camera_launch'),
                'launch', 'rs_launch.py',
            ])
        ),
        launch_arguments={
            'gyro_fps': LaunchConfiguration('gyro_fps'),
            'accel_fps': LaunchConfiguration('accel_fps'),
        }.items(),
    )

    # 6軸 (gyro+accel) から orientation を合成する Madgwick フィルタ。
    # RealSense は磁気センサ非搭載のため use_mag=false。yaw は磁気参照が
    # 無くドリフトするが、LIO-SAM は imuRPYWeight=0.01 で初期姿勢ヒントに
    # しか使わないため実用上問題ない。
    madgwick_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter_madgwick',
        output='screen',
        parameters=[{
            'use_mag': False,
            'world_frame': 'enu',
            # TF は robot_state_publisher / LIO-SAM 側の責務なので出さない
            'publish_tf': False,
        }],
        remappings=[
            # unite_imu_method=2 (linear_interpolation) の統合トピック
            ('imu/data_raw', '/camera/camera/imu'),
            # 出力は既定の imu/data → /imu/data (LIO-SAM の imuTopic と一致)
        ],
    )

    return LaunchDescription([
        gyro_fps_arg,
        accel_fps_arg,
        realsense_launch,
        madgwick_node,
    ])
