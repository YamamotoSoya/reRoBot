# claude: 統合 bringup (2026-08-10)。実機には 2D LiDAR (UTM-30LX) と 3D LiDAR (R-Fans-16)
#   が両方常設 + BNO086 IMU 搭載予定のため、旧 rerobot_bringup_{2d,3d}.launch.py を
#   1 本に統合し、起動するセンサドライバを boolean 引数で選ぶ方式にした。
#     lidar_2d:=true/false  ... urg_node (HOKUYO, /scan)
#     lidar_3d:=true/false  ... rfans_driver (R-Fans, /sdk_could)
#     imu:=true/false       ... bno086_imu_driver (/imu/data)
#     ekf:=true/false       ... robot_localization EKF (車輪 odom + IMU 融合, 2026-08-11)。
#                               true で /odometry/filtered + TF odom->base_link を EKF が
#                               担当し、epos4_odometry の TF は自動オフ。imu:=true と併用。
#   URDF は rerobot.urdf 1 本 (laser / rfans / imu_link を常に含む — 使わないセンサの
#   静的 TF が出ていても無害)。params は config/params.yaml 1 本。
#   直接叩いてもよいが、構成別ラッパ (rerobot_bringup_{2d,3d}{,_imu}.launch.py /
#   rerobot_bringup_2d3d_imu.launch.py) を使うのが推奨。
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
# claude: launch 引数は文字列で解決されるため、int 等の型付き param は ParameterValue で包む
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory("rerobot_bringup")
    params_file = os.path.join(pkg_share, "config", "params.yaml")
    urdf_file = os.path.join(pkg_share, "urdf", "rerobot.urdf")

    # claude: センサ選択引数。ラッパ launch がここを固定して構成を作る。
    lidar_2d_arg = DeclareLaunchArgument(
        "lidar_2d", default_value="true",
        description="Start urg_node (HOKUYO UTM-30LX, /scan)")
    lidar_3d_arg = DeclareLaunchArgument(
        "lidar_3d", default_value="true",
        description="Start rfans_driver (R-Fans-16, /sdk_could)")
    imu_arg = DeclareLaunchArgument(
        "imu", default_value="true",
        description="Start bno086_imu_driver (/imu/data)")
    # claude_ekf: 車輪 odom + IMU の EKF 融合 (robot_localization)。true にすると
    # ekf_node が /odometry/filtered と TF odom->base_link を出し、epos4_odometry の
    # publish_tf を自動で false にする (TF 二重配信の防止。/odom topic 自体は残る)。
    ekf_arg = DeclareLaunchArgument(
        "ekf", default_value="false",
        description="Fuse wheel odom + IMU with robot_localization EKF")

    # claude: HOKUYO シリアルポート。udev rule (/etc/udev/rules.d/99-hokuyo-devices.rules,
    # Hokuyo VID 15d1 → /dev/ttyUSB-utm-30lx) が整備済みなので、USB を挿せば自動で
    # symlink が生える (実体は cdc_acm の /dev/ttyACM*)。symlink が無い = 物理的に未接続。
    serial_port_arg = DeclareLaunchArgument(
        "serial_port", default_value="/dev/ttyUSB-utm-30lx",
        description="HOKUYO LiDAR serial device path")

    # claude: R-Fans 接続/機種の上書き引数。未指定なら params.yaml の値を使う。
    device_ip_arg = DeclareLaunchArgument(
        "device_ip", default_value="192.168.0.3",
        description="R-Fans LiDAR device IP (UDP source)")
    rps_arg = DeclareLaunchArgument(
        "rps", default_value="10",
        description="R-Fans scan speed [Hz]: 5 / 10 / 20")
    model_arg = DeclareLaunchArgument(
        "model", default_value="R-Fans-16",
        description="R-Fans model: R-Fans-32 / R-Fans-16 / R-Fans-V6K / C-Fans-128 / C-Fans-32")

    # claude: BNO086 ボードの USB CDC デバイス。udev rule (tools/99-bno086.rules) 導入後は
    # 安定名に差し替え可。
    imu_port_arg = DeclareLaunchArgument(
        "imu_port", default_value="/dev/ttyACM0",
        description="BNO086 IMU board serial device path")

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

    # claude_ekf: ekf:=true のとき publish_tf を後勝ちで false に上書きする
    # (odom->base_link は ekf_node が出すため)。launch 引数は文字列なので
    # PythonExpression で bool 文字列に変換し ParameterValue(bool) で型付けする。
    odom_publish_tf = ParameterValue(
        PythonExpression(["'", LaunchConfiguration("ekf"), "'.lower() != 'true'"]),
        value_type=bool)
    epos4_odometry_node = Node(
        package="epos4_controller",
        executable="epos4_odometry",
        name="epos4_odometry_node",
        parameters=[params_file, {"publish_tf": odom_publish_tf}],
        output="screen",
    )

    # claude_ekf: robot_localization EKF (車輪 odom + IMU 融合)。config は ekf.yaml。
    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        condition=IfCondition(LaunchConfiguration("ekf")),
        parameters=[os.path.join(pkg_share, "config", "ekf.yaml")],
        output="screen",
    )

    # claude: base_link -> {m1_wheel_link, m2_wheel_link, laser, rfans, imu_link} の TF を
    # publish。固定 joint は /tf_static になる。epos4_odometry が /joint_states を出すので
    # remap 不要。
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        output="screen",
    )

    # claude: RViz の起動はここでは行わない (nav2.launch.py / slam.launch.py に委譲)。

    # claude: HOKUYO 2D LiDAR driver。frame_id は rerobot.urdf の laser link 名と一致。
    # urg_node の executable 名は ROS 2 Jazzy では `urg_node_driver`。
    urg_node_node = Node(
        package="urg_node",
        executable="urg_node_driver",
        name="urg_node",
        condition=IfCondition(LaunchConfiguration("lidar_2d")),
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

    # claude: R-Fans 3D LiDAR driver。frame_id 等の静的 param は params.yaml
    # (rfans_driver セクション) から読み、device_ip / rps / model のみ launch 引数で
    # 上書きする (後勝ちで yaml の値を置き換え)。出力は PointCloud2 /sdk_could。
    rfans_node = Node(
        package="rfans_driver",
        executable="driver_node",
        name="rfans_driver",  # claude: params.yaml の key と一致必須
        condition=IfCondition(LaunchConfiguration("lidar_3d")),
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

    # claude: BNO086 IMU driver (自作ボード, USB CDC)。frame_id 既定 imu_link は
    # rerobot.urdf の imu_link と一致。/imu/data を publish する。
    # ⚠️ realsense_imu.launch.py と同時起動すると /imu/data が衝突する (どちらか一方)。
    imu_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("bno086_imu_driver"),
                "launch",
                "bno086.launch.py",
            )
        ),
        condition=IfCondition(LaunchConfiguration("imu")),
        launch_arguments={"port": LaunchConfiguration("imu_port")}.items(),
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
        lidar_2d_arg,
        lidar_3d_arg,
        imu_arg,
        ekf_arg,
        serial_port_arg,
        device_ip_arg,
        rps_arg,
        model_arg,
        imu_port_arg,
        bus_config,
        delayed_nodes,
        ekf_node,
        robot_state_publisher_node,
        urg_node_node,
        rfans_node,
        imu_include,
    ])
