# claude: 3D LiDAR (R-Fans / rfans_driver) 構成用の composite bringup。
# claude_opt: 共通部 (bus_config + epos4_controller + epos4_odometry +
#   robot_state_publisher) は rerobot_bringup_common.launch.py に抽出済みで、
#   ここでは params/urdf の 3D 用ファイル指定と rfans_driver の追加のみを行う。
#   rfans の param は config/params_3d.yaml の rfans_driver セクションから読む。
#   device_ip / rps / model は launch 引数で yaml の値を上書き可能。
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
# claude: launch 引数は文字列で解決されるため、int 等の型付き param は ParameterValue で包む
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory("rerobot_bringup")
    params_file = os.path.join(pkg_share, "config", "params_3d.yaml")

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

    common = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "rerobot_bringup_common.launch.py")
        ),
        launch_arguments={
            "params_file": params_file,
            "urdf_file": os.path.join(pkg_share, "urdf", "rerobot_3d.urdf"),
        }.items(),
    )

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
        output="screen",
        # claude_robust: UDP 受信スレッドの異常等でプロセスが死んだら自動再起動する
        # (urg_node 側と同じ方針)。
        respawn=True,
        respawn_delay=2.0,
    )

    return LaunchDescription([
        device_ip_arg,
        rps_arg,
        model_arg,
        common,
        rfans_node,
    ])
