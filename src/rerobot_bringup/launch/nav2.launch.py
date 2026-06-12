# claude: 保存済み地図 (.pgm/.yaml) を再生する Nav2 「最小構成」launch。
# slam_toolbox には依存しない。map_server が地図を配信し、amcl が map->odom TF を
# 供給する古典的なローカリゼーション + ナビ構成 + keepout (進入禁止帯) フィルタ。
#
# 前提: rerobot_bringup.launch.py が先に上がっており、以下が流れていること。
#   - /scan            (HOKUYO urg_node)
#   - /odom + TF odom->base_link (epos4_odometry)
#   - TF base_link->laser など (robot_state_publisher)
# 本 launch が追加で供給するもの:
#   - map_server                : /map (保存済み占有格子地図)
#   - amcl                      : map->odom TF (自己位置推定)
#   - filter_mask_server        : /keepout_filter_mask (進入禁止帯マスク)
#   - costmap_filter_info_server: /costmap_filter_info (マスク値→コスト変換則)
#   - Nav2 サーバ群              : controller / planner / behavior / bt_navigator
#
# 地図ディレクトリ構成 (map_dir 直下):
#   <map_dir>/nav2/my_map.{yaml,pgm}        … 本体地図
#   <map_dir>/keep_out/keep_out.{yaml,pgm}  … keepout マスク
#
# 速度司令の配線: Nav2 既定の /cmd_vel を本機の /robot_speed_cmd (素の Twist) へ
# リマップする。Twist 化は params 側の enable_stamped_cmd_vel: false で行う。
#
# 起動後、RViz の "2D Pose Estimate" で初期姿勢を一度与えると amcl が収束する。
# その後 "Nav2 Goal" で目標を指定する。
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("rerobot_bringup")
    default_params = os.path.join(pkg_share, "config", "nav2_params.yaml")

    # ---- launch 引数 ----
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    map_dir = LaunchConfiguration("map_dir")

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="シミュレーション時刻 (/clock) を使うか。実機は false。",
    )
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_params,
        description="Nav2 パラメータファイルへのパス。",
    )
    # 地図一式の親ディレクトリ。別の計測日に切り替えるときはここだけ差し替える。
    map_dir_arg = DeclareLaunchArgument(
        "map_dir",
        default_value="/workspace/maps/slam_toolbox/2026_6_9__22:00",
        description="地図一式 (nav2/, keep_out/) を含む親ディレクトリ。",
    )

    # map_dir から本体地図と keepout マスクの yaml パスを組み立てる。
    map_yaml = PathJoinSubstitution([map_dir, "nav2", "my_map.yaml"])
    keepout_yaml = PathJoinSubstitution([map_dir, "keep_out", "keep_out.yaml"])

    # ---- 速度司令リマップ: Nav2 の /cmd_vel を本機の /robot_speed_cmd へ ----
    cmd_vel_remap = ("/cmd_vel", "/robot_speed_cmd")

    # ========================================================================
    # ローカリゼーション + keepout フィルタ。
    #   lifecycle_manager_localization が autostart=true でまとめて起動する。
    # ========================================================================
    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        # launch の map_dir からの yaml_filename を params 既定より優先。
        parameters=[params_file, {"use_sim_time": use_sim_time, "yaml_filename": map_yaml}],
    )

    amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
    )

    # keepout マスク配信サーバ (map_server 実体を別名・別トピックで起動)。
    filter_mask_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="filter_mask_server",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time, "yaml_filename": keepout_yaml}],
    )

    # マスク値→コスト変換則の配信サーバ。
    costmap_filter_info_server = Node(
        package="nav2_map_server",
        executable="costmap_filter_info_server",
        name="costmap_filter_info_server",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
    )

    lifecycle_manager_localization = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "autostart": True,
            # filter 系は costmap が購読する前に上げておく。
            "node_names": [
                "map_server",
                "amcl",
                "filter_mask_server",
                "costmap_filter_info_server",
            ],
        }],
    )

    # ========================================================================
    # Nav2 サーバ群 (走行に必要な最小セット)。
    #   lifecycle_manager_navigation が autostart=true でまとめて起動する。
    # ========================================================================
    controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
        remappings=[cmd_vel_remap],  # /cmd_vel -> /robot_speed_cmd
    )

    planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
    )

    behavior_server = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
        remappings=[cmd_vel_remap],  # 復帰行動の速度司令も /robot_speed_cmd へ
    )

    bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
    )

    lifecycle_manager_navigation = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "autostart": True,  # 起動時に自動で全ノードを activate
            "node_names": [
                "controller_server",
                "planner_server",
                "behavior_server",
                "bt_navigator",
            ],
        }],
    )

    return LaunchDescription([
        use_sim_time_arg,
        params_file_arg,
        map_dir_arg,
        map_server,
        amcl,
        filter_mask_server,
        costmap_filter_info_server,
        lifecycle_manager_localization,
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        lifecycle_manager_navigation,
    ])
