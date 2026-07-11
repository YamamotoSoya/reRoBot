# claude: 3D LiDAR (R-Fans) 構成の Nav2 自律走行 launch。
#   nav2.launch.py (2D) の amcl を lidar_localization_ros2 (NDT スキャンマッチング)
#   に差し替えたもの。それ以外 (map_server / keepout / Nav2 サーバ群 / RViz) は
#   2D と同一構成。costmap の障害物入力は /scan のままなので、先に
#   rerobot_bringup_3d.launch.py (pointcloud_to_laserscan 込み, odom_tf は既定 true)
#   を起動しておくこと。
#
# 前提 (bringup_3d が供給):
#   - /sdk_could          (R-Fans PointCloud2) … NDT 自己位置推定の入力
#   - /scan               (pointcloud_to_laserscan) … costmap 障害物層の入力
#   - /odom + TF odom->base_link (epos4_odometry)
#   - TF base_link->rfans など (robot_state_publisher)
# 本 launch が追加で供給するもの:
#   - map_server / filter_mask_server / costmap_filter_info_server (2D と同じ)
#   - lidar_localization : map->odom TF (enable_map_odom_tf モード)
#   - Nav2 サーバ群 + RViz (nav2.rviz)
#
# 地図ディレクトリ構成 (map_dir 直下):
#   <map_dir>/lio_sam/GlobalMap.pcd         … LIO-SAM save_map の 3D 地図 (NDT 用)
#   <map_dir>/nav2/my_map.{yaml,pgm}        … pcd_to_gridmap で投影した 2D 地図
#   <map_dir>/keep_out/keep_out.{yaml,pgm}  … 進入禁止帯マスク (2D 運用と同じ)
#
# 起動後の操作は 2D と同じ: RViz "2D Pose Estimate" で初期姿勢 (/initialpose)
# → NDT が収束したら "Nav2 Goal"。
#
# claude: lidar_localization_node は managed (lifecycle) node で自己 activate
#   しないため、slam.launch.py と同じ launch 側 CONFIGURE→ACTIVATE パターンで
#   起動する (nav2 lifecycle_manager は bond 前提のため使わない)。
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    pkg_share = get_package_share_directory("rerobot_bringup")
    default_params = os.path.join(pkg_share, "config", "nav2_params.yaml")
    localization_params = os.path.join(pkg_share, "config", "lidar_localization.yaml")

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
        description="Nav2 パラメータファイルへのパス (2D と共通)。",
    )
    map_dir_arg = DeclareLaunchArgument(
        "map_dir",
        default_value="/workspace/maps/lio_sam/latest",
        description="地図一式 (lio_sam/, nav2/, keep_out/) を含む親ディレクトリ。",
    )

    map_yaml = PathJoinSubstitution([map_dir, "nav2", "my_map.yaml"])
    keepout_yaml = PathJoinSubstitution([map_dir, "keep_out", "keep_out.yaml"])
    pcd_map = PathJoinSubstitution([map_dir, "lio_sam", "GlobalMap.pcd"])

    cmd_vel_remap = ("/cmd_vel", "/robot_speed_cmd")

    # ========================================================================
    # 2D 投影地図 + keepout (nav2.launch.py と同一)
    # ========================================================================
    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time, "yaml_filename": map_yaml}],
    )

    filter_mask_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="filter_mask_server",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time, "yaml_filename": keepout_yaml}],
    )

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
            # claude: amcl は 3D 構成では使わない。自己位置は lidar_localization
            # (下の LifecycleNode, launch 側でライフサイクル管理) が担う
            "node_names": [
                "map_server",
                "filter_mask_server",
                "costmap_filter_info_server",
            ],
        }],
    )

    # ========================================================================
    # NDT 自己位置推定 (amcl の代替)。/sdk_could + PCD 地図 → map->odom TF
    # ========================================================================
    lidar_localization = LifecycleNode(
        package="lidar_localization_ros2",
        executable="lidar_localization_node",
        name="lidar_localization",
        namespace="",
        parameters=[
            localization_params,
            {
                "use_sim_time": use_sim_time,
                "map_path": pcd_map,
                # claude: Nav2 互換モード。map->base_link 直接ではなく、
                # epos4_odometry の odom->base_link と合成して map->odom を出す
                "enable_map_odom_tf": True,
            },
        ],
        remappings=[
            ("/cloud", "/sdk_could"),  # claude: R-Fans 生点群を直接入力
            ("/pcl_pose", "/localization/pose_with_covariance"),
        ],
        output="screen",
    )

    localization_configure = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(lidar_localization),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )

    localization_activate = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=lidar_localization,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(lidar_localization),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                )
            ],
        )
    )

    # ========================================================================
    # Nav2 サーバ群 (nav2.launch.py と同一)
    # ========================================================================
    controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
        remappings=[cmd_vel_remap],
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
        remappings=[cmd_vel_remap],
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
            "autostart": True,
            "node_names": [
                "controller_server",
                "planner_server",
                "behavior_server",
                "bt_navigator",
            ],
        }],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", os.path.join(pkg_share, "rviz", "nav2.rviz")],
        output="screen",
    )

    return LaunchDescription([
        use_sim_time_arg,
        params_file_arg,
        map_dir_arg,
        map_server,
        filter_mask_server,
        costmap_filter_info_server,
        lifecycle_manager_localization,
        lidar_localization,
        localization_configure,
        localization_activate,
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        lifecycle_manager_navigation,
        rviz_node,
    ])
