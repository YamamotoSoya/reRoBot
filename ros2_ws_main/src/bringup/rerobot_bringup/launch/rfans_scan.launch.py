# claude: R-Fans 3D 点群 → 2D LaserScan (/scan) 変換 (2026-08-17 追加)。
# GLIM 由来の 3D 地図で Nav2 を走らせる「最短案」の自己位置入力を作る:
#   /rfans_driver/rfans_points → pointcloud_to_laserscan → /scan → AMCL + costmap
#
# 仕組み: ① 点群を target_frame (base_link) へ TF 変換 (URDF が取付角を吸収するので
# rfans プリセット tilted45/tilted15/flat のどれでも設定は共通)、② base_link 基準の
# 高さ帯 [min_height, max_height] の点だけ残す、③ 方位角ビンごとに最短距離を採用して
# LaserScan 化。
#
# ⚠️ 整合条件: min/max_height は 3D 地図→2D 変換 (pointcloud_to_2dmap) の高さクリップ帯
#   「床 +0.3〜1.5 m」と同じにすること。帯がずれると AMCL が「地図に無い壁」を観測して
#   自己位置が暴れる (docs/features/2026-08-17_glim_map_to_nav2.md 参照)。
# ⚠️ range_max=30.0 は nav2_params.yaml の amcl laser_max_range=30.0 に合わせた値。
#   変えるなら両方セットで。
# ⚠️ /scan は HOKUYO urg_node と同名トピック。同時起動しないこと —
#   bringup は lidar_2d:=false lidar_3d:=true で使う。
#
# 前提: rerobot_bringup (lidar_3d:=true) が先に上がっており、
#   /rfans_driver/rfans_points と TF base_link->rfans が流れていること。
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    min_height = LaunchConfiguration("min_height")
    max_height = LaunchConfiguration("max_height")
    range_max = LaunchConfiguration("range_max")

    min_height_arg = DeclareLaunchArgument(
        "min_height",
        default_value="0.3",
        description="採用する点の高さ帯の下限 [m] (base_link 基準。地図スライス帯と揃える)。",
    )
    max_height_arg = DeclareLaunchArgument(
        "max_height",
        default_value="1.5",
        description="採用する点の高さ帯の上限 [m] (base_link 基準。地図スライス帯と揃える)。",
    )
    range_max_arg = DeclareLaunchArgument(
        "range_max",
        default_value="30.0",
        description="スキャンの最大距離 [m]。amcl の laser_max_range と一致させる。",
    )

    pointcloud_to_laserscan = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="rfans_to_scan",
        output="screen",
        remappings=[
            ("cloud_in", "/rfans_driver/rfans_points"),
            ("scan", "/scan"),
        ],
        parameters=[{
            "target_frame": "base_link",   # 取付角の吸収は URDF (TF) に任せる
            "transform_tolerance": 0.1,
            "min_height": min_height,
            "max_height": max_height,
            "angle_min": -3.14159265,      # 全周 (R-Fans は 360°)
            "angle_max": 3.14159265,
            # R-Fans-16 の方位分解能 ~0.19°/step (30048 点/回転 ÷ 16 ビーム) に合わせる
            "angle_increment": 0.0035,
            "scan_time": 0.1,              # 10 Hz (1 回転 1 メッセージ)
            "range_min": 0.5,              # 車体・マスト自身の映り込みを除外
            "range_max": range_max,
            "use_inf": True,
        }],
    )

    return LaunchDescription([
        min_height_arg,
        max_height_arg,
        range_max_arg,
        pointcloud_to_laserscan,
    ])
