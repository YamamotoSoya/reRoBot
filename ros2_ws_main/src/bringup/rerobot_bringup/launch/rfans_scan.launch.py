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
# claude: 2026-10-01 追加 — allpoints:=true で「全点版」/scan_all も同時に出す (AMCL 専用)。
#   方位ビンを R-Fans の方位刻み (0.00016 rad ≈ 0.009°) まで細かくし、同じ方位ビンで最近点 1 点に
#   絞らずほぼ全点に自分の席を持たせる。/scan (0.0035 rad、最近点) は costmap 用にそのまま残す
#   (/scan_all を costmap に入れると 3.9 万本のレイトレースで重い)。AMCL 側は nav2.launch.py amcl_scan:=all。
#   比較実験の経緯: docs/issue/2026-10-01_amcl_hedge_corridor_scan_reduction.md
#
# 前提: rerobot_bringup (lidar_3d:=true) が先に上がっており、
#   /rfans_driver/rfans_points と TF base_link->rfans が流れていること。
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
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
        default_value="6.0",
        description="採用する点の高さ帯の上限 [m] (base_link 基準。地図スライス帯と揃える)。",
    )
    # claude: 2026-10-04 AMCL 専用スキャン (/scan_all, /scan_far) の高さ帯を /scan と分ける。
    #   AMCL 用地図 (屋根・軒・建物上部込み、例 0.3〜3.0 m) に合わせて上限を上げても、
    #   costmap 用の /scan は従来帯のまま (屋根を障害物にしない)。既定は min/max_height を引き継ぐ。
    #   ⚠ 整合条件: amcl_min/max_height は nav2.launch.py loc_map_yaml の地図の帯と揃える。
    amcl_min_height = LaunchConfiguration("amcl_min_height")
    amcl_max_height = LaunchConfiguration("amcl_max_height")
    amcl_min_height_arg = DeclareLaunchArgument(
        "amcl_min_height",
        default_value=min_height,
        description="/scan_all・/scan_far の高さ帯の下限 [m] (既定 = min_height)。AMCL 用地図の帯と揃える。",
    )
    amcl_max_height_arg = DeclareLaunchArgument(
        "amcl_max_height",
        default_value=max_height,
        description="/scan_all・/scan_far の高さ帯の上限 [m] (既定 = max_height)。AMCL 用地図の帯と揃える。",
    )
    range_max_arg = DeclareLaunchArgument(
        "range_max",
        default_value="150.0",
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

    # claude: 2026-10-01 全点版 (AMCL 専用、allpoints:=true のときだけ)
    allpoints = LaunchConfiguration("allpoints")
    allpoints_arg = DeclareLaunchArgument(
        "allpoints",
        default_value="false",
        description="true で /scan_all (方位ビン 0.00016 rad = ほぼ全点) も出す。AMCL 側は nav2.launch.py amcl_scan:=all",
    )
    pointcloud_to_laserscan_all = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="rfans_to_scan_all",
        output="screen",
        remappings=[
            ("cloud_in", "/rfans_driver/rfans_points"),
            ("scan", "/scan_all"),
        ],
        parameters=[{
            "target_frame": "base_link",
            "transform_tolerance": 0.1,
            "min_height": amcl_min_height,  # claude: 2026-10-04 AMCL 専用帯 (既定は /scan と共通)
            "max_height": amcl_max_height,
            "angle_min": -3.14159265,
            "angle_max": 3.14159265,
            "angle_increment": 0.00016,    # R-Fans の方位刻み (bag 実測 0.009°)。39,270 席
            "scan_time": 0.1,
            "range_min": 0.5,
            "range_max": range_max,
            "use_inf": True,
        }],
        condition=IfCondition(allpoints),
    )

    # claude: 2026-10-01 最遠点版 (AMCL 専用、farthest:=true のときだけ)。pointcloud_to_laserscan は
    #   ビン内最近点しか選べないため自作ノード rfans_scan_modes (mode=farthest) で作る。
    #   ⚠ 並走比較では平常時に約 1.2 m ずれた (地図は植え込みも壁として描くので、奥の点だけでは合わない)。
    farthest = LaunchConfiguration("farthest")
    farthest_arg = DeclareLaunchArgument(
        "farthest",
        default_value="false",
        description="true で /scan_far (方位ビン 0.0035 rad 内の最遠点) も出す。AMCL 側は nav2.launch.py amcl_scan:=far",
    )
    scan_far = Node(
        package="rfans_scan_modes",
        executable="scan_modes",
        name="rfans_to_scan_far",
        output="screen",
        parameters=[{
            "cloud_topic": "/rfans_driver/rfans_points",
            "scan_topic": "/scan_far",
            "target_frame": "base_link",
            "mode": "farthest",
            "min_height": amcl_min_height,  # claude: 2026-10-04 AMCL 専用帯
            "max_height": amcl_max_height,
            "angle_increment": 0.0035,
            "range_min": 0.5,
            "range_max": range_max,
        }],
        condition=IfCondition(farthest),
    )

    return LaunchDescription([
        min_height_arg,
        max_height_arg,
        amcl_min_height_arg,  # claude
        amcl_max_height_arg,  # claude
        range_max_arg,
        allpoints_arg,  # claude
        farthest_arg,   # claude
        pointcloud_to_laserscan,
        pointcloud_to_laserscan_all,  # claude
        scan_far,       # claude
    ])
