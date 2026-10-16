# claude: 統合 bringup (2026-08-10)。実機には 2D LiDAR (UTM-30LX) と 3D LiDAR (R-Fans-16)
#   が両方常設 + BNO086 IMU 搭載予定のため、旧 rerobot_bringup_{2d,3d}.launch.py を
#   1 本に統合し、起動するセンサドライバを boolean 引数で選ぶ方式にした。
#     lidar_2d:=true/false  ... urg_node (HOKUYO, /scan)
#     lidar_3d:=true/false  ... surestar_rfans_ros2 (R-Fans, /rfans_driver/rfans_points)
#     imu:=true/false       ... bno086_imu_driver (/imu/data)
#     imu_wit:=true/false   ... witmotion_ros (WT901C, /imu_wit/data)。imu:=true のときだけ有効
#                               (既定 true = BNO086 と並走が標準構成、2026-10-01)
#     ekf:=true/false       ... robot_localization EKF (車輪 odom + IMU 融合, 2026-08-11)。
#                               true で /odometry/filtered + TF odom->base_link を EKF が
#                               担当し、epos4_odometry の TF は自動オフ。imu:=true と併用。
#   起動の流れ (2026-10-01 に表示を整理): 冒頭に構成バナー → [1/3] CANopen + センサ →
#   [2/3] 5 s 後に controller/odometry → [3/3] check_delay 秒後に bringup_check が
#   EPOS4 状態と各トピック周波数を表で表示 (scripts/bringup_check.py)。
#   URDF は rerobot.urdf 1 本 (laser / rfans / imu_link を常に含む — 使わないセンサの
#   静的 TF が出ていても無害)。params は config/params.yaml 1 本。
# claude: 2026-10-08 「全部入り」の robot bringup に変更 (構成別ラッパは不要になった)。
#   既定で全センサ + EKF + rfans_scan (3D→2D) を起動する。外すのは故障・未接続・CPU を空けたいときだけ。
#   LaserScan は裸の /scan を誰も出さない規約 (urg と 3D→2D の衝突を名前で避ける。将来 urg を増やしても同じ):
#     urg (前)            → /urg_front/scan   (将来 /urg_left/scan, /urg_right/scan を足す)
#     R-Fans 3D→2D 最近点 → /rfans/scan       (costmap 用。rfans_scan:=true 既定)
#     R-Fans 3D→2D 全点   → /rfans/scan_all   (AMCL 全点版。rfans_scan_all:=true のときだけ — 重いので既定 off)
#   使う側 (costmap / AMCL / slam_toolbox) がどれを読むかを明示する。旧ラッパ rerobot_bringup_*.launch.py は削除候補。
# claude: 2026-10-16 速度指令の調停器 twist_mux を常駐させた (config/twist_mux.yaml)。
#   joy_teleop (/joy_vel, 優先 100) と Nav2 (/cmd_vel, 優先 10) は自分のトピックに出すだけで、
#   /robot_speed_cmd の publisher は twist_mux の 1 つだけになる。LB を離せば 0.5 s で Nav2 側に落ちる。
#   boolean で外せるようにしていないのは、外すと joy の出力先 /joy_vel を誰も受けなくなるため。
#   epos4_teleop (キーボード) は台上デバッグ専用なので従来どおり /robot_speed_cmd に直接出す。
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, LogInfo,
                            OpaqueFunction, TimerAction)
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
        description="Start surestar_rfans_ros2 (R-Fans-16, /rfans_driver/rfans_points)")
    imu_arg = DeclareLaunchArgument(
        "imu", default_value="true",
        description="Start bno086_imu_driver (/imu/data)")
    # claude_imu_wit: 比較用 IMU WT901C を BNO086 と並走させる (2026-10-01 にデフォルト構成化)。
    # imu:=true と AND で効く — IMU なし構成 (rerobot_bringup_{2d,3d}.launch.py) では起動しない。
    imu_wit_arg = DeclareLaunchArgument(
        "imu_wit", default_value="true",
        description="Start witmotion_ros for WT901C (/imu_wit/data); effective only with imu:=true")
    # claude_imu_wit: /dev 配下のデバイス名 (witmotion_ros は /dev/ を前置して開く)。
    # udev rule (tools/99-wt901.rules) 未導入なら ttyUSB0 等を渡す。
    imu_wit_port_arg = DeclareLaunchArgument(
        "imu_wit_port", default_value="ttyUSB-wt901",
        description="WT901C serial device name under /dev (udev symlink or ttyUSBn)")
    # claude_ekf: 車輪 odom + IMU の EKF 融合 (robot_localization)。true にすると
    # ekf_node が /odometry/filtered と TF odom->base_link を出し、epos4_odometry の
    # publish_tf を自動で false にする (TF 二重配信の防止。/odom topic 自体は残る)。
    ekf_arg = DeclareLaunchArgument(
        "ekf", default_value="true",  # claude: 2026-10-08 既定 false → true (全部入り化)
        description="Fuse wheel odom + IMU with robot_localization EKF")

    # claude: 2026-10-08 R-Fans 点群 → 2D LaserScan (rfans_scan.launch.py) を同梱。lidar_3d と AND で効く。
    rfans_scan_arg = DeclareLaunchArgument(
        "rfans_scan", default_value="true",
        description="R-Fans 点群 → /rfans/scan (最近点, costmap 用) を出す。lidar_3d:=true のときだけ有効")
    rfans_scan_all_arg = DeclareLaunchArgument(
        "rfans_scan_all", default_value="false",
        description="/rfans/scan_all (全点 3.9 万本, AMCL 全点版 nav_amcl_scanall 用) も出す。重いので既定 off")
    # ⚠ 整合条件: scan_min/max_height = costmap が障害物にする高さ (ロボットがぶつかる帯)、
    #   scan_all_min/max_height = 自己位置推定用地図 (loc_map_yaml) を作った帯。
    # claude: 2026-10-15 2 本の帯を分離。従来は scan_all_* が scan_* を引き継いでいたため、scan_max_height:=100 だけ
    #   渡すと costmap 用 /rfans/scan まで 0.3〜100 になっていた (10-14 走行)。既定を costmap 0.3〜2.0 /
    #   AMCL 0.3〜100.0 (= maps/.../0.3-100.0_r100 の帯) に固定し、引き継ぎはやめる。
    scan_min_height_arg = DeclareLaunchArgument(
        "scan_min_height", default_value="0.3",
        description="/rfans/scan (costmap 用) の高さ帯の下限 [m] (base_link 基準)")
    scan_max_height_arg = DeclareLaunchArgument(
        "scan_max_height", default_value="2.0",
        description="/rfans/scan (costmap 用) の高さ帯の上限 [m]。ロボットがぶつかる高さまで (屋根・庇は入れない)")
    scan_all_min_height_arg = DeclareLaunchArgument(
        "scan_all_min_height", default_value="0.3",
        description="/rfans/scan_all (AMCL 用) の高さ帯の下限 [m]。loc_map_yaml の地図の帯と揃える")
    scan_all_max_height_arg = DeclareLaunchArgument(
        "scan_all_max_height", default_value="100.0",
        description="/rfans/scan_all (AMCL 用) の高さ帯の上限 [m]。loc_map_yaml の地図の帯と揃える (0.3-100.0_r100 なら 100.0)")

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
    # claude_imu_rate: BNO086 の報告レート [Hz] (2026-09-13)。基板 firmware の実用上限は
    # 200 Hz (README 実測: 200 Hz 要求→192 Hz・全項目揃い 99.7 %。250 Hz 以上は SHTP 取得が
    # 飽和して逆に落ちる)。EKF は 30 Hz で間引くので 100 のまま、GLIM (LIO) の deskew 用に
    # glim3d.sh が 200 を渡す。
    # claude: 2026-09-24 既定を 100 → 200 に変更 (BNO086 常設・GLIM 用 bag が主用途)。
    imu_rate_arg = DeclareLaunchArgument(
        "imu_rate", default_value="200.0",
        description="BNO086 IMU report rate [Hz] (firmware practical max 200)")

    # claude_check: 起動結果サマリ (bringup_check) を出すまでの待ち [s]。EPOS4 enable
    # (5 s TimerAction + 数 s) と R-Fans の回転安定を待つ。
    check_delay_arg = DeclareLaunchArgument(
        "check_delay", default_value="15.0",
        description="Seconds before bringup_check prints the startup summary")

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

    # claude: 2026-10-16 速度指令の調停器。joy (/joy_vel) > Nav2 (/cmd_vel) を優先度で選び /robot_speed_cmd へ素通し。
    #   出力トピック名は twist_mux 固定の cmd_vel_out なので remap で本機の名前にする。設定は config/twist_mux.yaml。
    twist_mux_node = Node(
        package="twist_mux",
        executable="twist_mux",
        name="twist_mux",
        parameters=[os.path.join(pkg_share, "config", "twist_mux.yaml")],
        remappings=[("/cmd_vel_out", "/robot_speed_cmd")],
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
            # claude: 2026-10-15 視野を ±90° → ±70° に。10-14 bag の collision ahead 50 件中 37 件は UTM が
            #   自車の右前 (base 系 −67°, 0.33〜0.39 m = laser から 75〜81°) に見る配線らしき点が footprint の縁
            #   (0.36 m) に乗ったもの。±70° で車輪前縁 (77°) ごと視野から外す。真横の壁は R-Fans (0.5 m 以上,
            #   z≥0.3) と、前方で見たときの costmap のマークが担う。
            "angle_min": -1.2217,
            "angle_max": 1.2217,
        }],
        remappings=[("scan", "/urg_front/scan")],  # claude: 2026-10-08 裸の /scan を出さない規約
        output="screen",
    )

    # claude: 2026-10-08 R-Fans 点群 → /rfans/scan (+ /rfans/scan_all)。scoped GroupAction で包み、
    #   渡した引数 (min_height 等) が後続 include に漏れないようにする。
    rfans_scan_include = GroupAction([IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, "launch", "tools", "rfans_scan.launch.py")),  # claude: 2026-10-09 tools/ へ移動に追従
        launch_arguments={
            "min_height": LaunchConfiguration("scan_min_height"),
            "max_height": LaunchConfiguration("scan_max_height"),
            "amcl_min_height": LaunchConfiguration("scan_all_min_height"),
            "amcl_max_height": LaunchConfiguration("scan_all_max_height"),
            "range_max": "150.0",
            "allpoints": LaunchConfiguration("rfans_scan_all"),
            "farthest": "false",
        }.items(),
    )], scoped=True, condition=IfCondition(PythonExpression([
        "'", LaunchConfiguration("lidar_3d"), "'.lower() == 'true' and '",
        LaunchConfiguration("rfans_scan"), "'.lower() == 'true'"])))

    # claude: R-Fans 3D LiDAR driver (2026-08-14 に surestar_rfans_ros2 へ刷新 —
    #   経緯と新旧比較は docs/issue/2026-08-14_rfans_driver_renewal.md)。
    #   公式オープンソース実装の 2 ノード構成: driver_node (UDP→RfansPacket) +
    #   calculation_node (→PointCloud2 /rfans_driver/rfans_points, frame_id rfans)。
    #   旧 /sdk_could (typo topic) と LD_PRELOAD は刷新で廃止。静的 param は
    #   params.yaml (rfans_driver / rfans_calculation セクション) から読み、
    #   device_ip / rps / model のみ launch 引数で上書き (後勝ち)。
    rfans_node = Node(
        package="surestar_rfans_ros2",
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
        output="screen",
    )

    # claude: 新ドライバの点群組み立てノード (rfans_node とペア、lidar_3d 連動)
    rfans_calc_node = Node(
        package="surestar_rfans_ros2",
        executable="calculation_node",
        name="rfans_calculation",  # claude: params.yaml の key と一致必須
        condition=IfCondition(LaunchConfiguration("lidar_3d")),
        parameters=[
            params_file,
            {
                "device_ip": LaunchConfiguration("device_ip"),
                "rps": ParameterValue(LaunchConfiguration("rps"), value_type=int),
                "model": LaunchConfiguration("model"),
            },
        ],
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
        launch_arguments={
            "port": LaunchConfiguration("imu_port"),
            # claude_auto_tare: ドライバ既定 yaml は auto_tare: all で、accel/gyro が起動時姿勢の
            # 座標に回される (2026-09-24 発覚、docs/issue/2026-09-24_bno086_auto_tare_rotates_motion_outputs.md)。
            # off にした本リポの yaml を params_file で渡す。
            "params_file": os.path.join(
                get_package_share_directory("rerobot_bringup"), "config", "bno086.yaml"),
            # claude_imu_rate: 基板 launch は imu_rate_hz を double で declare するため、
            # "200" (整数) のまま渡すと rclpy が型不一致で落ちる。float 文字列に強制する。
            "imu_rate_hz": PythonExpression(
                ["str(float('", LaunchConfiguration("imu_rate"), "'))"]),
        }.items(),
    )

    # claude_imu_wit: WT901C (witmotion_ros)。topic /imu_wit/data・frame_id imu_wit_link は
    # config/wt901.yaml、TF は rerobot.urdf の imu_wit_joint。/imu/data とは別名なので衝突しない。
    imu_wit_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "tools", "wt901_imu.launch.py")),  # claude: 2026-10-09 tools/ へ移動に追従
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration("imu"), "'.lower() == 'true' and '",
            LaunchConfiguration("imu_wit"), "'.lower() == 'true'"])),
        # claude_imu_wit: params_file は明示必須 — include の launch 引数は後続 include に漏れるため、
        # 省略すると直前の BNO086 include の params_file (bno086.yaml) を拾ってしまう (2026-10-01 実測)。
        launch_arguments={
            "port": LaunchConfiguration("imu_wit_port"),
            "params_file": os.path.join(pkg_share, "config", "wt901.yaml"),
        }.items(),
    )

    # claude_check: 解決済みの引数から「何を起動するか」のバナーを組む (launch 冒頭に表示)
    def _banner(context):
        def on(name):
            return LaunchConfiguration(name).perform(context).lower() == "true"
        def cfg(name):
            return LaunchConfiguration(name).perform(context)
        mark = lambda b: "起動 " if b else "----"  # noqa: E731
        lines = [
            "",
            "=" * 64,
            " reRoBot bringup — 起動構成",
            "=" * 64,
            f"  [{mark(True)}] EPOS4 x2 (CANopen can0) + controller / odometry",
            f"  [{mark(True)}] twist_mux  /joy_vel (優先) + /cmd_vel (Nav2) → /robot_speed_cmd",
            f"  [{mark(on('lidar_2d'))}] 2D LiDAR UTM-30LX      port={cfg('serial_port')}  → /urg_front/scan",
            f"  [{mark(on('lidar_3d'))}] 3D LiDAR R-Fans-16     ip={cfg('device_ip')} rps={cfg('rps')}",
            f"  [{mark(on('lidar_3d') and on('rfans_scan'))}]   └ 3D→2D /rfans/scan    帯 {cfg('scan_min_height')}〜{cfg('scan_max_height')} m",
            f"  [{mark(on('lidar_3d') and on('rfans_scan') and on('rfans_scan_all'))}]   └ 3D→2D /rfans/scan_all 帯 {cfg('scan_all_min_height')}〜{cfg('scan_all_max_height')} m",
            f"  [{mark(on('imu'))}] IMU BNO086             port={cfg('imu_port')} rate={cfg('imu_rate')} Hz",
            f"  [{mark(on('imu') and on('imu_wit'))}] IMU WT901C             port=/dev/{cfg('imu_wit_port')}",
            f"  [{mark(on('ekf'))}] EKF (robot_localization)",
            "=" * 64,
            f" [1/3] CANopen バスとセンサドライバを起動します",
            f" [2/3] 5 秒後に epos4_controller / epos4_odometry を起動 (EPOS4 enable)",
            f" [3/3] {cfg('check_delay')} 秒後に起動結果の一覧を表示します",
            "=" * 64,
        ]
        return [LogInfo(msg="\n".join(lines))]

    # claude_check: 起動結果サマリ。1 回判定して表を出したら終了する (launch は止まらない)。
    # claude: 2026-10-16 OpaqueFunction で引数を「今」解決してから TimerAction を作る。
    #   以前は TimerAction(period=LaunchConfiguration("check_delay")) + ParameterValue(LaunchConfiguration(...)) で、
    #   タイマーの coroutine が走る時点で引数を引いていた。auto_run.launch.py のように GroupAction(scoped=True) で
    #   include されると、include 直後にスコープが閉じて launch configuration が消えるため
    #   "launch configuration 'check_delay' does not exist" で bringup_check が一度も起動せず、未回収の
    #   Task 例外が launch に残って Ctrl-C でも launch が終了しなくなっていた (10-16 実測)。
    #   → 値をすべて Python の素の型にしてから渡す (タイマー発火時に context を参照しない)。
    def _delayed_check(context):
        cfg = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
        on = lambda name: cfg(name).strip().lower() == "true"  # noqa: E731
        bringup_check_node = Node(
            package="rerobot_bringup",
            executable="bringup_check.py",
            name="bringup_check",
            parameters=[{
                "lidar_2d": on("lidar_2d"),
                "lidar_3d": on("lidar_3d"),
                "rfans_scan": on("rfans_scan"),
                "rfans_scan_all": on("rfans_scan_all"),
                "imu": on("imu"),
                "imu_wit": on("imu_wit"),
                "ekf": on("ekf"),
                "imu_rate": float(cfg("imu_rate")),
                "rps": int(cfg("rps")),
            }],
            output="screen",
            emulate_tty=True,  # claude: 色付き表示のため
        )
        return [TimerAction(
            period=float(cfg("check_delay")),
            actions=[LogInfo(msg="[3/3] 起動チェックを開始します"), bringup_check_node],
        )]

    delayed_check = OpaqueFunction(function=_delayed_check)

    # Delay controller/odometry so the ros2_canopen device_manager has time to
    # advertise /motor*/cia402_device_*/{init,enable,cyclic_velocity_mode}.
    # Without this, the controller's constructor-time wait_for_service(1s) calls
    # race the bus_config launch and silently fail, leaving the EPOS4s disabled.
    delayed_nodes = TimerAction(
        period=5.0,
        actions=[LogInfo(msg="[2/3] epos4_controller / epos4_odometry を起動します"),
                 epos4_controller_node, epos4_odometry_node],
    )

    return LaunchDescription([
        lidar_2d_arg,
        lidar_3d_arg,
        imu_arg,
        imu_wit_arg,
        ekf_arg,
        rfans_scan_arg,           # claude: 2026-10-08
        rfans_scan_all_arg,
        scan_min_height_arg,
        scan_max_height_arg,
        scan_all_min_height_arg,
        scan_all_max_height_arg,
        serial_port_arg,
        device_ip_arg,
        rps_arg,
        model_arg,
        imu_port_arg,
        imu_rate_arg,
        imu_wit_port_arg,
        check_delay_arg,
        OpaqueFunction(function=_banner),
        bus_config,
        delayed_nodes,
        twist_mux_node,  # claude: 2026-10-16
        ekf_node,
        robot_state_publisher_node,
        urg_node_node,
        rfans_node,
        rfans_calc_node,
        rfans_scan_include,  # claude: 2026-10-08
        imu_include,
        imu_wit_include,
        delayed_check,
    ])
