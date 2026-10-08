# claude: 保存済み地図 (.pgm/.yaml) を再生する Nav2 「最小構成」launch。
# slam_toolbox には依存しない。map_server が地図を配信し、amcl が map->odom TF を
# 供給する古典的なローカリゼーション + ナビ構成 + keepout (進入禁止帯) フィルタ。
#
# claude: 2026-10-08 中身を部品 (launch/parts/) に分割し、本ファイルは互換用の薄い構成になった。
#   新規の運用は構成別 launch を使う (方式・入力は構成ファイル名で固定):
#     nav_amcl.launch.py          = 本ファイル既定 (amcl_scan:=default)
#     nav_amcl_scanall.launch.py  = 本ファイル amcl_scan:=all [loc_map_yaml:=...]
#     nav_emcl2.launch.py / nav_lidar3d.launch.py (仮置き)
#   本ファイルは引数 (amcl_scan / loc_map_yaml / map_dir …) をすべて従来どおり受け付ける。
#   claude: 2026-10-08 裸の /scan 廃止に伴い、amcl_scan:=default の入力は scan_topic (既定 /urg_front/scan —
#   nav2d.sh = urg 構成向け)。GLIM 地図なら scan_topic:=/rfans/scan。all / far は /rfans/scan_all / _far に読み替える。
#   ⚠ AMCL のパラメータは nav2_params.yaml から config/localization/amcl.yaml へ移した。
#     独自の params_file を渡していた場合、amcl 節は amcl_params_file:= で渡す。
#
# 前提: rerobot_bringup (robot_2d など) が先に上がっており、以下が流れていること。
#   - /scan, TF odom->base_link (EKF), TF base_link->laser など (robot_state_publisher)
#
# 地図の渡し方は 2 通り:
#   a) map_dir 規約 — <map_dir>/nav2/map.{yaml,pgm} + <map_dir>/keep_out/keep_out.{yaml,pgm}
#   b) map_yaml:=<yaml へのフルパス> で直接指定。keepout マスク未作成の地図は use_keepout:=false
#
# 自己位置推定用と経路計画用の地図を分ける (loc_map_yaml、2026-10-04):
#   loc_map_yaml:=<yaml> で map_server_loc が /map_loc を配信し、AMCL だけがそれを読む。
#   例: loc_map_yaml:=<map_dir>/loc_0.3-3.0/map.yaml amcl_scan:=all
#       (rfans_scan 側: allpoints:=true amcl_min_height:=0.3 amcl_max_height:=3.0)
#
# 起動後、RViz の "2D Pose Estimate" で初期姿勢を一度与えると amcl が収束する。
# その後 "Nav2 Goal" で目標を指定する。
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression


def generate_launch_description():
    pkg_share = get_package_share_directory("rerobot_bringup")
    parts = os.path.join(pkg_share, "launch", "parts")

    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    amcl_params_file = LaunchConfiguration("amcl_params_file")
    map_dir = LaunchConfiguration("map_dir")
    map_yaml = LaunchConfiguration("map_yaml")
    keepout_yaml = LaunchConfiguration("keepout_yaml")
    use_keepout = LaunchConfiguration("use_keepout")
    amcl_scan = LaunchConfiguration("amcl_scan")
    loc_map_yaml = LaunchConfiguration("loc_map_yaml")
    scan_topic = LaunchConfiguration("scan_topic")
    # claude: 2026-10-08 amcl_scan (default/all/far) → 実トピック名と max_beams へ読み替え
    amcl_scan_topic = PythonExpression([
        "{'all': '/rfans/scan_all', 'far': '/rfans/scan_far'}.get('", amcl_scan, "', '", scan_topic, "')"])
    amcl_max_beams = PythonExpression(["'2000' if '", amcl_scan, "' == 'all' else '60'"])

    args = [
        DeclareLaunchArgument(
            "use_sim_time", default_value="false",
            description="シミュレーション時刻 (/clock) を使うか。実機は false。"),
        DeclareLaunchArgument(
            "params_file", default_value=os.path.join(pkg_share, "config", "nav2_params.yaml"),
            description="Nav2 パラメータファイルへのパス (amcl 以外)。"),
        DeclareLaunchArgument(  # claude: 2026-10-08 amcl 節の分離に伴い追加
            "amcl_params_file", default_value=os.path.join(pkg_share, "config", "localization", "amcl.yaml"),
            description="AMCL パラメータファイルへのパス。"),
        # 地図一式の親ディレクトリ。別の計測日に切り替えるときはここだけ差し替える。
        DeclareLaunchArgument(
            "map_dir", default_value="/workspace/maps/2d/slam_toolbox/2026_6_9__22:00",
            description="地図一式 (nav2/, keep_out/) を含む親ディレクトリ。"),
        DeclareLaunchArgument(
            "map_yaml", default_value=PathJoinSubstitution([map_dir, "nav2", "map.yaml"]),
            description="本体地図 yaml へのフルパス。指定すると map_dir 規約より優先。"),
        DeclareLaunchArgument(
            "keepout_yaml", default_value=PathJoinSubstitution([map_dir, "keep_out", "keep_out.yaml"]),
            description="keepout マスク yaml へのフルパス。use_keepout:=false なら未使用。"),
        DeclareLaunchArgument(
            "use_keepout", default_value="true",
            description="keepout マスクを配信するか。マスク未作成の地図では false にする。"),
        DeclareLaunchArgument(
            "amcl_scan", default_value="default",
            description="AMCL の入力スキャン: default=scan_topic / all=/rfans/scan_all (全点) / far=/rfans/scan_far (最遠点)"),
        DeclareLaunchArgument(  # claude: 2026-10-08
            "scan_topic", default_value="/urg_front/scan",
            description="amcl_scan:=default のときの入力 LaserScan (GLIM 地図なら /rfans/scan)。"),
        DeclareLaunchArgument(
            "loc_map_yaml", default_value="",
            description="AMCL 専用地図 yaml (/map_loc で配信)。空なら AMCL は /map を読む。"),
    ]

    # claude: include は GroupAction(scoped=True) で包む — 子 launch に渡した引数 (params_file 等) が
    #   親の launch configuration を上書きして後続 include に漏れるのを防ぐ (2026-10-08 実際に
    #   nav2.launch.py で loc_amcl の params_file が navigation 側に漏れ controller_server が落ちた)。
    def include(rel, launch_arguments):
        return GroupAction([IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(parts, rel)),
            launch_arguments=launch_arguments.items(),
        )], scoped=True)

    return LaunchDescription(args + [
        include("map_keepout.launch.py", {
            "use_sim_time": use_sim_time, "params_file": params_file,
            "map_yaml": map_yaml, "keepout_yaml": keepout_yaml, "use_keepout": use_keepout,
        }),
        include("localization/loc_amcl.launch.py", {
            "use_sim_time": use_sim_time, "params_file": amcl_params_file,
            "scan_topic": amcl_scan_topic, "max_beams": amcl_max_beams, "loc_map_yaml": loc_map_yaml,
        }),
        include("navigation.launch.py", {"use_sim_time": use_sim_time, "params_file": params_file}),
        include("rviz_nav.launch.py", {
            "use_sim_time": use_sim_time, "rviz_config": os.path.join(pkg_share, "rviz", "nav2.rviz"),
        }),
    ])
