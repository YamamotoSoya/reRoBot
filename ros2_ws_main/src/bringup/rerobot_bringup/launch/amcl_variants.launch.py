# claude: 2026-10-01 新設。
# AMCL を複数系統「並走」させる launch。config/amcl_variants.yaml の variants ごとに
#   スキャン生成ノード (pointcloud_to_laserscan / rfans_scan_modes / なし)
#   + 名前空間 /loc_<name> の nav2_amcl (tf_broadcast false、scan_topic 個別、initialpose は /initialpose 共通)
# を立て、変種専用の lifecycle_manager で活性化する。本番 AMCL (nav2.launch.py) には触らない。
#
# 使い方:
#   実走:  nav2d.sh の後に  ros2 launch rerobot_bringup amcl_variants.launch.py
#   再生:  ros2 launch rerobot_bringup amcl_variants.launch.py use_sim_time:=true start_map_server:=true
#          (再生では nav2.launch.py を立てないので map_server をここで起動する)
import math
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.substitution import Substitution
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml


class OpaqueSubstitution(Substitution):
    """perform 時に関数を呼ぶ Substitution (ファイル内容を launch 引数として渡す用)。"""

    def __init__(self, fn):
        super().__init__()
        self._fn = fn

    def perform(self, context):
        return self._fn(context)


def _setup(context, *args, **kwargs):
    share = get_package_share_directory("rerobot_bringup")
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context)
    params_file = LaunchConfiguration("params_file").perform(context)
    variants_file = LaunchConfiguration("variants_file").perform(context)
    only = [v for v in LaunchConfiguration("only").perform(context).split(",") if v]
    # stagger > 0: AMCL ごとに専用 lifecycle_manager を stagger 秒ずつずらして起動する。
    #   nav2_amcl の粒子フィルタは configure 時に srand48(time(NULL)) (秒単位) で種を決めるため、
    #   同じ秒に configure されたコピーは同じ乱数列になり独立な試行にならない (2026-10-01 critic 指摘)。
    stagger = float(LaunchConfiguration("stagger").perform(context))
    sim = use_sim_time.lower() in ("true", "1")

    with open(variants_file, encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    common = cfg.get("common", {})
    variants = cfg["variants"]
    if only:
        variants = {k: v for k, v in variants.items() if k in only}

    # AMCL の max_beams 既定値を params_file から拾う (間隔ログ用)
    with open(params_file, encoding="utf-8") as f:
        base_amcl = yaml.safe_load(f).get("amcl", {}).get("ros__parameters", {})

    actions, managed = [], []
    for name, v in variants.items():
        scan_topic = v.get("scan_topic", f"/scan_{name}")
        src = v.get("source", "modes")
        band_lo = float(v.get("min_height", common.get("min_height", 0.3)))
        band_hi = float(v.get("max_height", common.get("max_height", 1.5)))
        inc = float(v.get("angle_increment", 0.0035))
        rmax = float(v.get("range_max", common.get("range_max", 30.0)))
        cloud = v.get("cloud_topic", common.get("cloud_topic", "/rfans_driver/rfans_points"))
        frame = v.get("target_frame", common.get("target_frame", "base_link"))

        if src == "p2l":
            actions.append(
                Node(
                    package="pointcloud_to_laserscan",
                    executable="pointcloud_to_laserscan_node",
                    name=f"scan_{name}",
                    remappings=[("cloud_in", cloud), ("scan", scan_topic)],
                    parameters=[
                        {
                            "use_sim_time": sim,
                            "target_frame": frame,
                            "transform_tolerance": 0.05,
                            "min_height": band_lo,
                            "max_height": band_hi,
                            "angle_min": -math.pi,
                            "angle_max": math.pi,
                            "angle_increment": inc,
                            "scan_time": 0.1,
                            "range_min": float(v.get("range_min", 0.5)),
                            "range_max": rmax,
                            "use_inf": True,
                        }
                    ],
                )
            )
        elif src == "modes":
            actions.append(
                Node(
                    package="rfans_scan_modes",
                    executable="scan_modes",
                    name=f"scan_{name}",
                    output="screen",
                    parameters=[
                        {
                            "use_sim_time": sim,
                            "cloud_topic": cloud,
                            "scan_topic": scan_topic,
                            "target_frame": frame,
                            "mode": v.get("mode", "nearest"),
                            "nth_k": int(v.get("nth_k", 1)),
                            "min_height": band_lo,
                            "max_height": band_hi,
                            "angle_increment": inc,
                            "range_min": float(v.get("range_min", 0.5)),
                            "range_max": rmax,
                        }
                    ],
                )
            )
        elif src not in ("bag_scan", "none"):
            raise ValueError(f"variant {name}: unknown source {src}")

        # ⚠ RewrittenYaml の param_rewrites は「params_file に既にあるキー」しか置換しない
        #   (map_topic / tf_broadcast は nav2_params.yaml に無いので効かず、AMCL が /loc_x/map を待ち続けた)。
        #   RewrittenYaml は名前空間付けだけに使い、上書きは型付き dict で後から渡す。
        amcl_over = dict(v.get("amcl") or {})
        cfg_amcl = RewrittenYaml(
            source_file=params_file, root_key=f"loc_{name}", param_rewrites={}, convert_types=True
        )
        overrides = {
            "use_sim_time": sim,
            "tf_broadcast": False,
            "scan_topic": scan_topic,
            "map_topic": "/map",
            **amcl_over,
        }
        actions.append(
            Node(
                package="nav2_amcl",
                executable="amcl",
                name="amcl",
                namespace=f"loc_{name}",
                output="screen",
                parameters=[cfg_amcl, overrides],
                remappings=[("initialpose", "/initialpose")],
            )
        )
        managed.append(f"/loc_{name}/amcl")

        # 間引き間隔のログ: 席数 / max_beams とリング数 16 との最大公約数 (ストロボ効果の検知)
        seats = int(round(2 * math.pi / inc))
        mb = int(amcl_over.get("max_beams", base_amcl.get("max_beams", 60)))
        step = max(1, (seats - 1) // max(1, mb - 1))
        actions.append(
            LogInfo(
                msg=f"[variants] {name}: scan={scan_topic} src={src} band=[{band_lo},{band_hi}] "
                f"seats={seats} max_beams={mb} step={step} gcd(step,16)={math.gcd(step, 16)}"
                + ("  ⚠ step は 16 の倍数/約数 → 特定リングだけ拾う恐れ" if math.gcd(step, 16) > 1 else "")
            )
        )

    if stagger > 0:
        for i, node_name in enumerate(managed):
            short = node_name.strip("/").replace("/", "_")
            actions.append(
                TimerAction(
                    period=i * stagger,
                    actions=[
                        Node(
                            package="nav2_lifecycle_manager",
                            executable="lifecycle_manager",
                            name=f"lcm_{short}",
                            output="screen",
                            parameters=[{"use_sim_time": sim, "autostart": True, "bond_timeout": 0.0,
                                         "node_names": [node_name]}],
                        )
                    ],
                )
            )
        actions.append(LogInfo(msg=f"[variants] managers={len(managed)} stagger={stagger}s"))
    else:
        actions.append(
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_variants",
                output="screen",
                # bond_timeout 0: 管理ノードが別名前空間 (/loc_*) にいると bond トピックが一致せず 4 s で失敗するため無効化
                parameters=[{"use_sim_time": sim, "autostart": True, "bond_timeout": 0.0, "node_names": managed}],
            )
        )
        actions.append(LogInfo(msg="[variants] managers=1 stagger=0"))
    return actions


def generate_launch_description():
    share = get_package_share_directory("rerobot_bringup")
    use_sim_time = LaunchConfiguration("use_sim_time")
    map_yaml = LaunchConfiguration("map_yaml")
    start_map_server = LaunchConfiguration("start_map_server")

    # 再生用 map_server (+ 専用 lifecycle_manager)。実走では nav2.launch.py のものを使うので起動しない。
    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time, "yaml_filename": map_yaml}],
        condition=IfCondition(start_map_server),
    )
    map_lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_map_replay",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time, "autostart": True, "bond_timeout": 0.0, "node_names": ["map_server"]}],
        condition=IfCondition(start_map_server),
    )

    # 再生用 robot_state_publisher (静的 TF)。--start-offset 付き bag 再生では /tf_static が出ないので URDF から出す。
    # ⚠ URDF をシェルで -p robot_description:="$(cat …)" と渡すと、コメント内の「,」「(」を rcl が誤解析して
    #   Abort する (2026-10-01 判明)。ここでは Python でファイルを読み launch 経由で渡す。
    urdf_path = LaunchConfiguration("urdf")
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher_replay",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "robot_description": ParameterValue(
                    OpaqueSubstitution(lambda ctx: open(urdf_path.perform(ctx), encoding="utf-8").read()),
                    value_type=str,
                ),
            }
        ],
        condition=IfCondition(LaunchConfiguration("start_robot_state_publisher")),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("start_robot_state_publisher", default_value="false"),
            DeclareLaunchArgument("urdf", default_value=os.path.join(share, "urdf", "rerobot.urdf")),
            rsp,
            DeclareLaunchArgument(
                "params_file", default_value=os.path.join(share, "config", "nav2_params.yaml")
            ),
            DeclareLaunchArgument(
                "variants_file", default_value=os.path.join(share, "config", "amcl_variants.yaml")
            ),
            DeclareLaunchArgument("only", default_value="", description="カンマ区切りで変種名を絞る"),
            DeclareLaunchArgument("stagger", default_value="0.0", description="AMCL ごとの起動ずらし [s] (乱数種を独立にする。反復 run では 1.1)"),
            DeclareLaunchArgument("start_map_server", default_value="false"),
            DeclareLaunchArgument(
                "map_yaml",
                default_value="/workspace/maps/2d/glim/2026-09-180915_dumpbase/nav2/my_map.yaml",
            ),
            map_server,
            map_lifecycle,
            OpaqueFunction(function=_setup),
        ]
    )
