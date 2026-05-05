# claude: async_slam_toolbox_node を本リポジトリ用 params で起動する単独 launch。
# 前提として rerobot_bringup.launch.py が urg_node + robot_state_publisher + epos4_odometry
# を立ち上げ、/scan と TF (odom→base_link, base_link→laser) が流れていること。
#
# claude: async_slam_toolbox_node は managed (lifecycle) node で、起動直後は
# `unconfigured` 状態のまま待機する。`autostart: true` パラメータは Jazzy 配布版の
# slam_toolbox では効かなかったため、launch 側から明示的に CONFIGURE→ACTIVATE を
# 発火する slam_toolbox 公式 (online_async_launch.py) と同じパターンを採用する。
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler   # life-cycle-Event_by_claude
from launch.events import matches_action                     # life-cycle-Event_by_claude
from launch_ros.actions import LifecycleNode                 # life-cycle-Event_by_claude
from launch_ros.event_handlers import OnStateTransition      # life-cycle-Event_by_claude
from launch_ros.events.lifecycle import ChangeState          # life-cycle-Event_by_claude
from lifecycle_msgs.msg import Transition                    # life-cycle-Event_by_claude


def generate_launch_description():
    pkg_share = get_package_share_directory("rerobot_bringup")
    slam_params = os.path.join(pkg_share, "config", "slam_toolbox.yaml")

    # life-cycle-Event_by_claude: 通常の Node ではなく LifecycleNode を使うことで
    # launch 側から ChangeState イベントを送れるようになる。
    slam_toolbox_node = LifecycleNode(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        namespace="",
        parameters=[slam_params],
        output="screen",
    )

    # life-cycle-Event_by_claude:
    # launch 開始直後に CONFIGURE 遷移を発火 (unconfigured → inactive)。
    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(slam_toolbox_node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )

    # life-cycle-Event_by_claude:
    # configure 完了 (= inactive 到達) を検知して ACTIVATE を発火 (inactive → active)。
    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_toolbox_node,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(slam_toolbox_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                )
            ],
        )
    )

    return LaunchDescription([
        slam_toolbox_node,
        configure_event,                                     # life-cycle-Event_by_claude
        activate_event,                                      # life-cycle-Event_by_claude
    ])
