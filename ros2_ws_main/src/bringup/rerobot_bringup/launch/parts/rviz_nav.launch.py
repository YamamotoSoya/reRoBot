# claude: 2026-10-08 部品 — Nav2 用 RViz (nav2.rviz)。nav2.launch.py から切り出し。
#   Fixed Frame: map, Navigation 2 パネル, /map・keepout・costmap・plan・パーティクル (amcl / emcl2 両方) 等。
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("rerobot_bringup")
    rviz_config = LaunchConfiguration("rviz_config")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("rviz_config", default_value=os.path.join(pkg_share, "rviz", "nav2.rviz")),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config],
            parameters=[{"use_sim_time": use_sim_time}],
            output="screen",
        ),
    ])
