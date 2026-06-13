# ============================================================
#  joy_teleop.launch.py                                 # claude
#  Xbox コントローラ teleop: joy_node + teleop_twist_joy_node を起動し、
#  teleop_twist_joy の既定出力 /cmd_vel を、本ロボットが購読する
#  /robot_speed_cmd にリマップして epos4_controller へ直結する。
#
#  使い方:
#     ros2 launch rerobot_bringup joy_teleop.launch.py
#
#  ★ この機能をやめたいとき（完全撤去手順）:
#     1. このファイル (launch/joy_teleop.launch.py) を削除
#     2. config/joy_teleop.yaml を削除
#     3. package.xml の joy / teleop_twist_joy の exec_depend 2行を削除
#     → 既存の launch / config には手を入れていないので元に戻ります。
# ============================================================
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('rerobot_bringup'),
        'config', 'joy_teleop.yaml')

    # /dev/input/js* を読んで sensor_msgs/Joy を publish
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[params_file],
        output='screen',
    )

    # Joy → geometry_msgs/Twist 変換。出力 /cmd_vel を /robot_speed_cmd へ
    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[params_file],
        remappings=[('/cmd_vel', '/robot_speed_cmd')],
        output='screen',
    )

    return LaunchDescription([joy_node, teleop_node])
