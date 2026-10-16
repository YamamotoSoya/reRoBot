# ============================================================
#  joy_teleop.launch.py                                 # claude
#  Xbox コントローラ teleop: joy_node + teleop_twist_joy_node を起動し、
#  teleop_twist_joy の既定出力 /cmd_vel を /joy_vel にリマップする。
#  /joy_vel は rerobot_bringup.launch.py が常駐させる twist_mux が最優先入力として受け、
#  /robot_speed_cmd (epos4_controller) へ流す (2026-10-16。それ以前は /robot_speed_cmd 直結)。
#  → LB を握っている間は Nav2 の指令より joy が勝ち、離せば 0.5 s で Nav2 に戻る。
#  ⚠ bringup (twist_mux) 無しでこの launch だけ上げてもロボットは動かない (/joy_vel を誰も受けない)。
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

    # Joy → geometry_msgs/Twist 変換。出力 /cmd_vel を /joy_vel へ (twist_mux の最優先入力)
    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[params_file],
        remappings=[('/cmd_vel', '/joy_vel')],  # claude: 2026-10-16 /robot_speed_cmd 直結 → twist_mux 経由
        output='screen',
    )

    # claude: 2026-10-09 停止点ティーチング用マーカー。「2-Y」ボタン (button 3) で /teach_marker (時刻 + ラベル) を出す。
    #   地図作成 bag に /teach_marker を記録しておき、座標は後で GLIM 軌跡から引く (scripts/teach_marker.py)
    teach_node = Node(
        package='rerobot_bringup',
        executable='teach_marker.py',
        name='teach_marker',
        parameters=[params_file],
        output='screen',
    )

    return LaunchDescription([joy_node, teleop_node, teach_node])
