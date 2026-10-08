# claude: 2026-10-08 部品 (仮置き) — 自己位置推定: 3D LiDAR + 3D 点群地図。方式は未定。
#
# 実装時の約束 (loc_amcl.launch.py 冒頭と同じ + 3D 固有):
#   - 出力 TF map->odom は 2D に落とす (z / roll / pitch = 0)。3D 方式の 6DoF をそのまま出すと
#     map 上で base_link が傾き、costmap・RViz がずれる。方式側の 2D 拘束か、推定姿勢 → 2D TF の
#     小ノードで吸収する。
#   - 依存が main と衝突する方式なら別コンテナで動かしてよい (契約は DDS 上の TF/topic だけ)。
#     その場合この部品は「どのコンテナで何を起動するか」を案内するだけにする。
#   - 3D 地図は maps/3d/、経路計画用 /map は glim_dump_to_2dmap 由来 (同じ map 座標系)。
#
# 現状は誤って起動しても Nav2 が TF 無しで待ち続けないよう、エラーを出して launch 全体を止める。
from launch import LaunchDescription
from launch.actions import EmitEvent, LogInfo
from launch.events import Shutdown


def generate_launch_description():
    return LaunchDescription([
        LogInfo(msg="[loc_lidar3d] 3D 自己位置推定は未実装です (方式未定)。launch を終了します。"),
        EmitEvent(event=Shutdown(reason="loc_lidar3d is not implemented yet")),
    ])
