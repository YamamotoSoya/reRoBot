#!/usr/bin/env bash
# claude: キーボード teleop (w/s/a/d, space=停止, f=脱力トグル, r=距離リセット, q=終了)。
# bringup (2d/3d どちらでも) が動いている状態で実行する。対話型なので -it 必須。
set -eu
cd "$(dirname "$0")/.."
docker exec -it rerobot_env bash -c \
  'source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash && \
   ros2 run epos4_teleop teleop_keyboard --ros-args \
     --params-file /workspace/src/app/epos4_teleop/config/params.yaml'
