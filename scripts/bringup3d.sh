#!/usr/bin/env bash
# claude: 3D bringup (CANopen bus + epos4 + robot_state_publisher + rfans_driver) を
# main コンテナでバックグラウンド起動する。点群は /sdk_could (frame: rfans)。
# ログ: docker exec rerobot_env tail -f /workspace/log/bringup3d.log
set -eu
cd "$(dirname "$0")/.."
docker compose up -d main

docker exec rerobot_env bash -c 'test -f /workspace/install/setup.bash' || {
  echo "[bringup3d] ERROR: main ws が未ビルドです。先に scripts/build.sh main を実行してください。"
  exit 1
}

docker exec -d rerobot_env bash -c \
  'source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash && \
   exec ros2 launch rerobot_bringup rerobot_bringup_3d.launch.py \
     >> /workspace/log/bringup3d.log 2>&1'
echo "[bringup3d] 起動しました (EPOS4 初期化に数秒かかります)"
echo "            ログ: docker exec rerobot_env tail -f /workspace/log/bringup3d.log"
