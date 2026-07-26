#!/usr/bin/env bash
# claude: 2D bringup (CANopen bus + epos4_controller/odometry + robot_state_publisher + urg_node)
# を main コンテナでバックグラウンド起動する。前提: scripts/can_up.sh 済み・build.sh main 済み。
# ログ: docker exec rerobot_env tail -f /workspace/log/bringup2d.log
set -eu
cd "$(dirname "$0")/.."
docker compose up -d main

docker exec rerobot_env bash -c 'test -f /workspace/install/setup.bash' || {
  echo "[bringup2d] ERROR: main ws が未ビルドです。先に scripts/build.sh main を実行してください。"
  exit 1
}

docker exec -d rerobot_env bash -c \
  'source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash && \
   exec ros2 launch rerobot_bringup rerobot_bringup_2d.launch.py \
     >> /workspace/log/bringup2d.log 2>&1'
echo "[bringup2d] 起動しました (EPOS4 初期化に数秒かかります)"
echo "            ログ: docker exec rerobot_env tail -f /workspace/log/bringup2d.log"
