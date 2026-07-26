#!/usr/bin/env bash
# claude: 2D 自律走行の一括起動 = bringup2d (バックグラウンド) + Nav2 + RViz (フォアグラウンド)。
# nav2 は main コンテナに統合されている (依存衝突が無いためコンテナ分離しない設計判断)。
# 終了は Ctrl-C (Nav2 側のみ停止。bringup も止めるなら scripts/stop.sh)。
set -eu
cd "$(dirname "$0")/.."
./scripts/bringup2d.sh

echo "[nav2d] Nav2 + RViz を起動します (Ctrl-C で Nav2 のみ終了)"
docker exec -it rerobot_env bash -c \
  'source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash && \
   ros2 launch rerobot_bringup nav2.launch.py'
