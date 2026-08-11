#!/usr/bin/env bash
# claude: 2D 自律走行の一括起動 = bringup2d (バックグラウンド) + Nav2 + RViz (フォアグラウンド)。
# nav2 は main コンテナに統合されている (依存衝突が無いためコンテナ分離しない設計判断)。
# 終了は Ctrl-C (Nav2 側のみ停止。bringup も止めるなら scripts/stop.sh)。
# claude_ekf: 2026-08-11 から IMU + EKF 融合オドメトリが標準 (nav2_params.yaml の
#   odom_topic=/odometry/filtered と対)。IMU 故障時の暫定運転は
#   ./scripts/bringup2d.sh (EKF なし) + nav2_params.yaml の odom_topic を /odom に戻す。
set -eu
cd "$(dirname "$0")/.."
IMU=true EKF=true ./scripts/bringup2d.sh

echo "[nav2d] Nav2 + RViz を起動します (Ctrl-C で Nav2 のみ終了)"
# claude: スクリプト引数を launch にそのまま渡す。
#   例: ./scripts/nav2d.sh map_yaml:=/workspace/maps/slam_toolbox/2026_8_11__18-20.yaml use_keepout:=false
docker exec -it rerobot_env bash -c \
  'source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash && \
   ros2 launch rerobot_bringup nav2.launch.py "$@"' _ "$@"
