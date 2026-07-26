#!/usr/bin/env bash
# claude: 3D SLAM (GLIM) の一括起動 = bringup3d (main, バックグラウンド)
#         + glim_rosnode (glim コンテナ = koide3 公式イメージ, フォアグラウンド)。
# GLIM は /sdk_could を購読する (config: ros2_ws_glim/config → /glim_config に mount)。
# 公式イメージ内の ROS 環境は /ros_entrypoint.sh 経由で source する (overlay の場所に依存しないため)。
set -eu
cd "$(dirname "$0")/.."
./scripts/bringup3d.sh

docker compose --profile glim up -d glim

echo "[glim3d] /sdk_could の点群を待っています (最大 30 秒)..."
docker exec glim_env /ros_entrypoint.sh bash -c \
  'timeout 30 bash -c "until ros2 topic list 2>/dev/null | grep -q sdk_could; do sleep 1; done"' || {
  echo "[glim3d] ERROR: /sdk_could が見えません。bringup3d のログと R-Fans の接続を確認してください。"
  exit 1
}

echo "[glim3d] GLIM を起動します (Ctrl-C で GLIM のみ終了)"
docker exec -it glim_env /ros_entrypoint.sh \
  ros2 run glim_ros glim_rosnode --ros-args -p config_path:=/glim_config
