#!/usr/bin/env bash
# claude: 2D 地図作成の一括起動 = bringup2d (main, バックグラウンド)
#         + slam_toolbox + RViz (slamtoolbox コンテナ, フォアグラウンド)。
# /scan と TF は main → slamtoolbox に DDS 経由で流れる。終了は Ctrl-C。
set -eu
cd "$(dirname "$0")/.."
./scripts/bringup2d.sh

docker compose --profile slamtoolbox up -d slamtoolbox
docker exec slamtoolbox_env bash -c 'test -f /workspace/install/setup.bash' || {
  echo "[slam2d] ERROR: slamtoolbox ws が未ビルドです。先に scripts/build.sh slamtoolbox を実行してください。"
  exit 1
}

echo "[slam2d] slam_toolbox + RViz を起動します (Ctrl-C で SLAM のみ終了)"
docker exec -it slamtoolbox_env bash -c \
  'source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash && \
   ros2 launch rerobot_slamtoolbox slam.launch.py'
