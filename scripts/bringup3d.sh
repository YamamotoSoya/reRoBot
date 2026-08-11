#!/usr/bin/env bash
# claude: 3D bringup (CANopen bus + epos4 + robot_state_publisher + rfans_driver) を
# main コンテナでバックグラウンド起動する。点群は /sdk_could (frame: rfans)。
# ログ: docker exec rerobot_env tail -f /workspace/log/bringup3d.log
# claude: IMU=true (env 変数) で BNO086 を同時起動する (GLIM の LIO 構成入力用, 2026-08-11)。
#   例: IMU=true ./scripts/bringup3d.sh   (glim3d.sh はこの構成で呼ぶ)
set -eu
cd "$(dirname "$0")/.."
IMU="${IMU:-false}"
docker compose up -d main

docker exec rerobot_env bash -c 'test -f /workspace/install/setup.bash' || {
  echo "[bringup3d] ERROR: main ws が未ビルドです。先に scripts/build.sh main を実行してください。"
  exit 1
}

# claude: IMU 指定時は実体 launch に boolean を渡す (IMU=false なら従来の
# rerobot_bringup_3d.launch.py と同一構成)。
docker exec -d rerobot_env bash -c \
  "source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash && \
   exec ros2 launch rerobot_bringup rerobot_bringup.launch.py \
     lidar_2d:=false lidar_3d:=true imu:=${IMU} \
     >> /workspace/log/bringup3d.log 2>&1"
echo "[bringup3d] 起動しました (EPOS4 初期化に数秒かかります / imu=${IMU})"
echo "            ログ: docker exec rerobot_env tail -f /workspace/log/bringup3d.log"
