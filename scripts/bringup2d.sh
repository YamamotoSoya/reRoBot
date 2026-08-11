#!/usr/bin/env bash
# claude: 2D bringup (CANopen bus + epos4_controller/odometry + robot_state_publisher + urg_node)
# を main コンテナでバックグラウンド起動する。前提: can0 up (通常はアダプタ挿入で自動)・build.sh main 済み。
# ログ: docker exec rerobot_env tail -f /workspace/log/bringup2d.log
# claude_ekf: IMU=true / EKF=true (env 変数) で BNO086 + robot_localization 融合を有効化 (2026-08-11)。
#   例: IMU=true EKF=true ./scripts/bringup2d.sh   (nav2d.sh はこの構成で呼ぶ)
set -eu
cd "$(dirname "$0")/.."
IMU="${IMU:-false}"
EKF="${EKF:-false}"
docker compose up -d main

docker exec rerobot_env bash -c 'test -f /workspace/install/setup.bash' || {
  echo "[bringup2d] ERROR: main ws が未ビルドです。先に scripts/build.sh main を実行してください。"
  exit 1
}

# claude_ekf: ラッパ launch は imu を固定してしまうため、IMU/EKF 指定時は実体 launch に
# boolean を渡す (IMU=false EKF=false なら従来の rerobot_bringup_2d.launch.py と同一構成)。
docker exec -d rerobot_env bash -c \
  "source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash && \
   exec ros2 launch rerobot_bringup rerobot_bringup.launch.py \
     lidar_2d:=true lidar_3d:=false imu:=${IMU} ekf:=${EKF} \
     >> /workspace/log/bringup2d.log 2>&1"
echo "[bringup2d] 起動しました (EPOS4 初期化に数秒かかります / imu=${IMU} ekf=${EKF})"
echo "            ログ: docker exec rerobot_env tail -f /workspace/log/bringup2d.log"
