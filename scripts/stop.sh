#!/usr/bin/env bash
# claude: 全コンテナ内の ROS プロセスへ SIGINT を送って停止する (launch の Ctrl-C 相当)。
# コンテナ自体は bash 常駐のまま残す。完全に落とすときは docker compose down。
# 注意: epos4_controller は SIGINT で EPOS4 を disable してから終了する設計なので、
#       kill -9 ではなく必ず SIGINT で止めること。
set -u
cd "$(dirname "$0")/.."
for c in rerobot_env slamtoolbox_env glim_env liosam_env; do
  if docker ps --format '{{.Names}}' | grep -qx "$c"; then
    docker exec "$c" bash -c \
      "pkill -INT -f 'ros2 launch|ros2 run|component_container|glim_rosnode|rviz2' 2>/dev/null || true"
    echo "[stop] $c: SIGINT 送信"
  fi
done
echo "[stop] 完了 (コンテナは常駐したままです)"
