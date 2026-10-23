#!/usr/bin/env bash
# claude: 2026-10-17。同一 seg を「旧 T_lidar_imu」「yaw 補正版」それぞれ N 回回して、
#         実行間のばらつきの幅と、補正による差を分離する。
#
# 背景: このセッションで、odometry の設定を完全に同一にした 2 run で odom z 幅が
#       32.95 → 42.33 m (28%) 動いた実例がある。単発どうしの比較では設定の効果と
#       ばらつきを区別できない (docs/issue/2026-10-10 §6(b)/§7 と同じ警告)。
#
#   ./scripts/glim_yaw_repeat.sh <seg番号> <繰り返し回数>
#   例) ./scripts/glim_yaw_repeat.sh 3 3    … seg3 を旧 3 回 + yawfix 3 回
#
# 2 つの config は T_lidar_imu の 2 要素だけが異なる (他は完全一致を確認済み)。
# bag は 1 回だけ取得して使い回し、最後に削除する。
set -eu
cd "$(dirname "$0")/.."
REPO=$(pwd)

S=${1:?使い方: $0 <seg番号> <繰り返し回数>}
N=${2:-3}
BAG="2026-10-03_1126_tsukuba_seg$S"
HOST_BAG="$REPO/bags/raw/$BAG"
DUMPDIR="/workspace/bags/glim/${BAG}_dump"
LOGDIR="$REPO/bags/exp/glim_yaw_repeat"
mkdir -p "$LOGDIR"

declare -A CFG=(
  [old]=/workspace/bags/exp/glim_g1_old_config
  [yawfix]=/workspace/bags/exp/2026-10-17_glim_g1_yawfix_config
)

docker compose --profile glim_gpu up -d glim_gpu >/dev/null

if [ ! -d "$HOST_BAG" ]; then
  echo "[seg$S] Drive から取得"
  rclone cat "cit-share-bags:bags/raw/$BAG.tar.zst" 2>/dev/null | zstd -dc --long=31 | tar -xf - -C "$REPO/bags/raw/"
  [ -d "$HOST_BAG" ] || { echo "ERROR: 展開に失敗"; exit 1; }
fi
echo "[seg$S] bag $(du -sh "$HOST_BAG"|cut -f1) / 空き $(df -h "$REPO"|awk 'NR==2{print $4}')"

for v in old yawfix; do
  for i in $(seq 1 "$N"); do
    tag="rep_${v}_r${i}"
    echo "[seg$S] $tag 実行中..."
    docker exec glim_gpu_env /ros_entrypoint.sh bash -c "
      ros2 run glim_ros glim_rosbag /workspace/bags/raw/$BAG \
        --ros-args -p config_path:=${CFG[$v]} -p auto_quit:=true \
        -p dump_path:=$DUMPDIR/$tag \
        > /workspace/bags/exp/glim_yaw_repeat/${BAG}_${tag}.log 2>&1" \
      || echo "[seg$S] WARN: $tag 非ゼロ終了"
  done
done

rm -rf "$HOST_BAG"
echo "[seg$S] bag を削除 / 空き $(df -h "$REPO"|awk 'NR==2{print $4}')"
echo "=== 完了。集計は scripts/glim_yaw_repeat_report.sh ==="
