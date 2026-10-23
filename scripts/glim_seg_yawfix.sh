#!/usr/bin/env bash
# claude: 2026-10-17。つくば seg を yaw 補正版 T_lidar_imu で GLIM 再計算する。
#
# seg の raw bag はローカルに無く Drive にしかないため、1 本ずつ
#   取得 (ストリーム展開) → GLIM 実行 → bag 削除
# を繰り返してディスクを使い回す。展開後は seg1 約 21 GB / seg2 約 16 GB /
# seg3 約 4 GB / seg4 約 3.5 GB。
#
#   ./scripts/glim_seg_yawfix.sh 3 4        … seg3, seg4 を順に
#
# 出力:
#   bags/glim/2026-10-03_1126_tsukuba_seg<N>_dump/g1_imu_strong_yawfix/
#   bags/glim/seg<N>_yawfix.log
set -eu
cd "$(dirname "$0")/.."
REPO=$(pwd)

# claude: 使う config と dump のタグを env で選ぶ (既定は yaw 補正版)。
#   例) CFG_DIR=glim_g1_old_range180 TAG=g1_old_range180 ./scripts/glim_seg_yawfix.sh 3 4 2 1
CFG_DIR=${CFG_DIR:-2026-10-17_glim_g1_yawfix_config}
TAG=${TAG:-g1_imu_strong_yawfix}
CFG=/workspace/bags/exp/$CFG_DIR
[ -d "$REPO/bags/exp/$CFG_DIR" ] || { echo "ERROR: config が無い: bags/exp/$CFG_DIR"; exit 1; }
echo "[config] $CFG_DIR  → dump タグ: $TAG"

docker compose --profile glim_gpu up -d glim_gpu >/dev/null

for s in "$@"; do
  BAG="2026-10-03_1126_tsukuba_seg$s"
  HOST_BAG="$REPO/bags/raw/$BAG"
  LOG="$REPO/bags/glim/seg${s}_${TAG}.log"

  if [ ! -d "$HOST_BAG" ]; then
    echo "[seg$s] Drive から取得 (ストリーム展開)"
    # 中間の .tar.zst を置かずに展開する — 圧縮ファイルの置き場ぶんの空きが要らない
    rclone cat "cit-share-bags:bags/raw/$BAG.tar.zst" 2>/dev/null | zstd -dc --long=31 | tar -xf - -C "$REPO/bags/raw/"
    [ -d "$HOST_BAG" ] || { echo "[seg$s] ERROR: 展開に失敗"; exit 1; }
  fi
  echo "[seg$s] bag: $(du -sh "$HOST_BAG" | cut -f1) / 空き $(df -h "$REPO" | awk 'NR==2{print $4}')"

  echo "[seg$s] GLIM 実行"
  docker exec glim_gpu_env /ros_entrypoint.sh bash -c "
    ros2 run glim_ros glim_rosbag /workspace/bags/raw/$BAG \
      --ros-args -p config_path:=$CFG -p auto_quit:=true \
      -p dump_path:=/workspace/bags/glim/${BAG}_dump/$TAG \
      > /workspace/bags/glim/seg${s}_${TAG}.log 2>&1" || echo "[seg$s] WARN: 非ゼロ終了"

  echo "[seg$s] 完了: dump $(du -sh "$REPO/bags/glim/${BAG}_dump/$TAG" 2>/dev/null | cut -f1)"
  grep -c "T_lidar_imu is not accurate" "$LOG" 2>/dev/null | head -1 | xargs -I{} echo "[seg$s] T_lidar_imu 警告 {} 回"

  # 次の seg のために bag を消す (Drive に原本があるので復旧可能)
  rm -rf "$HOST_BAG"
  echo "[seg$s] bag を削除 / 空き $(df -h "$REPO" | awk 'NR==2{print $4}')"
done
echo "=== 全 seg 完了 ==="
