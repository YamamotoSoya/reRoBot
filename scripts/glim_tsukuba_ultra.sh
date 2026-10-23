#!/usr/bin/env bash
# claude: 2026-10-23 作成。つくば bag を「手動でつなぐ前提の高密度 seg 地図」に分けて作る。
#
# 背景: この PC は 1 run あたり約 8,400 万点 (VRAM 8 GB 律速、実測) しか保持できない。
#   ultra 設定の密度は約 123,000 点/m なので、1 本の seg は約 680 m が上限。
#   seg1 (1006 m) / seg2 (796 m) はそのままでは入らないので、glim_rosbag の
#   start_offset + playback_duration で「bag を切らずに」仮想分割する。
#   境界は前後 30 秒ずつ重ねてある — 手動でつなぐときに共通形状が要るため。
#
#   ./scripts/glim_tsukuba_ultra.sh                 … 全 5 本 (seg3 は作成済みなので対象外)
#   ./scripts/glim_tsukuba_ultra.sh seg4 seg1_sub1  … 指定したものだけ
#   ./scripts/glim_tsukuba_ultra.sh --stop
#
# 出力: bags/glim/2026-10-03_1126_tsukuba_seg<N>_dump/ultra[_subK]/
#       bags/exp/glim_hq/2026-10-03_1126_tsukuba_seg<N>/ultra[_subK].glim.log
#
# bag は Drive からストリーム展開し、その seg の全 sub を回し終えたら消す (ディスク使い回し)。
set -eu
cd "$(dirname "$0")/.."
REPO=$(pwd)
CFG=${CONFIG:-tsukuba_ultra}

if [ "${1:-}" = "--stop" ]; then
  docker exec glim_gpu_env pkill -f glim_rosbag 2>/dev/null || true
  pkill -f glim_tsukuba_ultra 2>/dev/null || true
  echo "[ultra] 停止要求を送りました"; exit 0
fi

# 名前:seg番号:start_offset:duration   (duration 0 = 全体)
JOBS="
seg4:4:0:0
seg1_sub1:1:0:1094.1
seg1_sub2:1:1034.1:1405.9
seg2_sub1:2:0:902.8
seg2_sub2:2:842.8:927.9
"

WANT="$*"
docker compose --profile glim_gpu up -d glim_gpu >/dev/null

prev_seg=""
for job in $JOBS; do
  name=${job%%:*}; rest=${job#*:}
  S=${rest%%:*}; rest=${rest#*:}
  OFF=${rest%%:*}; DUR=${rest#*:}
  [ -z "$WANT" ] || case " $WANT " in *" $name "*) ;; *) continue;; esac

  BAG="2026-10-03_1126_tsukuba_seg$S"
  HOST_BAG="$REPO/bags/raw/$BAG"
  TAG=$([ "$name" = "seg$S" ] && echo ultra || echo "ultra_${name#*_}")
  LOGDIR="$REPO/bags/exp/glim_hq/$BAG"; mkdir -p "$LOGDIR"
  LOG="$LOGDIR/${TAG}.glim.log"; CSV="$LOGDIR/${TAG}.resource.csv"

  # 前の seg の bag はもう要らない (Drive に原本がある)
  if [ -n "$prev_seg" ] && [ "$prev_seg" != "$S" ]; then
    rm -rf "$REPO/bags/raw/2026-10-03_1126_tsukuba_seg$prev_seg"
    echo "[$name] seg$prev_seg の bag を削除 / 空き $(df -h "$REPO"|awk 'NR==2{print $4}')"
  fi
  prev_seg=$S

  if [ ! -d "$HOST_BAG" ]; then
    echo "[$name] Drive から取得 (ストリーム展開)"
    rclone cat "cit-share-bags:bags/raw/$BAG.tar.zst" 2>/dev/null | zstd -dc --long=31 | tar -xf - -C "$REPO/bags/raw/"
    [ -d "$HOST_BAG" ] || { echo "[$name] ERROR: 展開に失敗"; exit 1; }
  fi

  ARGS="-p config_path:=/glim_config/$CFG -p auto_quit:=true -p dump_path:=/workspace/bags/glim/${BAG}_dump/$TAG"
  [ "$OFF" != "0" ] && ARGS="$ARGS -p start_offset:=$OFF"
  [ "$DUR" != "0" ] && ARGS="$ARGS -p playback_duration:=$DUR"

  echo "[$name] 実行 (config=$CFG offset=$OFF dur=$DUR) -> $TAG"
  echo "elapsed_s,rss_mb,sys_used_mb,sys_avail_mb,vram_mb" > "$CSV"
  ( t0=$(date +%s)
    while docker exec glim_gpu_env pgrep -f glim_rosbag >/dev/null 2>&1; do
      rss=$(ps -eo rss,comm --no-headers 2>/dev/null | awk '$2=="glim_rosbag"{s+=$1} END{printf "%d", s/1024}')
      read -r used avail <<<"$(free -m | awk '/^Mem:/{print $3, $7}')"
      vram=$(docker exec glim_gpu_env nvidia-smi --query-gpu=memory.used --format=csv,noheader,nounits 2>/dev/null | head -1)
      echo "$(( $(date +%s) - t0 )),${rss:-0},${used:-0},${avail:-0},${vram:-0}" >> "$CSV"
      sleep 15
    done ) &
  sampler=$!
  docker exec glim_gpu_env /ros_entrypoint.sh \
    bash -c "ros2 run glim_ros glim_rosbag /workspace/bags/raw/$BAG --ros-args $ARGS > /workspace/bags/exp/glim_hq/$BAG/${TAG}.glim.log 2>&1" \
    || echo "[$name] WARN: 非ゼロ終了"
  wait $sampler 2>/dev/null || true
  echo "[$name] 完了: $(du -sh "$REPO/bags/glim/${BAG}_dump/$TAG" 2>/dev/null | cut -f1)  ピーク $(awk -F, 'NR>1{if($2>r)r=$2; if($5>v)v=$5} END{printf "RSS %d MB / VRAM %d MB", r, v}' "$CSV")"
done
[ -n "$prev_seg" ] && rm -rf "$REPO/bags/raw/2026-10-03_1126_tsukuba_seg$prev_seg"
echo "=== 全て完了 ==="
