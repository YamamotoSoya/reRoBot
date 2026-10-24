#!/usr/bin/env bash
# claude: 2026-10-23 作成。新習志野チャレンジ bag を 4 区間に分けて GLIM にかける。
#
# 区間は GLIM 補正後軌跡 (gpu_default) の走行距離で 4 等分し、境界に前後 30 秒の重なりを入れたもの。
# bag は切らず glim_rosbag の start_offset / playback_duration で仮想分割する。
#
#   CONFIG=<config ディレクトリ名> TAG=<dump タグ> ./scripts/glim_shinnara_seg.sh [S1 S2 S3 S4]
#     CONFIG 未指定 = /glim_config (= GPU 既定。glim_gpu_env では config.gpu.json が被さる)
#     引数なし = 4 区間すべて
#
#   例) 通常密度   : TAG=seg_default ./scripts/glim_shinnara_seg.sh
#       ultra      : CONFIG=shinnara_ultra TAG=seg_ultra ./scripts/glim_shinnara_seg.sh
#       posegraph  : CONFIG=shinnara_pg_dense TAG=seg_pg ./scripts/glim_shinnara_seg.sh
#
#   停止: ./scripts/glim_shinnara_seg.sh --stop
#
# 出力: bags/glim/2026-10-22_1622_shinnarachallenge_dump/<TAG>_<S>/
#       bags/exp/glim_hq/2026-10-22_1622_shinnarachallenge/<TAG>_<S>.{glim.log,resource.csv}
#
# ⚠️ 唯一のループ (始点↔終点、走行 1621 m ↔ 26 m) は S1 と S4 にまたがるため分割すると失われる。
set -eu
cd "$(dirname "$0")/.."
REPO=$(pwd)
BAG=2026-10-22_1622_shinnarachallenge

if [ "${1:-}" = "--stop" ]; then
  docker exec glim_gpu_env pkill -f glim_rosbag 2>/dev/null || true
  echo "[shinnara_seg] 停止要求を送りました"; exit 0
fi

TAG=${TAG:?TAG=<dump タグ> を指定してください}
CFG=${CONFIG:-}
CFGPATH=/glim_config
if [ -n "$CFG" ]; then
  [ -d "$REPO/ros2_ws_glim/config/$CFG" ] || { echo "ERROR: config が無い: ros2_ws_glim/config/$CFG"; exit 1; }
  CFGPATH=/glim_config/$CFG
fi

# 名前:start_offset:playback_duration
JOBS="S1:0:1065.6 S2:1005.6:1015.0 S3:1960.7:955.2 S4:2855.9:973.7"
WANT="$*"

[ -d "$REPO/bags/raw/$BAG" ] || { echo "ERROR: bag が無い: bags/raw/$BAG"; exit 1; }
docker compose --profile glim_gpu up -d glim_gpu >/dev/null
LOGDIR="$REPO/bags/exp/glim_hq/$BAG"; mkdir -p "$LOGDIR"

for job in $JOBS; do
  S=${job%%:*}; rest=${job#*:}; OFF=${rest%%:*}; DUR=${rest#*:}
  [ -z "$WANT" ] || case " $WANT " in *" $S "*) ;; *) continue;; esac

  T="${TAG}_${S}"; LOG="$LOGDIR/${T}.glim.log"; CSV="$LOGDIR/${T}.resource.csv"
  ARGS="-p config_path:=$CFGPATH -p auto_quit:=true -p dump_path:=/workspace/bags/glim/${BAG}_dump/$T"
  [ "$OFF" != "0" ] && ARGS="$ARGS -p start_offset:=$OFF"
  ARGS="$ARGS -p playback_duration:=$DUR"

  echo "[$S] 実行 (config=$CFGPATH offset=$OFF dur=$DUR) -> $T  / 空き $(df -h "$REPO"|awk 'NR==2{print $4}')"
  echo "elapsed_s,rss_mb,sys_avail_mb,vram_mb" > "$CSV"
  ( t0=$(date +%s)
    while docker exec glim_gpu_env pgrep -f glim_rosbag >/dev/null 2>&1; do
      rss=$(ps -eo rss,comm --no-headers | awk '$2=="glim_rosbag"{s+=$1} END{printf "%d", s/1024}')
      av=$(free -m | awk '/^Mem:/{print $7}')
      vr=$(docker exec glim_gpu_env nvidia-smi --query-gpu=memory.used --format=csv,noheader,nounits 2>/dev/null | head -1)
      echo "$(($(date +%s)-t0)),${rss:-0},${av:-0},${vr:-0}" >> "$CSV"; sleep 15
    done ) &
  sampler=$!
  docker exec glim_gpu_env /ros_entrypoint.sh bash -c \
    "ros2 run glim_ros glim_rosbag /workspace/bags/raw/$BAG --ros-args $ARGS > /workspace/bags/exp/glim_hq/$BAG/${T}.glim.log 2>&1" \
    || echo "[$S] WARN: 非ゼロ終了"
  wait $sampler 2>/dev/null || true
  echo "[$S] 完了: $(du -sh "$REPO/bags/glim/${BAG}_dump/$T" 2>/dev/null | cut -f1)  ピーク $(awk -F, 'NR>1{if($2>r)r=$2; if($4>v)v=$4} END{printf "RSS %d MB / VRAM %d MB", r, v}' "$CSV")"
done
echo "=== $TAG 完了 ==="
