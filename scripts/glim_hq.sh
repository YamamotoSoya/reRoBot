#!/usr/bin/env bash
# claude: 2026-10-14 作成。GLIM の HQ 設定 (config/hq) でオフライン処理を回し、
#         RAM / VRAM / submap 数を 10 秒おきに記録する。
#
# ★デタッチ実行★ 起動したらすぐ戻る。GLIM 本体は docker exec -d でコンテナに投げ切るので、
#   ホスト側の端末を閉じても、ssh が切れても、呼び出し元がタイムアウトで殺されても生き続ける。
#   (2026-10-14: 前身は docker exec を前景で保持していたため、呼び出し元が殺されたときに
#    コンテナ内の glim_rosbag が孤児として残り、ログも資源記録も失われた。)
#
#   起動:   ./scripts/glim_hq.sh <bag ディレクトリ名> <タグ> [playback_duration 秒]
#   状態:   ./scripts/glim_hq.sh --status <bag ディレクトリ名> <タグ>
#   停止:   ./scripts/glim_hq.sh --stop
#
#   例) 較正走行 (bag 先頭 10 分):  ./scripts/glim_hq.sh 2026-10-03_1126_tsukuba cal600 600
#       本番 (bag 全体):            ./scripts/glim_hq.sh 2026-10-03_1126_tsukuba hq
#
# 出力:
#   bags/glim/<bag>_dump/<tag>/                      … dump 本体 (コンテナが root で作る)
#   bags/exp/glim_hq/<bag>/<tag>.glim.log            … GLIM の標準出力
#   bags/exp/glim_hq/<bag>/<tag>.resource.csv        … 10 秒おきの RAM / VRAM / submap 数
#
# ⚠️ CPU 版 glim コンテナと同時に動かさないこと
#    (network_mode: host + 同一 ROS_DOMAIN_ID でトピックが衝突する)。
set -eu
cd "$(dirname "$0")/.."
REPO=$(pwd)

# --- 停止 --------------------------------------------------------------------
if [ "${1:-}" = "--stop" ]; then
  docker exec glim_gpu_env pkill -f glim_rosbag 2>/dev/null || true
  pkill -f 'glim_hq_sampler' 2>/dev/null || true
  echo "[glim_hq] 停止要求を送りました"
  exit 0
fi

# --- 状態表示 ----------------------------------------------------------------
if [ "${1:-}" = "--status" ]; then
  BAG=${2:?使い方: $0 --status <bag> <タグ>}; TAG=${3:?使い方: $0 --status <bag> <タグ>}
  LOGDIR="$REPO/bags/exp/glim_hq/$BAG"; CSV="$LOGDIR/${TAG}.resource.csv"; GLOG="$LOGDIR/${TAG}.glim.log"
  DUMP="$REPO/bags/glim/${BAG}_dump/$TAG"
  if docker exec glim_gpu_env pgrep -f glim_rosbag >/dev/null 2>&1; then
    echo "[glim_hq] 実行中"
  else
    echo "[glim_hq] 停止 (完了または異常終了)"
  fi
  echo "--- 直近の playback speed ---"
  grep -o "playback speed: [0-9.]*x" "$GLOG" 2>/dev/null | tail -3
  grep -E "reached playback|waiting for|saved|error|Error|terminate|what\(\)" "$GLOG" 2>/dev/null | tail -5
  echo "--- 資源 (直近) ---"
  # NF==6 で壊れた行を弾く
  [ -f "$CSV" ] && awk -F, 'NF==6 && NR>1{l=$0} END{split(l,a,",");
      printf "経過 %d 分 / RSS %d MB / システム使用 %d MB (空き %d MB) / VRAM %d MB / %.3fx\n",a[1]/60,a[2],a[3],a[4],a[5],a[6]}' "$CSV"
  echo "--- 資源ピーク ---"
  [ -f "$CSV" ] && awk -F, 'NF==6 && NR>1{if($2>r)r=$2; if($3>u)u=$3; if($5>v)v=$5; n=$1}
      END{printf "RSS %d MB / システム使用 %d MB / VRAM %d MB / 経過 %d 分\n",r,u,v,n/60}' "$CSV"
  echo "--- dump ---"
  du -sh "$DUMP" 2>/dev/null || echo "(未作成 — GLIM は終了時にまとめて保存する)"
  exit 0
fi

# --- 起動 --------------------------------------------------------------------
BAG=${1:?使い方: $0 <bag ディレクトリ名> <タグ> [playback_duration 秒]}
TAG=${2:?使い方: $0 <bag ディレクトリ名> <タグ> [playback_duration 秒]}
DURATION=${3:-0}   # 0 = bag 全体

[ -d "$REPO/bags/raw/$BAG" ] || { echo "ERROR: bag が無い: $REPO/bags/raw/$BAG"; exit 1; }
if docker exec glim_gpu_env pgrep -f glim_rosbag >/dev/null 2>&1; then
  echo "ERROR: glim_rosbag が既に走っています。--stop で止めてから再実行してください。"; exit 1
fi

# claude: dump の親はコンテナ (root) が作るのでホストから書き込めない。
#   ログ類はホスト所有の bags/exp/glim_hq/ に置く (root からも書ける 775)。
LOGDIR="$REPO/bags/exp/glim_hq/$BAG"
CSV="$LOGDIR/${TAG}.resource.csv"
GLOG="$LOGDIR/${TAG}.glim.log"
HOST_DUMP="$REPO/bags/glim/${BAG}_dump/$TAG"
mkdir -p "$LOGDIR"

C_BAG="/workspace/bags/raw/$BAG"
C_DUMP="/workspace/bags/glim/${BAG}_dump/$TAG"
C_LOG="/workspace/bags/exp/glim_hq/$BAG/${TAG}.glim.log"

docker compose --profile glim_gpu up -d glim_gpu >/dev/null

# claude: 使う config ディレクトリは env CONFIG で選ぶ (既定 hq)。
#   例) CONFIG=hq3 ./scripts/glim_hq.sh <bag> hq3
CONFIG=${CONFIG:-hq}
[ -d "$REPO/ros2_ws_glim/config/$CONFIG" ] || { echo "ERROR: config が無い: ros2_ws_glim/config/$CONFIG"; exit 1; }
ARGS="-p config_path:=/glim_config/$CONFIG -p auto_quit:=true -p dump_path:=$C_DUMP"
[ "$DURATION" -gt 0 ] && ARGS="$ARGS -p playback_duration:=$DURATION.0"

# --- GLIM 本体 (デタッチ) -----------------------------------------------------
# auto_quit:=true なので bag を読み切ったら自分で保存して終了する。
docker exec -d glim_gpu_env /ros_entrypoint.sh \
  bash -c "ros2 run glim_ros glim_rosbag $C_BAG --ros-args $ARGS > $C_LOG 2>&1"

# --- 資源サンプラ (デタッチ) --------------------------------------------------
# プロセス名に glim_hq_sampler を含めて --stop で拾えるようにする。
# claude: 最終列は submap 数ではなく「直近の playback speed」。
#   GLIM は submap 挿入を info で出さないので数えられない (2026-10-14 実機ログで確認)。
#   速度の減衰こそ長時間走行で見たい量なので、こちらを記録する。
echo "elapsed_s,rss_mb,sys_used_mb,sys_avail_mb,vram_mb,playback_x" > "$CSV"
setsid nohup bash -c "
  # glim_hq_sampler
  t0=\$(date +%s)
  # claude: 生存判定は 3 回連続で失敗したときだけ終了とみなす。
  #   docker exec は負荷が高いと一時的に失敗することがあり、1 回で抜けると
  #   GLIM が走っているのにサンプラだけ死んで資源ログが欠ける (2026-10-15 に踏んだ)。
  miss=0
  while [ \$miss -lt 3 ]; do
    if docker exec glim_gpu_env pgrep -f glim_rosbag >/dev/null 2>&1; then miss=0; else miss=\$((miss+1)); sleep 10; continue; fi
    el=\$((\$(date +%s) - t0))
    rss=\$(ps -eo rss,comm --no-headers 2>/dev/null | awk '\$2==\"glim_rosbag\"{s+=\$1} END{printf \"%d\", s/1024}')
    read -r used avail <<<\"\$(free -m | awk '/^Mem:/{print \$3, \$7}')\"
    vram=\$(docker exec glim_gpu_env nvidia-smi --query-gpu=memory.used --format=csv,noheader,nounits 2>/dev/null | head -1)
    # ⚠️ grep -c は不一致でも '0' を出して終了コード 1 を返す。'|| echo 0' を付けると
    #    0 が二重に出て CSV に余分な行が混ざる (2026-10-14 に踏んだ)。tail -1 で 1 行に固定する。
    spd=\$(grep -o 'playback speed: [0-9.]*x' '$GLOG' 2>/dev/null | tail -1 | grep -o '[0-9.]*')
    echo \"\${el},\${rss:-0},\${used:-0},\${avail:-0},\${vram:-0},\${spd:-0}\" >> '$CSV'
    sleep 10
  done
" >/dev/null 2>&1 &

sleep 5
echo "[glim_hq] 起動しました (デタッチ実行 — この端末を閉じても走り続けます)"
echo "  bag      : $BAG  (duration=${DURATION:-全体})"
echo "  config   : /glim_config/$CONFIG"
echo "  dump     : $HOST_DUMP"
echo "  log      : $GLOG"
echo "  資源     : $CSV"
echo
echo "  状態確認 : ./scripts/glim_hq.sh --status $BAG $TAG"
echo "  停止     : ./scripts/glim_hq.sh --stop"
