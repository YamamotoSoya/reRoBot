#!/bin/bash
# claude: 2026-10-11 新設 — 自己位置推定 1 構成 × bag 1 本の再生評価 (main コンテナ内で実行)。
#
#   localization 部品 (loc_amcl / loc_emcl2 / loc_lidar3d) を use_sim_time で起動
#     → 活性化を待つ → record_pose.py (TF map→base_link 記録 + /initialpose 送信) → bag 再生 (/clock)
#     → 再生終了で全停止。結果は <run_dir>/{est.tum,cov.txt,alpha.txt,alignment.jsonl,events.txt,replay.log,meta.json}
#
# 使い方:
#   replay_one.sh <bag_dir> <run_dir> <amcl|emcl2|lidar3d> <ref_dir> [launch 引数...]
#   例 (AMCL × 現行 nav2 地図):
#     bash /workspace/tools/loc_eval/replay_one.sh /workspace/bags/raw/2026-10-09_1643_5goukan_chal \
#       /workspace/bags/exp/2026-10-11_loc_eval/1643/amcl_dump0217_r1 amcl /workspace/bags/exp/2026-10-11_loc_eval/ref_1643 \
#       loc_map_yaml:=/workspace/maps/2d/glim/2026-10-09_1643_5goukan_chal/raw_dump/0.2-1.7/map.yaml scan_topic:=/rfans/scan
#   env: RATE=1.0 (再生倍率) START=0 (開始オフセット s) DUR= (再生秒数、空=最後まで) INIT_DELAY=3 (initialpose までの秒)
#        TOPICS="..." (再生 topic、既定は下記)
#
# 既知の癖 (amcl_compare/replay.sh と同じ):
#   - ros2 bag play は TTY なしだと SIGINT を無視する → SIGTERM で止める
#   - --start-offset 付きだと /tf_static が再生されない → 先に /tf_static だけ短く再生する (START>0 のとき)
#   - AMCL の乱数種は configure した秒 (time(NULL)) → 反復は直列で 1 s 以上ずらす (meta.json に記録)
source /opt/ros/jazzy/setup.bash; source /workspace/install/setup.bash   # set -u より前
set -u
BAG=${1:?bag dir}; RUN=${2:?run dir}; LOC=${3:?amcl|emcl2|lidar3d}; REF=${4:?ref dir (initialpose.yaml)}; shift 4
RATE=${RATE:-1.0}; START=${START:-0}; DUR=${DUR:-}; INIT_DELAY=${INIT_DELAY:-3}
TOPICS=${TOPICS:-"/tf /tf_static /rfans/scan /rfans_driver/rfans_points /imu/data /odom"}
HERE=$(cd "$(dirname "$0")" && pwd)
mkdir -p "$RUN"; LOG=$RUN/replay.log; : > "$LOG"
PAT='nav2_amc[l]|/amc[l] |emcl2_nod[e]|lidar_localization_nod[e]|start_lifecycle_nod[e]|map_level_bridg[e]|lifecycle_manage[r]|map_serve[r]|record_pos[e]'
pkill -TERM -f "ros2 bag pla[y]" 2>/dev/null; pkill -TERM -f "$PAT" 2>/dev/null; sleep 1
cleanup() {
  pkill -TERM -f "ros2 bag pla[y]" 2>/dev/null
  pkill -TERM -f "record_pos[e]" 2>/dev/null; sleep 1
  kill -INT ${LP:-0} 2>/dev/null; sleep 3
  pkill -TERM -f "$PAT" 2>/dev/null; sleep 1
  pkill -KILL -f "$PAT" 2>/dev/null
}
trap cleanup EXIT
log() { echo "[replay $(date +%H:%M:%S)] $*" | tee -a "$LOG"; }
log "bag=$BAG run=$RUN loc=$LOC rate=$RATE start=$START dur=${DUR:-all} args=$*"

case "$LOC" in
  amcl)    LAUNCH=loc_amcl.launch.py;    READY='Managed nodes are active'; NREADY=2 ;;  # loc_map + amcl の 2 manager
  emcl2)   LAUNCH=loc_emcl2.launch.py;   READY='Managed nodes are active'; NREADY=1 ;;  # loc_map のみ (emcl2 は非 lifecycle)
  lidar3d) LAUNCH=loc_lidar3d.launch.py; READY='lidar_localization: active'; NREADY=1 ;;
  *) echo "unknown loc: $LOC"; exit 2 ;;
esac
T_LAUNCH=$(date +%s)
ros2 launch rerobot_bringup "$LAUNCH" use_sim_time:=true "$@" >> "$LOG" 2>&1 &
LP=$!
for i in $(seq 1 180); do
  [ "$(grep -c "$READY" "$LOG")" -ge "$NREADY" ] && break
  kill -0 $LP 2>/dev/null || { log "launch died"; exit 3; }
  sleep 1
done
[ "$(grep -c "$READY" "$LOG")" -ge "$NREADY" ] || { log "not ready after ${i} s"; exit 4; }
log "localizer ready after ${i} s"
[ "$LOC" = emcl2 ] && sleep 2   # emcl2 本体の起動待ち

if [ "$START" != 0 ]; then
  ros2 bag play "$BAG" --topics /tf_static --playback-duration 1 --disable-keyboard-controls >/dev/null 2>&1
fi
DUR_ARG=(); [ -n "$DUR" ] && DUR_ARG=(--playback-duration "$DUR")
# claude: 2026-10-24 PREWARM=1 — AMCL は最初のスキャンで尤度場を作る間 (つくば composite 地図 280 M セルで約 30 s)
#   コールバックを抜けないので、その間に bag が進むと initialpose が効くのは出発後になる (人垣試験で 8 m 先だった)。
#   --start-paused で開始し play_next で最初の点群 1 枚だけ流し、`ros2 param get` が返る (= 単一スレッド executor が
#   空いた = 尤度場完成) のを待ってから record_pose を起動して resume する。時刻は飛ばない (play_next は再生時刻を進めるだけ)。
#   PREWARM_PROC (既定 nav2_amcl/amcl) で待つプロセスを変えられる。emcl2 / lidar3d では不要 (地図の前処理は起動時)。
PAUSED=(); [ "${PREWARM:-0}" = 1 ] && PAUSED=(--start-paused)
# shellcheck disable=SC2086
# QoS 上書き: bag の /rfans/scan は best_effort 記録で emcl2 (reliable 購読) に届かない → reliable で再生 (qos_override.yaml)
ros2 bag play "$BAG" --clock 100 --rate "$RATE" --start-offset "$START" "${DUR_ARG[@]}" --disable-keyboard-controls \
  "${PAUSED[@]}" --qos-profile-overrides-path "$HERE/qos_override.yaml" --topics $TOPICS >> "$LOG" 2>&1 &
PP=$!
if [ "${PREWARM:-0}" = 1 ]; then
  #   10-24 (2): 待つのは「localizer の入力 topic が実際に出た」こと (既定 /rfans/scan_all = 点群→scan 変換後)。点群が届いただけでは
  #   変換ノードが TF 待ちで止まっていることがあり、AMCL が何もしていない静けさを完了と誤判定した (near_amcl_off_r1)。
  PREWARM_TOPIC=${PREWARM_TOPIC:-/rfans/scan_all}; PREWARM_MSG=${PREWARM_MSG:-sensor_msgs/msg/LaserScan}
  for i in $(seq 1 60); do ros2 service list 2>/dev/null | grep -q /rosbag2_player/play_next && break; sleep 1; done
  python3 - "$PREWARM_TOPIC" "$PREWARM_MSG" <<'PY' >> "$LOG" 2>&1
import sys, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.utilities import get_message
from rosbag2_interfaces.srv import PlayNext
rclpy.init(); n = Node("prewarm_step"); got = []
n.create_subscription(get_message(sys.argv[2]), sys.argv[1], lambda m: got.append(1), qos_profile_sensor_data)
cli = n.create_client(PlayNext, "/rosbag2_player/play_next"); cli.wait_for_service(timeout_sec=30.0)
k = 0
while not got and k < 3000:
    fut = cli.call_async(PlayNext.Request()); rclpy.spin_until_future_complete(n, fut, timeout_sec=5.0); k += 1
    rclpy.spin_once(n, timeout_sec=0.02)
for _ in range(20):
    if got: break
    rclpy.spin_once(n, timeout_sec=0.1)
print(f"[prewarm] stepped {k} messages, {sys.argv[1]} received={bool(got)}")
n.destroy_node(); rclpy.shutdown()
PY
  # 尤度場の完成待ち: AMCL の parameter サービスは別スレッドで応答してしまう (10-24 実測) ので、プロセスの CPU 時間が
  #   2 s 連続でほぼ増えなくなる (= コールバックを抜けて待機) ことを合図にする。PREWARM_PROC = pgrep -f のパターン。
  PREWARM_PROC=${PREWARM_PROC:-nav2_amcl/amcl}
  #   ⚠ 10-24 critic: 点群が届く前の静かな 5 s を「完了」と誤判定した run が 4 本あった → 一度 busy (≥50 tick/s) を見てから
  #   idle 2 s で完了とする。20 s 待っても busy が来なければ (小さい地図で一瞬で終わった等) そのまま進む。
  PWPID=$(pgrep -f "$PREWARM_PROC" | head -1); T0=$(date +%s); idle=0; busy=0
  start=$(awk '{print $14+$15}' /proc/$PWPID/stat 2>/dev/null || echo 0); prev=$start
  for i in $(seq 1 300); do
    sleep 1; cur=$(awk '{print $14+$15}' /proc/$PWPID/stat 2>/dev/null || echo 0)
    d=$((cur - prev)); prev=$cur
    [ $d -ge 50 ] && busy=1
    if [ $d -lt 20 ]; then idle=$((idle + 1)); else idle=0; fi
    # 完了 = busy を見た後に idle 2 s、かつ合計 CPU ≥ 3 s (scan 受信程度の短い busy を完了と取らない)
    [ $busy = 1 ] && [ $idle -ge 2 ] && [ $((cur - start)) -ge 300 ] && break
    [ $busy = 0 ] && [ $i -ge 30 ] && { log "prewarm: no busy phase seen in 30 s (small map?)"; break; }
  done
  log "prewarm: $PREWARM_PROC (pid $PWPID) idle after $(( $(date +%s) - T0 )) s (busy seen=$busy)"
fi
python3 "$HERE/record_pose.py" "$RUN" --init "$REF/initialpose.yaml" --initialpose_delay "$INIT_DELAY" >> "$LOG" 2>&1 &
sleep 2
if [ "${PREWARM:-0}" = 1 ]; then
  ros2 service call /rosbag2_player/resume rosbag2_interfaces/srv/Resume >/dev/null 2>&1
  log "prewarm: resumed"
fi
log "bag play started (pid $PP)"
wait $PP
log "bag play finished"
sleep 2
N=$(wc -l < "$RUN/est.tum" 2>/dev/null || echo 0)
cat > "$RUN/meta.json" <<EOF
{"bag": "$BAG", "loc": "$LOC", "launch_args": "$*", "rate": $RATE, "start": $START, "dur": "${DUR}",
 "init_delay": $INIT_DELAY, "topics": "$TOPICS", "launch_epoch": $T_LAUNCH, "est_rows": $N,
 "git_head": "$(git -C /workspace rev-parse --short HEAD 2>/dev/null || echo unknown)"}
EOF
log "done: est rows=$N"
