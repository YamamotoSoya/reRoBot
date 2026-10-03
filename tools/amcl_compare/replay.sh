#!/bin/bash
# claude: 2026-10-01 新設。
# AMCL 並走比較の再生ハーネス (main コンテナ内で実行)。
#   bag 再生 (/clock 付き) → robot_state_publisher (静的 TF) → amcl_variants.launch.py (map_server 込み)
#   → /loc_*/amcl_pose と /scan_* を bag に記録 → DURATION 秒後に全停止。
# bag の /initialpose もそのまま流すので、変種 AMCL は実走と同じ瞬間・同じ姿勢で初期化される。
#
# 使い方 (docker exec rerobot_env bash /workspace/tools/amcl_compare/replay.sh ...):
#   [VARIANTS=<variants.yaml>] [STAGGER=1.1] replay.sh <bag_dir> <start_offset_s> <duration_s> <out_bag_dir> [only=base,near,...]
# 例:
#   replay.sh /workspace/bags/raw/2026-09-27_1948_5goukan_auto 190 120 /workspace/bags/exp/amcl_variants/1948_A
#
# 既知の癖:
#   - --start-offset 付きだと /tf_static が再生されないので launch 側の robot_state_publisher (start_robot_state_publisher) で URDF から出す。
#   - 1948 の map→odom (記録 AMCL の TF) は 190 s 以降にしか無いが、変種 AMCL は TF を出さず内部で持つだけなので無関係。
#   - AMCL が初期姿勢受信時に「Failed to transform initial pose in time」と警告することがあるが、
#     その場合は odom 補正なし (identity) で姿勢を設定して続行する (nav2_amcl の仕様)。静止中なら実害なし。
source /opt/ros/jazzy/setup.bash; source /workspace/install/setup.bash   # set -u より前 (setup.bash が未定義変数を参照する)
set -u
BAG=${1:?bag dir}; START=${2:-0}; DUR=${3:-120}; OUT=${4:?out dir}; ONLY=${5:-}
URDF=$(ros2 pkg prefix rerobot_bringup)/share/rerobot_bringup/urdf/rerobot.urdf
LOG=${OUT}.log
mkdir -p "$(dirname "$OUT")"; rm -rf "$OUT"
pkill -TERM -f "ros2 bag pla[y]" 2>/dev/null; pkill -TERM -f "ros2 bag recor[d]" 2>/dev/null; pkill -TERM -f "robot_state_publishe[r]" 2>/dev/null
pkill -f "nav2_amc[l]|/amc[l] |lifecycle_manage[r]|map_serve[r]|scan_mode[s]|pointcloud_to_laserscan_nod[e]" 2>/dev/null; sleep 1
cleanup() {
  # ros2 bag play/record は TTY 無しだと SIGINT を無視するので SIGTERM で止める (record は TERM でも mcap を finalize する)
  pkill -TERM -f "ros2 bag recor[d]" 2>/dev/null
  for _ in $(seq 1 20); do pgrep -f "ros2 bag recor[d]" >/dev/null || break; sleep 0.5; done
  pkill -TERM -f "ros2 bag pla[y]" 2>/dev/null; kill -INT ${LP:-0} 2>/dev/null; pkill -TERM -f "robot_state_publishe[r]" 2>/dev/null; pkill -TERM -f "initialpose_rela[y]" 2>/dev/null; sleep 4
  pkill -TERM -f "nav2_amc[l]|/amc[l] |lifecycle_manage[r]|map_serve[r]|scan_mode[s]|pointcloud_to_laserscan_nod[e]" 2>/dev/null
}
trap cleanup EXIT
echo "[replay] bag=$BAG start=$START dur=$DUR out=$OUT only=${ONLY:-all}" | tee "$LOG"
ONLY_ARG=(); [ -n "$ONLY" ] && ONLY_ARG=("only:=$ONLY")   # 空の only:= は launch が拒否する
[ -n "${VARIANTS:-}" ] && ONLY_ARG+=("variants_file:=$VARIANTS")   # env VARIANTS=<yaml> で変種定義を差し替え
[ -n "${MAP_YAML:-}" ] && ONLY_ARG+=("map_yaml:=$MAP_YAML")   # claude: 2026-10-04 env MAP_YAML=<yaml> で再生用 /map を差し替え (既定は launch 側の旧 dumpbase)
ros2 launch rerobot_bringup amcl_variants.launch.py use_sim_time:=true start_map_server:=true start_robot_state_publisher:=true stagger:=${STAGGER:-0} "${ONLY_ARG[@]}" >> "$LOG" 2>&1 &
LP=$!
# 全 manager の活性化を待つ (stagger 時は manager が AMCL ごとに 1 つ)
NM=""; for i in $(seq 1 120); do NM=$(grep -o "\[variants\] managers=[0-9]*" "$LOG" | head -1 | cut -d= -f2); [ -n "$NM" ] && [ "$(grep -c "\(lcm_\|lifecycle_manager_variants\).*Managed nodes are active" "$LOG")" -ge "$NM" ] && break; sleep 1; done
echo "[replay] variants active ($(grep -c "\(lcm_\|lifecycle_manager_variants\).*Managed nodes are active" "$LOG")/${NM:-?} managers) after ${i} s" | tee -a "$LOG"
echo "[replay] AMCL configure 秒 (乱数種): $(grep -E "loc_[a-z0-9_]+\.amcl\]: Configuring" "$LOG" | grep -o "\[[0-9]\{10\}" | sort | uniq -c | awk '{printf "%s×%s ", $2, $1}')" | tee -a "$LOG"
# relay も bag より先に (bag の最初の /initialpose を受け損ねないため)
python3 /workspace/tools/amcl_compare/initialpose_relay.py >> "$LOG" 2>&1 &
# 記録を先に始める (bag の /initialpose を取りこぼさないため)
ros2 bag record -o "$OUT" --use-sim-time -e "/loc_.*/amcl_pose|/scan_.*|/initialpose" >/dev/null 2>&1 &
sleep 2
ros2 bag play "$BAG" --clock 100 --rate 1.0 --start-offset "$START" --disable-keyboard-controls \
  --topics /scan /odometry/filtered /odom /tf /imu/data /rfans_driver/rfans_points /initialpose \
  --remap /initialpose:=/initialpose_bag >/dev/null 2>&1 &
sleep 4
sleep "$DUR"
echo "[replay] done. fill-rate logs:" | tee -a "$LOG"; grep -h "bin fill" "$LOG" | tail -n 6
grep -c "Setting pose" "$LOG" | sed 's/^/[replay] AMCL Setting pose count (all variants): /'
