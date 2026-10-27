#!/bin/bash
# claude: 2026-10-24 — 人垣試験の一括実行 (main コンテナ内、nohup で背景実行)。
#   bag_base (人垣なし) / bag_crowd (人垣あり) × {AMCL 現行, AMCL beam skip, AMCL 高帯 2.0 m, lidar3d} を直列に回し、
#   score.py で採点する。AMCL は乱数で run ごとに変わるので反復 (base 2 本、crowd 3 本)。
#   順番: base/crowd の AMCL 系 → lidar3d → (高帯地図ができていれば) 高帯。途中で止めるなら pkill -f queue.sh + replay 群。
source /opt/ros/jazzy/setup.bash; source /workspace/install/setup.bash   # set -u より前 (setup.bash が未定義変数を参照する)
set -u
E=/workspace/bags/exp/2026-10-24_crowd_sim
M=/workspace/maps/2d/glim/2026-10-03_1126_tsukuba_composite
M3=/workspace/maps/3d/2026-10-03_1126_tsukuba_composite_startcrop
REF=$E/ref
RUNS=$E/runs; mkdir -p $RUNS
AMCL_ARGS=(scan_topic:=/rfans/scan_all max_beams:=2000)
LOC03=loc_map_yaml:=$M/raw_traj/0.3-100.0_r100/map.yaml
LOC20=loc_map_yaml:=$M/raw_traj/2.0-100.0_r100_seg1/map.yaml
BS=params_file:=/workspace/tools/loc_eval/variants/amcl_beamskip.yaml
log() { echo "[queue $(date +%H:%M:%S)] $*" | tee -a $E/queue.log; }
run() {  # run <name> <bag> <loc> [env KEY=VAL ...] -- [launch args]
  local name=$1 bag=$2 loc=$3; shift 3
  local envs=()
  while [ $# -gt 0 ] && [ "$1" != "--" ]; do envs+=("$1"); shift; done
  [ "${1:-}" = "--" ] && shift
  if [ -s $RUNS/$name/est.tum ]; then log "skip $name (exists)"; return; fi
  log "start $name"
  env PREWARM=$([ "$loc" = amcl ] && echo 1 || echo 0) "${envs[@]}" bash /workspace/tools/crowd_sim/run_eval.sh $E/$bag $RUNS/$name $loc $REF "$@" > $RUNS/$name.out 2>&1
  log "end $name rc=$? rows=$(wc -l < $RUNS/$name/est.tum 2>/dev/null) init=$(grep -c init_ok $RUNS/$name/events.txt 2>/dev/null)"
  sleep 2
}
until grep -q DONE $E/inject.log 2>/dev/null; do sleep 20; done
log "bags ready"

# --- AMCL 現行 (scan_all 0.3-100, alpha 0.02, max_beams 2000) ---
run base_amcl_r1   bag_base  amcl -- "${AMCL_ARGS[@]}" $LOC03
run crowd_amcl_r1  bag_crowd amcl -- "${AMCL_ARGS[@]}" $LOC03
run crowd_amcl_r2  bag_crowd amcl -- "${AMCL_ARGS[@]}" $LOC03
# --- AMCL beam skip ---
run crowd_bskip_r1 bag_crowd amcl -- "${AMCL_ARGS[@]}" $LOC03 $BS
run crowd_bskip_r2 bag_crowd amcl -- "${AMCL_ARGS[@]}" $LOC03 $BS
run base_bskip_r1  bag_base  amcl -- "${AMCL_ARGS[@]}" $LOC03 $BS
# --- lidar3d (決定論なので 1 本ずつ) ---
run base_lidar3d_r1  bag_base  lidar3d -- map_path:=$M3/map.pcd traj_path:=$M3/traj_lidar.txt odom_topic:=/odom
run crowd_lidar3d_r1 bag_crowd lidar3d -- map_path:=$M3/map.pcd traj_path:=$M3/traj_lidar.txt odom_topic:=/odom
# --- 反復の残り ---
run base_amcl_r2   bag_base  amcl -- "${AMCL_ARGS[@]}" $LOC03
run crowd_amcl_r3  bag_crowd amcl -- "${AMCL_ARGS[@]}" $LOC03
run crowd_bskip_r3 bag_crowd amcl -- "${AMCL_ARGS[@]}" $LOC03 $BS
run base_bskip_r2  bag_base  amcl -- "${AMCL_ARGS[@]}" $LOC03 $BS
# --- 高帯 2.0-100 (地図 + scan_all の帯を揃える) ---
until grep -q DONE $E/map_hi20.log 2>/dev/null; do sleep 30; done
log "hi20 map ready"
run crowd_hi20_r1 bag_crowd amcl SCANALL_MIN=2.0 -- "${AMCL_ARGS[@]}" $LOC20
run base_hi20_r1  bag_base  amcl SCANALL_MIN=2.0 -- "${AMCL_ARGS[@]}" $LOC20
run crowd_hi20_r2 bag_crowd amcl SCANALL_MIN=2.0 -- "${AMCL_ARGS[@]}" $LOC20
run crowd_hi20_r3 bag_crowd amcl SCANALL_MIN=2.0 -- "${AMCL_ARGS[@]}" $LOC20
run base_hi20_r2  bag_base  amcl SCANALL_MIN=2.0 -- "${AMCL_ARGS[@]}" $LOC20

log "scoring"
python3 /workspace/tools/loc_eval/score.py $REF "$RUNS/*_r*" --out $E/score > $E/score.log 2>&1
log "ALL DONE"
