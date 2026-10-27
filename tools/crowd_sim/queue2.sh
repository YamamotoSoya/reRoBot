#!/bin/bash
# claude: 2026-10-24 (2) — critic 査読後の追試 (main コンテナ内、nohup)。
#   (1) 近距離人垣 bag_crowd_near (1.5〜8 m、108 人、遠方ビームの 79 % 遮蔽) × initialpose を横 0.7 m・yaw 10° ずらした ref_off
#   (2) 人垣なし × ずらし (ずらしだけの効果)、(3) 近距離人垣 × 正解 initialpose (人垣だけの効果)、(4) lidar3d 同条件
#   (5) prewarm 偽完了 (idle after 5 s) だった 4 run の再実行、(6) likelihood_field_prob 単独 (beam skip なし) で bskip の交絡を切る
source /opt/ros/jazzy/setup.bash; source /workspace/install/setup.bash
set -u
E=/workspace/bags/exp/2026-10-24_crowd_sim
M=/workspace/maps/2d/glim/2026-10-03_1126_tsukuba_composite
M3=/workspace/maps/3d/2026-10-03_1126_tsukuba_composite_startcrop
RUNS=$E/runs; mkdir -p $RUNS
AMCL_ARGS=(scan_topic:=/rfans/scan_all max_beams:=2000)
LOC03=loc_map_yaml:=$M/raw_traj/0.3-100.0_r100/map.yaml
LOC20=loc_map_yaml:=$M/raw_traj/2.0-100.0_r100_seg1/map.yaml
BS=params_file:=/workspace/tools/loc_eval/variants/amcl_beamskip.yaml
PROB=params_file:=/workspace/tools/loc_eval/variants/amcl_probonly.yaml
L3=(map_path:=$M3/map.pcd traj_path:=$M3/traj_lidar.txt odom_topic:=/odom)
log() { echo "[queue2 $(date +%H:%M:%S)] $*" | tee -a $E/queue.log; }
run() {  # run <name> <bag> <loc> <ref_dir> [env KEY=VAL ...] -- [launch args]
  local name=$1 bag=$2 loc=$3 ref=$4; shift 4
  local envs=()
  while [ $# -gt 0 ] && [ "$1" != "--" ]; do envs+=("$1"); shift; done
  [ "${1:-}" = "--" ] && shift
  if [ -s $RUNS/$name/est.tum ]; then log "skip $name (exists)"; return; fi
  log "start $name"
  env PREWARM=$([ "$loc" = amcl ] && echo 1 || echo 0) "${envs[@]}" bash /workspace/tools/crowd_sim/run_eval.sh $E/$bag $RUNS/$name $loc $E/$ref "$@" > $RUNS/$name.out 2>&1
  log "end $name rc=$? rows=$(wc -l < $RUNS/$name/est.tum 2>/dev/null) init=$(grep -c init_ok $RUNS/$name/events.txt 2>/dev/null) prewarm=$(grep -o 'idle after [0-9]* s' $RUNS/$name/replay.log | head -1)"
  sleep 2
}
run near_amcl_off_r1  bag_crowd_near amcl ref_off -- "${AMCL_ARGS[@]}" $LOC03
run base_amcl_off_r1  bag_base       amcl ref_off -- "${AMCL_ARGS[@]}" $LOC03
run near_amcl_r1      bag_crowd_near amcl ref     -- "${AMCL_ARGS[@]}" $LOC03
run near_lidar3d_off_r1 bag_crowd_near lidar3d ref_off -- "${L3[@]}"
run base_lidar3d_off_r1 bag_base       lidar3d ref_off -- "${L3[@]}"
run near_amcl_off_r2  bag_crowd_near amcl ref_off -- "${AMCL_ARGS[@]}" $LOC03
run base_amcl_off_r2  bag_base       amcl ref_off -- "${AMCL_ARGS[@]}" $LOC03
run near_amcl_off_r3  bag_crowd_near amcl ref_off -- "${AMCL_ARGS[@]}" $LOC03
run near_amcl_r2      bag_crowd_near amcl ref     -- "${AMCL_ARGS[@]}" $LOC03
# prewarm 偽完了の再実行 (古い結果は _bad に退避済み)
run crowd_hi20_r1  bag_crowd amcl ref SCANALL_MIN=2.0 -- "${AMCL_ARGS[@]}" $LOC20
run crowd_hi20_r3  bag_crowd amcl ref SCANALL_MIN=2.0 -- "${AMCL_ARGS[@]}" $LOC20
run base_hi20_r2   bag_base  amcl ref SCANALL_MIN=2.0 -- "${AMCL_ARGS[@]}" $LOC20
run crowd_bskip_r3 bag_crowd amcl ref -- "${AMCL_ARGS[@]}" $LOC03 $BS
# prob モデル単独
run base_prob_r1 bag_base amcl ref -- "${AMCL_ARGS[@]}" $LOC03 $PROB
run base_prob_r2 bag_base amcl ref -- "${AMCL_ARGS[@]}" $LOC03 $PROB
log "scoring"
python3 /workspace/tools/loc_eval/score.py $E/ref "$RUNS/*_r*" --out $E/score > $E/score.log 2>&1
log "ALL DONE 2"
