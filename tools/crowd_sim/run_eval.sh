#!/bin/bash
# claude: 2026-10-24 — 人垣試験用の再生評価 1 本 (main コンテナ内)。
#   bag に /rfans/scan が無い (seg1 由来) ので、pointcloud_to_laserscan ×2 (/rfans/scan + /rfans/scan_all) を
#   use_sim_time で立ててから tools/loc_eval/replay_one.sh に渡す。帯は env で変えられる (高帯試験用)。
#
#   run_eval.sh <bag> <run_dir> <amcl|emcl2|lidar3d> <ref_dir> [launch 引数...]
#   env: SCAN_MIN=0.3 SCAN_MAX=2.0 SCANALL_MIN=0.3 SCANALL_MAX=100.0 (+ replay_one.sh の RATE/START/DUR/INIT_DELAY)
source /opt/ros/jazzy/setup.bash; source /workspace/install/setup.bash
set -u
BAG=${1:?bag}; RUN=${2:?run dir}; LOC=${3:?loc}; REF=${4:?ref}; shift 4
SCAN_MIN=${SCAN_MIN:-0.3}; SCAN_MAX=${SCAN_MAX:-2.0}; SCANALL_MIN=${SCANALL_MIN:-0.3}; SCANALL_MAX=${SCANALL_MAX:-100.0}
mkdir -p "$RUN"
pkill -TERM -f "rfans_to_sca[n]" 2>/dev/null; sleep 1
COMMON=(-p use_sim_time:=true -p target_frame:=base_link -p transform_tolerance:=0.1
        -p angle_min:=-3.14159265 -p angle_max:=3.14159265 -p scan_time:=0.1
        -p range_min:=0.5 -p range_max:=150.0 -p use_inf:=true)
ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node --ros-args -r __node:=rfans_to_scan \
  -r cloud_in:=/rfans_driver/rfans_points -r scan:=/rfans/scan "${COMMON[@]}" \
  -p min_height:=$SCAN_MIN -p max_height:=$SCAN_MAX -p angle_increment:=0.0035 > "$RUN/scan_node.log" 2>&1 &
ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node --ros-args -r __node:=rfans_to_scan_all \
  -r cloud_in:=/rfans_driver/rfans_points -r scan:=/rfans/scan_all "${COMMON[@]}" \
  -p min_height:=$SCANALL_MIN -p max_height:=$SCANALL_MAX -p angle_increment:=0.00016 > "$RUN/scan_all_node.log" 2>&1 &
sleep 2
echo "[run_eval] scan bands: /rfans/scan $SCAN_MIN..$SCAN_MAX, /rfans/scan_all $SCANALL_MIN..$SCANALL_MAX" | tee "$RUN/run_eval.txt"
TOPICS="/tf /tf_static /rfans_driver/rfans_points /imu/data /odom" \
  bash /workspace/tools/loc_eval/replay_one.sh "$BAG" "$RUN" "$LOC" "$REF" "$@"
rc=$?
pkill -TERM -f "rfans_to_sca[n]" 2>/dev/null; sleep 1
exit $rc
