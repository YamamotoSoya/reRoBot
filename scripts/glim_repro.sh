#!/usr/bin/env bash
# claude: 2026-10-23 作成。追従喪失の最小再現。
#   bag 2400-2550 s (150 秒) だけを処理し、odom 段の走行距離を出す。
#   車輪オドメトリの実距離 = 75.0 m。通常密度は約 73 m、ultra は大きく下回るはず。
#   使い方: ./scripts/glim_repro.sh <config ディレクトリ名 | - > <タグ>
#           '-' は /glim_config (GPU 既定 = 通常密度)
set -eu
cd "$(dirname "$0")/.."
BAG=2026-10-22_1622_shinnarachallenge
CFG=${1:?使い方: $0 <config|-> <タグ>}; TAG=${2:?使い方: $0 <config|-> <タグ>}
P=/glim_config; [ "$CFG" != "-" ] && P=/glim_config/$CFG
D=/workspace/bags/glim/${BAG}_dump/repro_$TAG
docker exec glim_gpu_env rm -rf $D
docker exec glim_gpu_env /ros_entrypoint.sh bash -c \
  "ros2 run glim_ros glim_rosbag /workspace/bags/raw/$BAG --ros-args -p config_path:=$P \
   -p auto_quit:=true -p start_offset:=2400.0 -p playback_duration:=150.0 -p dump_path:=$D \
   > /workspace/bags/exp/glim_hq/$BAG/repro_$TAG.log 2>&1" || echo "WARN: 非ゼロ終了"
docker exec -i glim_gpu_env python3 -c "
import numpy as np,os,glob,sys
d='$D'
T=np.loadtxt(os.path.join(d,'odom_lidar.txt'))
L=float(np.linalg.norm(np.diff(T[:,1:4],axis=0),axis=1).sum())
n=sum(os.path.getsize(f) for f in glob.glob(d+'/0*/points_compact.bin'))//12
print(f'$TAG: odom 距離 {L:6.1f} m  (実距離 75.0 m の {L/75.0*100:3.0f}%)  / 点 {n:,}')
"
