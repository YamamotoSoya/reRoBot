#!/usr/bin/env bash
# claude: 各 workspace のビルド。usage: scripts/build.sh [main|slamtoolbox|liosam|images]
# - colcon build は必ずコンテナ内で実行する (ホストは humble で API 不一致 + 成果物が git を汚す)。
# - PC が落ちるため並列度を制限する: make は BUILD_JOBS 並列 (既定 2)、colcon は 1 パッケージずつ。
#   速くしたいときは BUILD_JOBS=4 ./scripts/build.sh main のように上書き (落ちたら下げる)。
# - main の sequential executor は canopen の並列ビルド破損対策 (従来からの必須制約)。
# - CMAKE_EXPORT_COMPILE_COMMANDS=ON は clangd / Dev Containers での参照解決用。
set -eu
cd "$(dirname "$0")/.."
ws="${1:-main}"
BUILD_JOBS="${BUILD_JOBS:-2}"

case "$ws" in
  main)
    docker compose up -d main
    docker exec -e MAKEFLAGS="-j${BUILD_JOBS}" -it rerobot_env bash -c \
      'source /opt/ros/jazzy/setup.bash && cd /workspace && \
       nice -n 10 colcon build --symlink-install --executor sequential \
         --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON'
    ;;
  slamtoolbox)
    docker compose --profile slamtoolbox up -d slamtoolbox
    docker exec -e MAKEFLAGS="-j${BUILD_JOBS}" -it slamtoolbox_env bash -c \
      'source /opt/ros/jazzy/setup.bash && cd /workspace && \
       nice -n 10 colcon build --symlink-install --executor sequential'
    ;;
  liosam)
    docker compose --profile liosam up -d liosam
    docker exec -e MAKEFLAGS="-j${BUILD_JOBS}" -it liosam_env bash -c \
      'source /opt/ros/jazzy/setup.bash && cd /workspace && \
       nice -n 10 colcon build --symlink-install --executor sequential'
    ;;
  images)
    # claude: Docker イメージの (再) ビルド。複数サービス同時だと PC が落ちるため 1 本ずつ直列。
    # glim は公式イメージベースの薄い層 (Dockerfile_glim) のみビルドする。
    export COMPOSE_PARALLEL_LIMIT=1
    for svc in main slamtoolbox glim liosam; do
      echo "[build] image: $svc"
      docker compose --profile slamtoolbox --profile glim --profile liosam build "$svc"
    done
    ;;
  glim_gpu)
    # claude: GLIM の GPU 版イメージ (2026-10-07)。CUDA ビルド版の公式イメージ (約 4.4 GB の pull)
    # を土台にするため、GPU 機でのみ明示的に建てる。上の images ループには意図的に入れていない
    # — GPU を持たない PC が巻き込まれて巨大イメージを引かないようにするため。
    export COMPOSE_PARALLEL_LIMIT=1
    docker compose --profile glim_gpu build glim_gpu
    ;;
  *)
    echo "usage: $0 [main|slamtoolbox|liosam|images|glim_gpu]"
    echo "  images   : main/slamtoolbox/glim/liosam のイメージを直列ビルド (CPU 構成。全 PC 共通)"
    echo "  glim_gpu : GLIM の GPU 版イメージ (CUDA。GPU 機のみ。images には含まれない)"
    echo "  (glim の colcon build は不要 — イメージは images に含む)"
    exit 1
    ;;
esac
echo "[build] $ws: 完了"
