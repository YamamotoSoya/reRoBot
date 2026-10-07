<!-- claude: 運用手引き 第2章 テンプレ (2026-09-22) -->

# 第2章 初回セットアップ

---

## 0. 取得

submodule 込みで clone する (`--recursive` を忘れると drivers/ が空になる)。

```
git clone --recursive https://github.com/YamamotoSoya/reRoBot.git
cd reRoBot
```

clone 済みで submodule が空だったとき:

```
git submodule update --init --recursive
```

## 1. コンテナを建てる

GUI を出すため、先にホストで X への接続を許可する。

```
xhost +local:docker
docker compose up -d main     # 常用コンテナ (profile なし)
```

他のコンテナは用途のときだけ profile 付きで起動する。

| コンテナ | 起動 | 用途 |
|---|---|---|
| `rerobot_env` | `docker compose up -d main` | CAN モータ制御・センサ・Nav2・teleop |
| `slamtoolbox_env` | `docker compose --profile slamtoolbox up -d slamtoolbox` | 2D 地図作成 |
| `glim_env` | `docker compose --profile glim up -d glim` | 3D SLAM GLIM (CPU) |
| `glim_gpu_env` | `docker compose --profile glim_gpu up -d glim_gpu` | 3D SLAM GLIM (GPU) |
| `liosam_env` | `docker compose --profile liosam up -d liosam` | LIO-SAM (凍結中) |

## 2. イメージをビルドする

初回、または Dockerfile を変更したとき。

```
./scripts/build.sh images       # main / slamtoolbox / glim / liosam を直列ビルド
```

⚠️ `docker compose build` を引数なしで直接叩かない。3 イメージが並列で走り、このマシンは落ちる。`build.sh` は 1 本ずつ直列に建てるようにしてある。

### GPU 版 GLIM (GPU 搭載機のみ)

`images` には含まれていない。CUDA 版の公式イメージが約 4.4 GB あり、GPU を持たない PC が巻き込まれないように分けてある。

```
./scripts/build.sh glim_gpu
```

- ディスクを食う。pull は約 4.4 GB、**展開後のイメージは 15.6 GB** (CPU 版 5.5 GB の約 3 倍。2026-10-07 実測)。空きを `df -h /` で先に確認する。所要は回線によるが、実測で pull 約 2 分 + 展開を含めて計 5 分ほど。
- 前提: ホストに NVIDIA ドライバと nvidia-container-toolkit。`nvidia-smi` が通ること。
- ⚠️ CUDA 13 は Pascal 世代 (GTX 1080 等、compute capability 6.1) のサポートを削除済み。ベースイメージは `jazzy_cuda12.5` を使う (指定は `docker-compose.yml` の `glim_gpu` サービス内)。
- GPU で「描画」だけを行う設定 (CPU 版コンテナの offline_viewer 用) は `docker-compose.override.yml` に置く。このファイルは git 管理外で、GPU なし PC には**置かない** (置くとコンテナ作成が `could not select device driver` で失敗する)。

ビルド後の確認:

```
docker exec glim_gpu_env ls /root/ros2_ws/install/glim/lib | grep gpu
# → libodometry_estimation_gpu.so が出れば GPU 版
```

## 3. ワークスペースをビルドする

`colcon build` は**必ずコンテナ内**で行う。ホストは ROS 2 Humble で API が合わず、成果物が git を汚す。`build.sh` がコンテナ確保から実行までをやる。

```
./scripts/build.sh main          # ros2_ws_main (常用。初回は時間がかかる)
./scripts/build.sh slamtoolbox   # ros2_ws_slamtoolbox (軽量)
./scripts/build.sh liosam        # ros2_ws_liosam (凍結中)
```

- 並列度は `BUILD_JOBS` で変えられる (既定 2)。`BUILD_JOBS=4 ./scripts/build.sh main` のように上げられるが、**落ちたら下げる**。
- `main` は `--executor sequential` を使う。canopen の並列ビルドが壊れる既知問題への対処で、外してはいけない。
- GLIM は公式イメージにビルド済みのものが入っているため、`colcon build` は不要。

## 4. CAN を上げる

通常は udev と `canusb-up.service` で、挿すだけで `can0` が自動で上がる。状態確認と復旧:

```
./scripts/can_up.sh
```

## 5. 動作確認

```
./scripts/bringup2d.sh     # 2D bringup (バックグラウンド)
./scripts/teleop.sh        # キーボード teleop
./scripts/stop.sh          # 全コンテナの ROS プロセスに SIGINT
```

bringup は起動 15 秒後に EPOS4 の状態とセンサのトピック周波数を ✔/▲/✘ の表で出す。単体で再確認するには:

```
docker exec -it rerobot_env bash -c \
  'source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash && \
   ros2 run rerobot_bringup bringup_check.py'
```

---

← [第1章 全体像](01_overview.md) | → [第3章 基本パラメータ](03_parameters.md)
