<!-- claude: 運用手引き 第7章 テンプレ (2026-09-22) -->

# 第7章 GLIM

---

## GLIM 各操作
### CPU mode

コンテナは `glim_env`。`docker compose --profile glim up -d glim` で起動し、`docker exec -it glim_env bash` の中で以下を実行する。

② **GLIM オンライン** → bags/glim/<日付_時分>_<場所>_live_dump/

```
  ros2 run glim_ros glim_rosnode --ros-args \
    -p config_path:=/glim_config \
    -p dump_path:=/workspace/bags/glim/$(TZ=Asia/Tokyo date +%F_%H%M)_<場所>_live_dump
```

- 終了は Ctrl-C を 1 回だけ。保存処理は終了時に走るので、連打すると 08-12 に踏んだ「values.bin 欠け」が再発します。


③ **GLIM オフライン評価** → bags/glim/<元bag名>_dump/<タグ>/  (タグ = default / k20 / filtered …)

```
B=2026-09-27_1651_5goukan # 評価対象 bag のディレクトリ名 (bags/raw/ 直下)
ros2 run glim_ros glim_rosbag /workspace/bags/raw/$B \
 --ros-args -p config_path:=/glim_config \
 -p auto_quit:=false \
 -p dump_path:=/workspace/bags/glim/${B}_dump/default
```

- auto_quit:=true で bag 読み切り後に自動保存・自動終了 (手動 Ctrl-C 不要)。パラメータ実験のときは config_path を変異 config に差し替えるだけで、dump_path を bags/exp/<日付>_<テーマ>/EN/ 系に向ければ実験もこの木に収まります。
- dump を offline_viewer で開くときは、dump 内にコピーされた config の extension_modules から librviz_viewer.so を外す (08-12 に確立した回避策) のを忘れずに。

④ **GLIM offline viewer**
```
B=2026-08-14_0919_5goukan
ros2 run glim_ros offline_viewer /workspace/bags/glim/${B}_dump/default
```

### GPU mode

CPU 版とは**別コンテナ** (`glim_gpu_env`)。イメージのビルドは [第2章](02_setup.md#gpu-版-glim-gpu-搭載機のみ) を参照。
コマンドは CPU 版と同じで、入る先のコンテナだけが違う (設定の入口は中で差し替わるので `config_path` は同じ `/glim_config`)。

```
docker compose --profile glim_gpu up -d glim_gpu   # 起動 (CPU 版は止めておく)
docker exec -it glim_gpu_env bash                  # 以降はこの中で実行
```

⑤ **GLIM オフライン評価 (GPU)** → bags/glim/<元bag名>_dump/gpu/

```
B=2026-09-27_1651_5goukan # 評価対象 bag のディレクトリ名 (bags/raw/ 直下)
ros2 run glim_ros glim_rosbag /workspace/bags/raw/$B \
 --ros-args -p config_path:=/glim_config \
 -p auto_quit:=false \
 -p dump_path:=/workspace/bags/glim/${B}_dump/gpu
```

- ⚠️ **CPU 版コンテナと同時に起動しない**。どちらも network_mode: host + 同じ ROS_DOMAIN_ID なのでトピックが衝突する。
- ⚠️ GPU 用の config 3 本 (`config_{odometry,sub_mapping,global_mapping}_gpu.json`) は**上流デフォルトのまま**で、CPU 側の調整は入っていない。パラメータ名の体系が違うため単純移植もできない (例: `create_between_factors` が CPU true / GPU false、`randomsampling_rate` が 0.2 / 1.0、`submap_downsample_resolution` が 0.3 / 0.1)。**CPU 版と比較するなら条件を揃えてから**。
- 動いているのが GPU かの確認は、別端末で `nvidia-smi` を見る (`glim_rosbag` のプロセスが GPU メモリを掴んでいる)。
- オンライン実行 (②) と offline_viewer (④) も同じコンテナで同じコマンドが使える。

## 各種パラメータ



← [第6章 SLAM_toolbox](06_slam_toolbox.md) | → [第8章 LIO_SAM](08_lio_sam.md)
