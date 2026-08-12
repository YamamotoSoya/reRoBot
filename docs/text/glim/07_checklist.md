<!-- claude: GLIM 読本 第7章 (2026-08-12) -->

# 第7章 実務チェックリスト — 起動・評価・切替・診断

繰り返し参照する章。コマンドはすべて素の形 (docker exec + ros2 ...) で示す。
glim コンテナ内の ROS 環境は `/ros_entrypoint.sh` 経由で source する。

## 7.1 オンライン起動チェックリスト

```bash
# 0. main 側: 3D bringup (R-Fans + BNO086)。LIO 構成なので IMU 必須
docker exec -d rerobot_env bash -c \
  'source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash && \
   ros2 launch rerobot_bringup rerobot_bringup_3d_imu.launch.py'

# 1. 入力 2 本の確認 (どちらか欠けると GLIM は静かに待ち続ける)
docker exec rerobot_env bash -c 'source /opt/ros/jazzy/setup.bash && \
   ros2 topic hz /sdk_could --window 20 & ros2 topic hz /imu/data --window 100; wait'
   # → 点群 ~20 Hz / IMU ~100 Hz

# 2. glim コンテナ確保 + 本体起動 (フォアグラウンド)
docker compose --profile glim up -d glim
docker exec -it glim_env /ros_entrypoint.sh \
  ros2 run glim_ros glim_rosnode --ros-args -p config_path:=/glim_config

# 3. 稼働確認: TF odom→glim_base が動き出すこと
docker exec rerobot_env bash -c 'source /opt/ros/jazzy/setup.bash && \
   ros2 run tf2_ros tf2_echo odom glim_base'
```

⚠️ LIO 構成では `/imu/data` が来ないと odometry が初期化されず**エラーも出さずに
進まない**。「viewer は開くのに点群が積もらない」ときは、まず手順 1 に戻る。

## 7.2 glim_rosbag によるオフライン評価

実走 1 回 = bag 1 本。以降のパラメータ実験はすべて bag で回すのが基本動作
(bag は ROS クロックを介さず直接読まれるため use_sim_time も不要 — 2D SLAM との
大きな違い)。

```bash
# bag の記録 (main 側。点群 + IMU + TF 一式)
docker exec -it rerobot_env bash -c \
  'source /opt/ros/jazzy/setup.bash && \
   ros2 bag record -o /workspace/bags/<name> /sdk_could /imu/data /tf /tf_static /odom'

# GLIM で処理 (glim コンテナ。auto_quit を忘れない — 事例C-2)
docker exec -it glim_env /ros_entrypoint.sh \
  ros2 run glim_ros glim_rosbag /bags/<name> --ros-args \
    -p config_path:=/glim_config -p auto_quit:=true -p dump_path:=/bags/<name>_dump
```

⚠️ `bags/` はコンテナ (root) 所有でホストから直接書けない。ホスト側で加工したければ
`docker cp` か `docker exec -i ... python3 -` 経由で
(`docs/issue/2026-08-12_glim_horizontal_drift.md` の再現メモ)。

## 7.3 dump 生成物の読み方

```
<dump>/
├── 000000/ ... ................ submap (点群 + 推定姿勢)。個数 = 第②層の出力単位
├── graph.txt .................. 大域グラフの要約 — まずここを見る
│   └── num_submaps / num_all_frames / num_matching_cost_factors
├── odom_lidar.txt / odom_imu.txt  ①層の生軌跡 (大域最適化前) — TUM 形式
├── traj_lidar.txt / traj_imu.txt  ③層の最終軌跡 — TUM 形式 (t x y z qx qy qz qw)
└── config/ .................... 実効 config (コメント除去・正規化済み) + meta
    └── meta の imu_frame_id / lidar_frame_id = frame 自動検出の結果確認
```

即使える定量チェック 2 つ:

```bash
# (a) 軌跡の総延長 — 車輪オドメトリと比べる (事例A 段4 の手法。1% 差なら健全)
python3 - <<'EOF'
import numpy as np
t = np.loadtxt('traj_lidar.txt')          # docker cp で取り出してから
print('path length [m]:', np.linalg.norm(np.diff(t[:, 1:4], axis=0), axis=1).sum())
EOF

# (b) 最終地図の PLY 書き出し (zenity ダイアログ回避つき — 事例C-1)
docker exec -it glim_env /ros_entrypoint.sh \
  ros2 run glim_ros offline_viewer /bags/<name>_dump --export_path /bags/<name>.ply
```

`odom_*.txt` と `traj_*.txt` の差が「大域最適化が直した量」— 差が巨大なら①層 (odometry)
が既に苦しんでいる。

## 7.4 CT-ICP 切替レシピ (IMU レス比較・実証済み差分 4 点)

08-12 の比較実験で実際に使った差分。`/glim_config` を直接書き換えず、
**コピーを作って差分を当てる** (元構成を汚さない):

```bash
docker exec glim_env bash -c 'cp -r /glim_config /tmp/glim_ct'
# /tmp/glim_ct に以下の 4 差分を当てる:
```

| ファイル | 変更 |
|---|---|
| `config.json` | `"config_odometry": "config_odometry_ct.json"` |
| `config_sub_mapping_cpu.json` | `"enable_imu": false` |
| `config_global_mapping_cpu.json` | `"enable_imu": false` |
| `config_ros.json` | `"extension_modules": []` (ヘッドレス化 — viewer ハング回避) |

```bash
docker exec -it glim_env /ros_entrypoint.sh \
  ros2 run glim_ros glim_rosbag /bags/<name> --ros-args \
    -p config_path:=/tmp/glim_ct -p auto_quit:=true -p dump_path:=/bags/<name>_ct_dump
```

⚠️ `config.json` 内の古い「戻し手順」コメント (imu_frame_id: glim_base 云々) は
使わない — 上の 4 点が正 (第 5 章 5.6)。

## 7.5 症状からの逆引き表

| 症状 | 最初に見るもの | 原因の典型 | 対処 (章) |
|---|---|---|---|
| 点群が積もらない・無反応 | `ros2 topic hz /sdk_could /imu/data` | LIO 構成で IMU 欠け / bringup が imu なし | 7.1。glim3d 系は IMU 必須 |
| 起動直後に .so ロード失敗 | エラーの so 名 | GPU 系 config / libimu_validator を結線した | 第2章 2.2 (このイメージに無い) |
| topic list に見えるのに届かない | 両コンテナの ROS_DOMAIN_ID / compose の ipc | `ipc: host` 欠落 (FastDDS 共有メモリ) | docker-compose.yml 冒頭の設計メモ |
| 旋回するほど壁が二重・滲む | 時刻 (per-point time / stamp) と T_lidar_imu | deskew の材料が狂っている | 第3章 3.4 / 第5章 5.3 / タイムスタンプ読本 §7.2 |
| 床が傾く・z がドリフト | odometry 方式 | CT-ICP (重力基準なし) / IMU が実は使われていない | 第1章 1.2。enable_imu 3 点セット確認 (7.6) |
| 水平に流れる (床は健全) | 環境の特徴量・経路 | 縮退 × 車輪 prior 不在 | 事例A。撮り直し / 融合 / 第4章 4.11 |
| TF ツリーが壊れた (親二重) | `ros2 run tf2_tools view_frames` | publish_imu2lidar: true に戻した等 | 第5章 5.4 |
| offline_viewer が 0% で止まる | — | zenity ダイアログ待ち | `--export_path` (事例C-1) |
| dump が読めない / values.bin 欠け | 終了のさせ方 | Ctrl+C 連打で dump 中断 | `-p auto_quit:=true` (事例C-2) |
| 走行中に古い点群が消えて見える | — | viewer の表示間引き (仕様) | 事例C-3。証拠は dump で |
| 処理が実時間に追いつかない | CPU 使用率 | CPU 版で点数過多 | 第4章 4.4 downsample↑ / num_threads↑ / smoother_lag↓ |

## 7.6 変更時の連動点検

| 変えたもの | 連動して見直すもの |
|---|---|
| URDF の imu_joint / rfans_joint | **`config_sensors.json` の `T_lidar_imu` を再計算** (第5章 5.3)。EKF 側の設定も (CLAUDE.md 明記) |
| odometry 方式 (LIO ⇔ CT-ICP) | enable_imu ×2 + config_odometry の **3 点セット** (第5章 5.5) |
| rfans_driver の frame_id / stamp 仕様 | lidar_frame_id 自動検出の前提・per-point time 設定 (第4章 4.3)。タイムスタンプ読本 §7.2 で再検証 |
| IMU の機種・レート | imu_*_noise 4 兄弟 (第3章 3.3)・ang_scale の要否 (第4章 4.2)・imu_qos depth |
| config JSON 全般 | 再ビルド不要 (mount 直読み)。glim_rosnode の再起動のみ。実効値は dump の config/ で確認 |
| bringup の起動構成 | LIO は `/imu/data` 必須 — imu なし bringup と組むと無言で止まる (7.1) |

---

これで本書は終わり。2D SLAM 側の一般論・事例は
[slam_toolbox 読本](../slam_toolbox/00_index.md)、時刻の設計は
[タイムスタンプ読本](../timestamp/00_index.md)、車輪オドメトリ+IMU の融合 (EKF) は
[EKF 読本](../ekf_fusion/00_index.md) へ。
