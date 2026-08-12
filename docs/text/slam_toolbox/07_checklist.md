<!-- claude: slam_toolbox 読本 第7章 (2026-08-12) -->

# 第7章 実務チェックリスト — 起動・保存・診断・チューニング

繰り返し参照する章。コマンドはすべて素の形 (docker exec + ros2 ...) で示す。

## 7.1 起動チェックリスト

```bash
# 0. 前提: main コンテナで bringup が動いていること
docker exec rerobot_env bash -c \
  'source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash && \
   ros2 topic hz /scan --window 50'          # → ~40 Hz が出ること

# 1. TF の 2 系統を確認 (odom→base_link が動的、base_link→laser が static)
docker exec rerobot_env bash -c \
  'source /opt/ros/jazzy/setup.bash && \
   ros2 run tf2_ros tf2_echo odom base_link' # → 20 Hz 相当で更新されること

# 2. slamtoolbox コンテナ確保 + SLAM 起動 (フォアグラウンド)
docker compose --profile slamtoolbox up -d slamtoolbox
docker exec -it slamtoolbox_env bash -c \
  'source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash && \
   ros2 launch rerobot_slamtoolbox slam.launch.py'

# 3. 稼働確認 3 点セット
docker exec slamtoolbox_env bash -c 'source /opt/ros/jazzy/setup.bash && \
   ros2 lifecycle get /slam_toolbox'          # → active
docker exec slamtoolbox_env bash -c 'source /opt/ros/jazzy/setup.bash && \
   ros2 run tf2_ros tf2_echo map odom'        # → 出ること (最初は identity)
docker exec slamtoolbox_env bash -c 'source /opt/ros/jazzy/setup.bash && \
   ros2 topic hz /map --window 5'             # → ~0.5 Hz (map_update_interval=2.0)
```

## 7.2 地図の保存手順

走行して地図が完成したら、**slam_toolbox を止める前に**保存する。保存先は
`/workspace/maps` (= ホストの `maps/`、mount 済み) 配下にすること。

### (a) .pgm/.yaml — Nav2 用スナップショット

```bash
# slam_toolbox 自身の save_map サービスを使う (map_saver_cli 不要)
docker exec slamtoolbox_env bash -c 'source /opt/ros/jazzy/setup.bash && \
   ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
   "{name: {data: /workspace/maps/slam_toolbox/$(date +%Y_%m_%d__%H-%M)}}"'
```

Nav2 側 (main コンテナ) に nav2_map_server が入っているので、map_saver_cli でも保存できる:

```bash
docker exec rerobot_env bash -c \
  'source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash && \
   ros2 run nav2_map_server map_saver_cli \
     -f /workspace/maps/slam_toolbox/$(date +%Y_%m_%d__%H-%M) --ros-args -p save_map_timeout:=10.0'
```

### (b) .posegraph — 続きから作図 / localization モード用

```bash
docker exec slamtoolbox_env bash -c 'source /opt/ros/jazzy/setup.bash && \
   ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
   "{filename: /workspace/maps/slam_toolbox/$(date +%Y_%m_%d__%H-%M)}"'
```

再開するときは yaml の `map_file_name` + `map_start_pose` (または
`map_start_at_dock: true`) を設定して起動する (第4章 4.2)。

### 保存後の確認

```bash
ls -l maps/slam_toolbox/          # ホスト側に .pgm/.yaml (と .posegraph/.data) があること
```

⚠️ パスを `/workspace/maps` 以外にするとコンテナ内に保存され、ホストから見えず
コンテナ再作成で消える (第5章 5.6 の経緯)。

## 7.3 症状からの逆引き表

第 3 章 3.5 の「1 スキャンの旅」を上から当てる:

| 症状 | 最初に打つコマンド | 原因の典型 | 対処 (章) |
|---|---|---|---|
| /map が出ない・無反応・無エラー | `ros2 lifecycle get /slam_toolbox` | unconfigured/inactive のまま (autostart 不発) | 事例D。launch 経由で起動したか確認 |
| `queue is full` で drop 警告 | `ros2 param get /slam_toolbox scan_queue_size` | queue 1 / TF レート不足 / TF が来ていない | 事例A・第4章 4.3。rate-limit に注意 (実際はほぼ全滅) |
| /scan 自体が無い | `ros2 topic hz /scan` (main 側) | urg_node 死亡 (USB 断)。respawn なしなので静かに止まる | `docs/issue/2026-08-11_utm30lx_usb_instability.md`。bringup 再起動 |
| topic list に見えるのに echo できない | 別コンテナから `ros2 topic echo` | `ipc: host` 欠落 (FastDDS 共有メモリ) / ROS_DOMAIN_ID 不一致 | 第5章 5.1 |
| TF 解決エラー (base_footprint がどうの) | `ros2 param get /slam_toolbox base_frame` | base_frame が URDF に無い frame を指す | 第4章 4.2 (base_link に) |
| 3D 構成で SLAM が沈黙 | `ros2 topic list \| grep scan` | /sdk_could (PointCloud2) しか無く /scan 前提が満たされない | audit Issue 10。pointcloud_to_laserscan を挟むか GLIM を使う |
| 地図がループで閉じない (廊下がずれて重なる) | RViz で graph_visualization を見る | ドリフト > loop_search_maximum_distance / 応答閾値が厳しい | 第4章 4.8。ただし事例B の順序を先に |
| 地図が「絶妙に」歪む | 360° 旋回テストで yaw 精度実測 | オドメトリ (tread_width)・FOV ±90°・EKF 非使用 | 事例B の容疑者序列 |
| ループ閉じで地図が突然崩壊 | 直前の loop エッジを疑う | 偽ループ (閾値が緩い) | 第4章 4.8 を厳しく / `ceres_loss_function: HuberLoss` |
| bag 再生で SLAM が無反応 | ノードの use_sim_time を確認 | slam.launch.py は use_sim_time を注入しない | 第5章 5.2。`ros2 param set /slam_toolbox use_sim_time true` では遅い — 起動時指定が必要 |

## 7.4 bag 再生で SLAM を回す (オフライン再現)

実走なしでチューニング・切り分けをする手順 (事例C で実際に使った形の一般化):

```bash
# main 側: bag 再生 (--clock で /clock を出す)
docker exec -it rerobot_env bash -c \
  'source /opt/ros/jazzy/setup.bash && ros2 bag play /workspace/bags/<dir> --clock'

# slamtoolbox 側: use_sim_time を付けて起動する必要がある。
# slam.launch.py は注入しないので、単体ノードで上げるのが確実:
docker exec -it slamtoolbox_env bash -c \
  'source /opt/ros/jazzy/setup.bash && \
   ros2 run slam_toolbox async_slam_toolbox_node --ros-args \
     --params-file /workspace/install/rerobot_slamtoolbox/share/rerobot_slamtoolbox/config/slam_toolbox.yaml \
     -p use_sim_time:=true'
# lifecycle を手動発火 (launch を使わないため):
docker exec slamtoolbox_env bash -c 'source /opt/ros/jazzy/setup.bash && \
   ros2 lifecycle set /slam_toolbox configure && ros2 lifecycle set /slam_toolbox activate'
```

bag に TF が入っていない場合は odom→base_link / base_link→laser の供給を別途用意する
(static_transform_publisher か、bag に /tf を含めて録る)。

## 7.5 チューニングの順序 (事例B の教訓の手順化)

```
チューニングは必ずこの順で (前段を確定させてから次へ)
├── 0. 入力の健全性 ......... /scan 40 Hz・drop 警告ゼロ・TF 2 系統 (7.1 の 3 点セット)
├── 1. オドメトリ精度 ....... 360° その場旋回で yaw 誤差実測 (tread_width 較正)。
│                             直進 5 m で距離誤差実測 (gear_ratio/tire_diam)
├── 2. センサ入力の拘束量 .... FOV ±90° 制限の解除を検討 (第5章 5.5)
├── 3. prior の強化 ......... EKF 併用で SLAM (IMU=true EKF=true で bringup — 第5章 5.4)
├── 4. 採用ゲート ........... 速度域と minimum_travel/time の整合 (第3章 3.2)
└── 5. 最後にマッチング/ループ系 (第4章 4.6-4.8)
    ├── 縮退で流れる → distance_variance_penalty を下げ prior を強く
    ├── 大周回で閉じない → loop_search_maximum_distance を拡大
    └── 偽ループ → 応答閾値を上げる / HuberLoss
```

変更のたびに**同じ bag で再現比較**する (7.4)。実走のたびに条件が変わると
パラメータの効果が判定できない。

## 7.6 変更時の連動点検

| 変えたもの | 連動して見直すもの |
|---|---|
| yaml のパラメータ | 再ビルド不要 (symlink install)。slam_toolbox の再起動のみ |
| URDF のセンサ位置 | TF base_link→laser が変わる → 地図の取り直し推奨 |
| オドメトリの周期 (PDO sync) | scan_queue_size / transform_timeout の再計算 (第4章 4.3) |
| resolution | Nav2 costmap の resolution と整合確認 |
| 車体パラメータ (tread_width 等) | `/params-sync` スキルで 2 ファイル整合検査 + 地図取り直し |

---

これで本書は終わり。3D SLAM (GLIM) へ進むなら [GLIM 読本](../glim/00_index.md) が
この本の第 1 章・事例C を前提に書かれている。
