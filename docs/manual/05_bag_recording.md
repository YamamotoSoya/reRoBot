<!-- claude: 運用手引き 第5章 テンプレ (2026-09-22) -->

# 第5章 bag記録

---
### 記録対象topic

| topic | 型 | 内容 |
|---|---|---|
| **`/rfans_driver/rfans_points`** | `sensor_msgs/PointCloud2` | R-Fans-16 の 1 回転分の点群 (frame_id `rfans`、10 Hz)。各点に `x y z intensity ring time` を持ち、`time` はスキャン開始からの相対秒。GLIM / LIO-SAM の主入力 |
| **`/scan`** | `sensor_msgs/LaserScan` | UTM-30LX の 2D スキャン (frame_id `laser`、40 Hz)。slam_toolbox と amcl の入力。3D bringup では出ないので記録しても空 |
| **`/imu/data`** | `sensor_msgs/Imu` | BNO086 の角速度・加速度・姿勢 (frame_id `imu_link`、`IMU_RATE` 既定 100 Hz、GLIM 用は 200 Hz)。EKF と GLIM の入力 |
| **`/odom`** | `nav_msgs/Odometry` | 車輪エンコーダから `epos4_odometry` が積分した 2D 位置と速度 (odom → base_link)。twist は位置差分から計算した値 |
| **`/tf`** | `tf2_msgs/TFMessage` | 動く座標変換。odom → base_link (EKF あり時は EKF が出す)。SLAM 中は map → odom も流れる |
| **`/tf_static`** | `tf2_msgs/TFMessage` | 固定の座標変換 (base_link → laser / rfans / imu_link)。URDF からの取付位置なので、**これが無いと再生時にセンサ位置が復元できない** |
| **`/diagnostics`** | `diagnostic_msgs/DiagnosticArray` | `epos4_controller` の watchdog 出力。EPOS4 のフォルトコード・CAN リンク断・指令途絶を残す。走行中に止まった原因を後から追う用 |

用途によって足す topic:

| topic | 型 | 内容 |
|---|---|---|
| **`/odometry/filtered`** | `nav_msgs/Odometry` | EKF が車輪 odom と IMU を融合した結果 (`ekf:=true` 時のみ)。Nav2 が読んでいる odom はこちらなので、Nav2 の挙動を追うなら記録する |
| **`/motor1/cia402_device_1/joint_states`**<br>**`/motor2/cia402_device_2/joint_states`** | `sensor_msgs/JointState` | 各モータの生のシャフト角 (rad)。odometry 自体を疑うときの一次データ |
| **`/robot_speed_cmd`** | `geometry_msgs/Twist` | teleop / Nav2 が出した速度指令。指令と実速度のずれを見るとき。**R-Fans 瞬停 (回転ディップ) の調査中は常に含める** — 事象直前の減速が指令かモータの負けかを判別する材料 (`docs/issue/2026-09-02_rfans_scan_motor_dropout.md`) |
| **`/rfans_driver/rfans_packets`** | `surestar_rfans_ros2/msg/RfansPacket` | R-Fans-16 の **UDP 生パケット** (1206 B、約 750 packet/s、stamp + udp_count + data)。点群に変換する前の一次データなので、縦角表・時刻復元など **LiDAR 側の計算を後から直してもこれから点群を再生成できる**。`rfans_calculation` がこの topic を購読するので、bag 再生 + そのノード起動で `/rfans_driver/rfans_points` を作り直せる |
| **`/imu_wit/data`** | `sensor_msgs/Imu` | WITmotion WT901C-TTL の角速度・加速度・姿勢 (frame_id `imu_wit_link`、200 Hz)。BNO086 との比較用 (2026-09-23 追加)。stamp は受信時刻なので BNO086 より遅延・ジッタが乗る |
| **`/imu_wit/mag`** | `sensor_msgs/MagneticField` | 同 IMU の地磁気 (200 Hz)。BNO086 は磁気を出さないので、磁北基準の yaw を検証するならこれ。`/imu_wit/temperature` も出ているが解析には通常不要 |
| **`/behavior_tree_log`** | `nav2_msgs/BehaviorTreeLog` | bt_navigator の BT ノード状態遷移 (RUNNING/SUCCESS/FAILURE)。**自律移動中に止まったとき「planner 失敗か controller 失敗か、どの復帰行動に入ったか」を特定する一次データ**。状態が変わったときだけ出るので軽い |
| **`/rosout`** | `rcl_interfaces/msg/Log` | 全ノードのログ (INFO/WARN/ERROR)。`Failed to make progress` / `collision ahead` / `Aborting` 等の文言が残る。コンテナを消すと端末ログは失われるので bag に残しておく |
| **`/plan`** | `nav_msgs/Path` | planner_server の大域経路。更新が途切れた時刻 = 経路計画が通らなくなった時刻の目安 |
| **`/amcl_pose`**<br>**`/initialpose`**<br>**`/goal_pose`** | `geometry_msgs/PoseWithCovarianceStamped` 他 | amcl の自己位置推定 / RViz で与えた初期位置 / RViz で与えたゴール。自己位置の飛びや、いつどこへゴールを出したかを追う |
| **`/map`**<br>**`/keepout_filter_mask`** | `nav_msgs/OccupancyGrid` | 走行に使った地図と進入禁止帯マスク。latched (起動時 1 回) なので **Nav2 起動前から記録を始める** と確実に入る |
| **`/local_costmap/costmap`**<br>**`/local_costmap/published_footprint`** | `nav_msgs/OccupancyGrid` / `geometry_msgs/PolygonStamped` | 局所コストマップ (4 m 四方、2 Hz) と車体外形。RPP の衝突判定で止まった疑いを検証する。global costmap は地図全体で重いので通常は記録しない |

### 参考：すべての対象topicを記録
* 本番bag　必要最低限
```
ros2 bag record -s mcap -o /workspace/bags/raw/$(TZ=Asia/Tokyo date +%F_%H%M)_<場所> /rfans_driver/rfans_points /scan /imu/data /odom /tf /tf_static /diagnostics /robot_speed_cmd
```
容量目安 — **約 8.9 MB/s ≈ 32 GB/h** (既存 bag の実測平均。ほぼ全部が `/rfans_driver/rfans_points`):

| 速度 | 1 km | 2.2 km |
|---|---|---|
| 0.25 m/s (0.9 km/h) | 67 min / **36 GB** | 2.4 h / **78 GB** |
| 0.5 m/s (1.8 km/h、joy 通常) | 33 min / **18 GB** | 1.2 h / **39 GB** |
| 1 m/s (3.6 km/h、joy turbo) | 17 min / **9 GB** | 37 min / **20 GB** |

実測例: `2026-09-221158` (大学外周 約 0.9 km を joy 通常速度で走行、1951 s = 32.5 min、平均 0.46 m/s (1.7 km/h)) → 17.3 GB (= 16.1 GiB)。上表の 0.5 m/s 行 × 0.9 とほぼ一致。

* 実験用bag 前必要topic記録 (witmotion, 生R-Fansデータ)
```
ros2 bag record -s mcap -o /workspace/bags/raw/$(TZ=Asia/Tokyo date +%F_%H%M)_<場所> /rfans_driver/rfans_points /rfans_driver/rfans_packets /scan /imu/data /imu_wit/data /imu_wit/mag /odom /tf /tf_static /diagnostics /robot_speed_cmd
```
容量目安 — **約 10 MB/s ≈ 36 GB/h** (本番 + 生パケット約 1 MB/s + witmotion 約 0.1 MB/s):

| 速度 | 1 km | 2.2 km |
|---|---|---|
| 0.25 m/s (0.9 km/h) | 67 min / **40 GB** | 2.4 h / **88 GB** |
| 0.5 m/s (1.8 km/h、joy 通常) | 33 min / **20 GB** | 1.2 h / **44 GB** |
| 1 m/s (3.6 km/h、joy turbo) | 17 min / **10 GB** | 37 min / **22 GB** |

※ 生パケットのレートは UDP 1206 B × 約 750 packet/s (10 Hz、約 30k 点/回転) からの推定。初回の実験 bag で `ros2 bag info` を見て実測値に置き換える。
※ 容量は走行時間に比例する (速度が半分なら容量は倍)。記録前に `df -h /workspace/bags` で空きを確認する。

生パケットから点群を作り直すとき: `ros2 bag play <bag> --topics /rfans_driver/rfans_packets /tf_static --clock` を流しながら、main コンテナで `rfans_calculation` ノードだけを起動する (bringup 全体は不要)。

* 自律移動 (Nav2) 用bag　本番 + Nav2 の判断過程
```
ros2 bag record -s mcap -o /workspace/bags/raw/$(TZ=Asia/Tokyo date +%F_%H%M)_<場所>_auto /rfans_driver/rfans_points /scan /imu/data /odom /odometry/filtered /tf /tf_static /diagnostics /robot_speed_cmd /behavior_tree_log /rosout /plan /amcl_pose /initialpose /goal_pose /map /keepout_filter_mask /local_costmap/costmap /local_costmap/published_footprint
```
容量目安 — **本番 bag とほぼ同じ (約 8.9 MB/s ≈ 32 GB/h)**。追加した Nav2 系 topic は合計でも数十 KB/s 程度で、容量のほぼ全部は `/rfans_driver/rfans_points` のまま。R-Fans の生データも残したいときは `/rfans_driver/rfans_packets` を足す (+約 1 MB/s)。

※ 記録は **nav2.launch.py を起動する前に開始する**。`/map` と `/keepout_filter_mask` は起動時に 1 回しか出ないので、後から記録を始めると入らない。
※ 走行が止まったときは、`/behavior_tree_log` でどの BT ノードが FAILURE になったかを見て、`/rosout` で同じ時刻の WARN/ERROR 行を確認する。`/robot_speed_cmd` が 10 Hz・1.0 rad/s の出力に変わっていたら復帰行動の Spin (behavior_server)。EPOS 側の異常かどうかは `/diagnostics` の statusword (正常 = `0x1237`) で切り分ける。

← [第4章 起動と手動操作](04_startup_teleop.md) | → [第6章 SLAM_toolbox](06_slam_toolbox.md)
