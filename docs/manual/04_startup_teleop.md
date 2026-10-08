<!-- claude: 運用手引き 第4章 テンプレ (2026-09-22) -->

# 第4章 起動と手動操作

---
## 1. docker up
すべて
```
docker compose up 
```
対象dockerコンテナのみ (例：main, glim)
```
docker compose up main main glim
```
コンテナに入る (例：main)
```
docker exec -it rerobot_env bash
```

## 2. colcon build
```
colcon build --symlink-install
source install/setup.bash
```

## 3. rerobotlaunch
```
ros2 launch rerobot_bringup rerobot_bringup.launch.py
```
<!-- claude: 2026-10-01 追記 — WT901C 統合と起動結果サマリ -->
これ 1 本で EPOS4 x2 + 2D/3D LiDAR + IMU (BNO086 **と WT901C**) が上がる。起動直後に「起動構成」のバナー
(`[起動 ]` / `[----]` で何を起動するか) が出て、約 15 秒後に **起動結果の表** が出る:

```
 ✔ OK   EPOS4 右 (motor1) 状態             Operation enabled (0x0637)
 ✔ OK   IMU (WT901C) /imu_wit/data          199.8 Hz (期待 200)
 ✘ NG   3D LiDAR (R-Fans-16) 点群             0.0 Hz (期待 10)
          → LiDAR 電源 / Ethernet / device_ip (192.168.0.3)
 ...
```
(✔ = 期待周波数の 50% 以上 / ▲ LOW = 出ているが遅い / ✘ NG = 出ていない。EPOS4 は CiA402 状態が Operation enabled で ✔)

よく使う引数:

| 引数 | 既定 | 用途 |
|---|---|---|
| `lidar_2d` / `lidar_3d` / `imu` / `ekf` | true / true / true / **true** (2026-10-08 から) | 起動するものの選択 |
| `imu_wit` | true | WT901C を止めるなら `imu_wit:=false` (`imu:=false` のときは自動で起動しない) |
| `imu_wit_port` | `ttyUSB-wt901` | WT901C のデバイス名 (/dev 以下)。udev rule 未導入なら `ttyUSB0` など |
| `check_delay` | 15.0 | 起動結果の表を出すまでの秒数 |

走行中に状態を再確認したいときは、表の部分だけ単体で出せる (有効なセンサに合わせて `-p lidar_2d:=false` 等を付ける):
```
ros2 run rerobot_bringup bringup_check.py
```

<!-- claude: 2026-10-08 追記 — 全部入り bringup と出るトピック (robot 側の構成別 launch は廃止) -->
### 出るトピック (2026-10-08〜 全部入り)

`rerobot_bringup.launch.py` は**既定で全部を起動する** (urg + R-Fans + 3D→2D + BNO086 + WT901C + EKF)。
外すのは故障・未接続・CPU を空けたいときだけ (`lidar_2d:=false` など)。使い分けは [第15章](15_launch_guide.md)。

LaserScan は **裸の `/scan` を誰も出さない** (urg と 3D→2D の衝突を名前で避ける)。使う側が読むスキャンを選ぶ。

| topic | 出すもの | 止める引数 |
|---|---|---|
| `/urg_front/scan` | urg (前, 40 Hz) | `lidar_2d:=false` |
| `/rfans_driver/rfans_points` (+ `_packets`) | R-Fans 点群 | `lidar_3d:=false` |
| `/rfans/scan` | R-Fans 3D→2D 最近点 (costmap・AMCL 用) | `rfans_scan:=false` |
| `/rfans/scan_all` | R-Fans 3D→2D 全点 (nav_amcl_scanall 用) | **既定 off**。`rfans_scan_all:=true` で出す (重い) |
| `/imu/data`, `/imu/mag` | BNO086 | `imu:=false` |
| `/imu_wit/data` | WT901C | `imu_wit:=false` |
| `/odometry/filtered` + TF `odom→base_link` | EKF (Nav2 はこれを使う) | `ekf:=false` (TF は epos4_odometry に戻る) |
| `/odom` | 車輪オドメトリ | — |
| `/motor{1,2}/cia402_device_{1,2}/joint_states`, `/joint_states` | モータ軸角 / 車輪角 | — |
| `/tf_static` | センサ取付位置 (URDF) | — |
| (入力) `/robot_speed_cmd` | 速度指令 Twist (teleop / Nav2 が出す) | — |

高さ帯の引数 (R-Fans 3D→2D):

| 引数 | 既定 | 揃える相手 |
|---|---|---|
| `scan_min_height` / `scan_max_height` | 0.3 / 6.0 | `/rfans/scan` の帯。経路計画用 /map の地図の帯 |
| `scan_all_min_height` / `scan_all_max_height` | = 上と同じ | `/rfans/scan_all` の帯。自己位置推定用地図 (`loc_map_yaml`) の帯 |

将来 urg を増やすときは `/urg_left/scan`・`/urg_right/scan` を足し、`nav2_params.yaml` の costmap の `observation_sources` に並べる (合成ノード不要)。

⚠️ 過去の bag は `/scan` で記録されている (日付によって中身が urg か R-Fans 由来)。今の launch と組み合わせて再生するなら
`ros2 bag play ... --remap /scan:=/rfans/scan` (urg なら `/urg_front/scan`)。

## 4. joy_teleope
```
ros2 launch rerobot_bringup joy_teleop.launch.py
```

← [第3章 基本パラメータ](03_parameters.md) | → [第5章 bag記録](05_bag_recording.md)
