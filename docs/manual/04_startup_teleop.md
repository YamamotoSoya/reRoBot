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
| `lidar_2d` / `lidar_3d` / `imu` / `ekf` | true / true / true / false | 起動するものの選択 |
| `imu_wit` | true | WT901C を止めるなら `imu_wit:=false` (`imu:=false` のときは自動で起動しない) |
| `imu_wit_port` | `ttyUSB-wt901` | WT901C のデバイス名 (/dev 以下)。udev rule 未導入なら `ttyUSB0` など |
| `check_delay` | 15.0 | 起動結果の表を出すまでの秒数 |

走行中に状態を再確認したいときは、表の部分だけ単体で出せる (有効なセンサに合わせて `-p lidar_2d:=false` 等を付ける):
```
ros2 run rerobot_bringup bringup_check.py
```

## 4. joy_teleope
```
ros2 launch rerobot_bringup joy_teleop.launch.py
```

← [第3章 基本パラメータ](03_parameters.md) | → [第5章 bag記録](05_bag_recording.md)
