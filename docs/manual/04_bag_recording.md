<!-- claude: 運用手引き 第4章 テンプレ (2026-09-22) -->

# 第4章 bag記録

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
| **`/robot_speed_cmd`** | `geometry_msgs/Twist` | teleop / Nav2 が出した速度指令。指令と実速度のずれを見るとき |

### 参考：すべての対象topicを記録
```
ros2 bag record -s mcap -o /workspace/bags/5goukan/2d3dimu/online/rosbag/$(date +%F%H%M) /rfans_driver/rfans_points /scan /imu/data /odom /tf /tf_static /diagnostics
```


← [第3章 起動と手動操作](03_startup_teleop.md) | → [第5章 SLAM_toolbox](05_slam_toolbox.md)
