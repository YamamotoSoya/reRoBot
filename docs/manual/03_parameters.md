<!-- claude: 運用手引き 第3章 テンプレ (2026-09-22)。ユーザの箇条書き草稿を Claude が表形式に整理 (2026-09-22、値は同日時点)。 -->

# 第3章 基本パラメータ

日常的に触る設定ファイルは `ros2_ws_main/src/bringup/rerobot_bringup/config/` に集まっている。
ここでは「どのファイルが何を決めているか」と、各ファイルでよく変える代表パラメータだけをまとめる。
全パラメータの意味は各ファイル内のコメントを正とする。

| ファイル | 読むノード | 何を決めるか |
|---|---|---|
| [params.yaml](../../ros2_ws_main/src/bringup/rerobot_bringup/config/params.yaml) | `epos4_controller` / `epos4_odometry` / `rfans_driver` / `rfans_calculation` | 車体寸法・減速比・加減速制限・安全監視・3D LiDAR 接続 |
| [joy_teleop.yaml](../../ros2_ws_main/src/bringup/rerobot_bringup/config/joy_teleop.yaml) | `joy_node` / `teleop_twist_joy_node` | ゲームパッドの軸・ボタン割り当てと速度スケール |
| [ekf.yaml](../../ros2_ws_main/src/bringup/rerobot_bringup/config/ekf.yaml) | `ekf_filter_node` (robot_localization) | 車輪 odom と IMU の融合方針 |

---

## 3.1 車体基本パラメータ — `params.yaml`

車体の物理量とモータ制御の安全網。**車体を改修したら最初に見るファイル**。
`epos4_controller_node` と `epos4_odometry_node` のセクションに同じ車体値が並んでいるので、変えるときは両方を揃える。

| パラメータ | セクション | 現在値 | 意味 / 変えるとどうなるか |
|---|---|---|---|
| `tread_width` | controller, odometry | 0.529 m | 左右車輪の間隔。旋回時の左右速度差と、odometry の yaw 積分に効く。ずれると旋回角がずれる |
| `tire_diam` | controller, odometry | 0.256 m | タイヤ外径。指令速度→rpm 変換と走行距離の両方に比例して効く |
| `gear_ratio` | controller, odometry | 92.25 | 減速比 (モータ回転 / 車輪回転)。間違えると速度も距離も同じ倍率で狂う (旧 5.0 → 減速機交換で変更) |
| `invert_left` / `invert_right` | controller, odometry | true / true | 回転方向の反転。controller は指令、odometry は距離の符号に効く。前進指令で後退するならここ |
| `max_motor_accel_rpm_per_s` / `max_motor_decel_rpm_per_s` | controller | 15000 | 加減速ランプの上限。急停止時の回生スパイクで全モータ停止した対策。下げると滑らかになるが止まるのが遅くなる |
| `cmd_timeout_s` | controller | 0.5 s | `/robot_speed_cmd` がこの秒数途絶したら速度 0 へ。teleop / joy 切断時の暴走防止。0.0 で無効 |
| `link_loss_timeout_s` | controller | 1.0 s | 両モータの PDO がこの秒数途絶したら CAN リンク喪失と判定して目標 0 |
| `rps` | rfans_driver, rfans_calculation | 10 | 3D LiDAR の回転数 [Hz]。5 / 10 / 20。**2 セクションで同値必須** |
| `device_ip` | rfans_driver, rfans_calculation | 192.168.0.3 | LiDAR 実機の IP。ホスト側は 192.168.0.100 (README) |
| `frame_id` | rfans_calculation | `rfans` | 点群の座標フレーム名。URDF の rfans link 名と一致させる |
| `vangle_override` | rfans_calculation | 16 要素 | 実測した各リングの縦角。コメントアウトすると公称表に戻る |

⚠️ 車体値 (`tire_diam` / `gear_ratio` / `invert_*`) は
[epos4_teleop/config/params.yaml](../../ros2_ws_main/src/app/epos4_teleop/config/params.yaml) にも**手動コピー**で複製されている
(キーボード teleop は自分のファイルしか読まない)。teleop 側は走行距離の**表示**にしか使われないが、
`/params-sync` スキルで不一致を検査できる。

## 3.2 ゲームパッド — `joy_teleop.yaml`

Xbox パッドで動かすときの割り当てと速度。**速度感を変えたいときはここ** (Nav2 の速度上限とは別)。

| パラメータ | ノード | 現在値 | 意味 / 変えるとどうなるか |
|---|---|---|---|
| `scale_linear.x` / `scale_linear_turbo.x` | teleop_twist_joy | 0.5 / 1.0 m/s | スティック最大時の前進速度。通常 / turbo (RB 押下) |
| `scale_angular.yaw` / `scale_angular_turbo.yaw` | teleop_twist_joy | 0.3 / 1.0 rad/s | 同じく旋回速度 |
| `enable_button` | teleop_twist_joy | 4 (LB) | deadman。押している間だけ Twist を出す |
| `enable_turbo_button` | teleop_twist_joy | 5 (RB) | 押すと turbo スケールに切替 |
| `axis_linear.x` / `axis_angular.yaw` | teleop_twist_joy | 1 / 3 | 左スティック上下 / 右スティック左右。パッドを替えて向きが変なら番号を確認 |
| `deadzone` | joy_node | 0.05 | スティック中央の遊び。放しても微速で動くなら増やす |
| `autorepeat_rate` | joy_node | 20 Hz | 入力が変わらなくても再送する周期。controller の `cmd_timeout_s` (0.5 s) より十分速い必要がある |

## 3.3 センサ融合 — `ekf.yaml`

車輪 odom (`/odom`) と IMU (`/imu/data`) を融合して `/odometry/filtered` と TF odom→base_link を出す。
`ekf:=true` で起動したときだけ使われ、その間 `epos4_odometry` の TF 配信は自動で止まる。

| パラメータ | 現在値 | 意味 / 変えるとどうなるか |
|---|---|---|
| `frequency` | 30 Hz | 推定結果の出力周期 |
| `two_d_mode` | true | z / roll / pitch を 0 に拘束。平面走行なら true 固定 |
| `publish_tf` | true | odom→base_link を EKF が出す。false にするなら `epos4_odometry` 側を true に戻す |
| `odom0_config` | vx, vy, vyaw のみ true | 車輪 odom から**速度だけ**取る。pose を取ると IMU と二重に積分誤差を持ち込む |
| `imu0_config` | yaw, vyaw のみ true | IMU から姿勢角と角速度だけ取る。加速度は振動ノイズのため使わない |
| `imu0_relative` | true | yaw を起動時からの相対角で扱う。磁北基準の絶対 yaw に依存しないため |
| `sensor_timeout` | 0.2 s | この秒数センサが来なければ予測のみで進む。IMU レートを下げるなら要確認 |

## 3.4 その他の設定ファイル

上の 3 ファイル以外で、用途ごとに触るもの。詳細は各章で扱う。

| ファイル | 内容 | 章 |
|---|---|---|
| [nav2_params.yaml](../../ros2_ws_main/src/bringup/rerobot_bringup/config/nav2_params.yaml) | Nav2 の速度上限・コストマップ・controller / planner・amcl | [第10章](10_nav2.md) / [第11章](11_amcl.md) |
| [rerobot.urdf](../../ros2_ws_main/src/bringup/rerobot_bringup/urdf/rerobot.urdf) | センサ取付位置 (laser / rfans / imu_link)。rfans は `rerobot.{flat,tilted15,tilted45}.urdf` のプリセットからコピー | [第1章](01_overview.md) |
| [slam_toolbox.yaml](../../ros2_ws_slamtoolbox/src/rerobot_slamtoolbox/config/slam_toolbox.yaml) | 2D SLAM の解像度・スキャンマッチ設定 | [第6章](06_slam_toolbox.md) |
| [ros2_ws_glim/config/](../../ros2_ws_glim/config/) | GLIM の JSON 一式。`config_sensors.json` は URDF のプリセットと**セットで切替** | [第7章](07_glim.md) |

## 3.5 変更の反映

- `rerobot_bringup` の yaml / urdf は `--symlink-install` でインストールされるので、**再ビルド不要**。該当ノードを再起動 (`scripts/stop.sh` → 再 launch) すれば反映される。
- GLIM の JSON はコンテナに直接 mount されているので、これも編集して再起動するだけ。
- ⚠️ params ファイルのキーは `ros__parameters` (アンダースコア 2 つ)。1 つだと該当ノードが起動直後に落ち、症状からは分かりにくい。

---

← [第2章 初回セットアップ](02_setup.md) | → [第4章 起動と手動操作](04_startup_teleop.md)
