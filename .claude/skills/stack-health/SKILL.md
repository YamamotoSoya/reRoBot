---
name: stack-health
description: 起動中の reRoBot スタックを読み取り専用で診断する。CAN 状態・ノード・トピック周波数・TF・CiA402 状態を一括チェックし表で報告する。
---

<!-- claude: stack-health スキル定義。Claude 作成。-->

# Stack Health

起動中のスタックに対して**読み取り専用**の健全性チェックを行い、1 枚の表で報告する。
「なんか動きがおかしい」の一次切り分け用。

## いつ使うか

- bringup 後に「動かない」「片輪だけ動く」「odom が出ない」等の症状が出たとき
- 実機作業の開始時・走行試験の前に現状確認したいとき
- ユーザが「ヘルスチェックして」「状態を見て」等と要求したとき

## 前提

- コンテナ `rerobot_env` 内で実行する。コマンドはすべて次の形式:
  ```sh
  docker exec rerobot_env bash -c "source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash && <コマンド>"
  ```
  (非対話 bash は `.bashrc` を読まないため明示 source。`ROS_DOMAIN_ID=150` も
  `.bashrc` 側なので、必要なら `export ROS_DOMAIN_ID=150` を先頭に足す)
- 2D / 3D どちらの構成かをユーザに確認する。不明ならノード一覧から推定
  (`urg_node` があれば 2D、`rfans_driver` 系なら 3D)。

## 手順

各チェックの実測値を控えながら順に実行し、最後に表にまとめる。ブロックする
コマンドには必ず `timeout` を付ける。

1. **コンテナ稼働**: `docker ps --filter name=rerobot_env --format '{{.Status}}'`

2. **CAN インタフェース**:
   ```sh
   ip -details -statistics link show can0
   ```
   見る点: state が `UP`/`ERROR-ACTIVE` か (`BUS-OFF`/`ERROR-PASSIVE` は異常)、
   bitrate 1000000、RX/TX の bytes・errors・dropped カウンタ。
   数秒おいて 2 回実行し、RX bytes が増えていれば PDO が流れている。

3. **ノード一覧**: `timeout 10 ros2 node list` を期待集合と突合。目安 (実物と食い違えば
   実物を正として報告):
   - 共通: `/motor1/cia402_device_1`, `/motor2/cia402_device_2`, device_container,
     epos4_controller 系, epos4_odometry 系, `/robot_state_publisher`
   - 2D: urg_node / 3D: rfans_driver ノード

4. **主要トピックの周波数** (`timeout 15 ros2 topic hz --window 20 <topic>`):

   | トピック | 期待の目安 |
   |----------|-----------|
   | `/motor1/cia402_device_1/joint_states` | 約 20 Hz (PDO sync 50 ms) |
   | `/motor2/cia402_device_2/joint_states` | 約 20 Hz |
   | `/odom` | joint_states と同程度 |
   | `/scan` (2D) | 約 40 Hz (UTM-30LX) |
   | `/sdk_could` (3D) | 約 20 Hz |

   `/robot_speed_cmd` は teleop/Nav2 が動いていなければ 0 で正常。

5. **TF**: `timeout 5 ros2 run tf2_ros tf2_echo odom base_link` が変換を返すこと。
   SLAM/Nav2 稼働中なら `map → odom` も確認する。

6. **CiA402 状態 (SDO read)**: statusword 0x6041 を両モータから読む:
   ```sh
   timeout 10 ros2 service call /motor1/cia402_device_1/sdo_read \
     canopen_interfaces/srv/CORead "{index: 0x6041, subindex: 0}"
   ```
   (motor2 は `/motor2/cia402_device_2/sdo_read`。サービス名が違う場合は
   `ros2 service list | grep sdo` で実物を確認)

   判定 (statusword & 0x006F):
   | マスク後の値 | 状態 |
   |-------------|------|
   | 0x0027 | Operation enabled (正常・駆動可) |
   | 0x0023 | Switched on (enable 前) |
   | 0x0040 | Switch on disabled |
   | 0x0008 (bit3) | **Fault** |

7. **表で報告**する: `| チェック項目 | 期待 | 実測 | 判定 |`。❌ の項目には一次切り分け
   ヒントを付ける。既知問題との対応:
   - 片輪だけ動かない/enable 失敗 → `docs/report/2026-06-04_epos4_controller_init_race_dead_wheel.md`
   - joint_states の velocity が 0 → 仕様 (bus.yml で 0x606C 未マップ)。異常ではない
   - ノードが起動直後に消える → params の `ros__parameters` 綴り (CLAUDE.md の罠)
   - odom と実移動距離のズレ → `docs/issue/2026-07-07_wheel_odometry_encoder_scaling_4x.md`

## 守るべきこと

- **読み取り専用を厳守**。CiA402 の遷移サービス (init/enable 等) 呼び出し・
  `/robot_speed_cmd` への publish・パラメータ変更はしない。SDO read は可、
  **SDO write は不可**。
- `ros2 topic echo` を joint_states に使う場合、velocity は常に 0 (既知) なので
  回転判定は position の変化で行う。
- 実測値をそのまま表に載せる。**測っていない項目は「未計測」と書く**。捏造しない。
- タイムアウトで応答が無いのも重要な観測結果。「無応答 (timeout 10s)」と記録する。
