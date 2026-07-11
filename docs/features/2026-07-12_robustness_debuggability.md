<!-- claude: feature-doc スキルの設計文書。Claude 作成。-->

# 堅牢性・デバッグ性改善一式 (feat/claude-optimize)

- 日付: 2026-07-12
- 対象パッケージ: `epos4_controller` (src/epos4_controller)、`rerobot_bringup` (src/rerobot_bringup)
- 対象 ROS: ROS 2 Jazzy (検証は使い捨てコンテナ `rerobot-rerobot:latest`。実機 CAN 未検証)
- 関連文書: 該当なし (本ブランチの土台リファクタは git log 42f11af〜ab3e402 を参照)

## 1. 目的・概要

異常時に**黙って**おかしくなるパターンを潰し、走行会で問題が起きたときに
「なぜ止まったか / なぜ位置が飛んだか」をログから即座に切り分けられる状態にする。
対象としたのは: teleop 切断後も最後の速度で走り続ける、EPOS4 の FAULT に誰も気づかない、
エンコーダ値の飛びで odom がテレポートする、LiDAR プロセス死で SLAM が静かに固まる、
パラメータ設定ミス (gear_ratio=0 等) のまま走り出す、の 5 系統。

実装した機能:
- **デッドマン監視** (`epos4_controller`): `/robot_speed_cmd` が `cmd_timeout_sec` (既定 0.5 s) 途絶えたら両輪 0 rpm + WARN 1 回
- **statusword 常時監視** (`epos4_controller`): `motor_status_period_sec` (既定 2.0 s) ごとに SDO で 0x6041 を読み、FAULT / 励磁喪失 / 復帰を**状態変化時のみ** ERROR/WARN/INFO
- **車体パラメータ検証** (`epos4_controller` / `epos4_odometry`): tread_width / tire_diam / gear_ratio が 0 以下なら `RCLCPP_FATAL` + throw で起動拒否。起動時に実効値を INFO ログ
- **オドメトリジャンプガード** (`epos4_odometry`): 1 更新あたりの車輪移動が `pose_jump_threshold` (既定 0.5 m) を超えたら積分せず基準取り直し + WARN
- **入力無音監視** (`epos4_odometry`): 5 s ごとに「両モータ無音 / motor1 のみ無音 / motor2 のみ無音 / 両方流れているが同期ペア 0 (タイムスタンプずれ)」を切り分けて WARN、回復時 INFO
- **LiDAR respawn** (`rerobot_bringup` launch): urg_node / rfans_driver に `respawn=True, respawn_delay=2.0`

スコープ外 (意図的に未対応):
- **CAN バス自体の死活監視** (bus-off 検知等): ros2_canopen (submodule) 側の責務でアプリ層から統一的に見る手段がない。statusword SDO read の失敗 WARN が間接的な代替
- **FAULT からの自動リカバリ** (`recover` 自動発行): 走行中の勝手な再励磁は安全側でない。検知・ログに留め、復帰は人間の判断 (`/robot_free_mode` トグル等) に委ねる
- **デッドマン発動の diagnostics トピック化**: 現状ログのみ。Nav2 統合が本格化してから
- **epos4_controller / epos4_odometry の respawn**: どちらも起動時状態 (init シーケンス、pose 積分基準) を持つため、無条件 respawn はかえって危険と判断

## 2. 設計の勘所

### 2.1 デッドマンは「送信側の 100 Hz timer 冒頭」で判定する

判断: 判定を `timer_callback()` (100 Hz の TPDO 送信ループ) の冒頭に置いた。
コールバック駆動 (subscription 側) で止めると「メッセージが来ない」こと自体を検知できない。
送信ループは常に回っているので、そこで `now() - last_cmd_time_ > cmd_timeout_sec_` を
見るのが唯一確実な場所。WARN は発動時 1 回のみ (`have_cmd_` フラグを落とす) で、
タイムアウト中に毎周期 WARN を吐いてログを埋めない。

却下した代替案:
- 独立ウォッチドッグタイマー: 100 Hz ループと二重管理になり、判定と 0 rpm 書込の間に
  レースが生じる。同じスレッド (executor) 内で判定→上書きが最も単純
- EPOS4 側のハードウェアタイムアウト設定のみに頼る: PDO sync が生きている限り
  最後の目標速度が保持されるため、アプリ層の途絶検知の代替にならない

### 2.2 statusword 監視は「状態変化時のみログ」+ ゲート 3 枚

判断: 2 s ごとの SDO read 結果を `classify_status()` で "FAULT" / "OP_ENABLED" /
"NOT_ENABLED" の 3 状態に分類し、前回 (`MotorInterface::last_status_state`) と
異なるときだけログする。定常運転中は完全に無音で、変化の瞬間だけ残るので
「いつ FAULT に落ちたか」がログの 1 行で分かる。

判定式は実物と同一: FAULT = `sw & 0x0008`、Operation Enabled = `(sw & 0x6F) == 0x27`
(init シーケンスの SDO 検証と同じマスク)。

監視を止めるゲートが 3 つある:
- `init_done_` (atomic): init シーケンス完了前は状態遷移中で誤警報になるため監視しない
- `free_mode_`: 意図的な脱力中の NOT_ENABLED は正常なので黙る
- `reenable_active_` (atomic): free mode 復帰スレッドの enable→CSV 遷移中も黙る

却下した代替案:
- 毎回無条件ログ: 2 s ごとに 2 行増え続け、肝心の変化が埋もれる
- TPDO/RPDO に statusword をマッピング: bus.yml (submodule) の変更が必要で影響範囲が
  広い。2 s 周期の SDO read なら CAN 負荷はほぼゼロで十分

### 2.3 パラメータ検証は throw で「起動させない」

判断: gear_ratio 等が 0 以下のときの正しい動作は存在しない (IK でゼロ除算、
odom で距離が常に 0 or 発散)。WARN して既定値で続行すると「動くが値がおかしい」
という最悪のデバッグ対象になるため、`RCLCPP_FATAL` + `throw std::invalid_argument`
でプロセスごと落とす。params yaml の指定ミス (ファイル間違い・typo で宣言が
効かず既定値 1.0 が入る等) を起動の瞬間に露呈させる狙い。
正常起動時も実効値を 1 行 INFO するので「どの yaml が効いたか」を後から確認できる。

実測 (使い捨てコンテナ): `-p gear_ratio:=0.0` 付き起動で
`[FATAL] invalid chassis params: tread_width=0.410 tire_diam=0.150 gear_ratio=0.000`
→ `std::invalid_argument` で Aborted を確認。

### 2.4 オドメトリのジャンプガードは「棄却 + 基準取り直し」

判断: canopen ドライバ再起動などで joint_states.position が不連続に飛ぶと、
差分積分の odom は一瞬でテレポートする。閾値超過時はその 1 サンプルを積分せず、
`last position` を新しい値で取り直して return する (= 飛びを「なかったこと」に
するのではなく、飛んだ後の位置から続きを積む)。閾値 0.5 m は PDO sync 50 ms あたり
10 m/s 相当で、実車の最高速に対して 1 桁以上の余裕がある。

却下した代替案:
- クランプ (閾値でサチらせて積分継続): 飛びの原因がドライバ再起動の場合、
  クランプ値ぶん確実に汚染される。捨てる方が誤差が小さい
- メディアンフィルタ等の平滑化: 遅延が入る上、正常時の応答も鈍る

### 2.5 無音監視は「原因を名指しする」

判断: message_filters の ApproximateTime 同期は片方の入力が欠けると**無言で**
コールバックが止まる。odom が止まった事象は同じでも原因は 4 通りあるため、
`Subscriber::registerCallback` でトピック別カウンタ (`m1_msg_count_` /
`m2_msg_count_`) を、同期コールバックでペアカウンタ (`pair_count_`) を数え、
5 s ごとの増分で「両方無音 / motor1 のみ / motor2 のみ / 両方来ているがペア 0
(= タイムスタンプずれ)」を切り分けて WARN する。回復も INFO で残す。

却下した代替案:
- ペアカウンタのみの監視: 「odom が止まった」ことしか分からず、CAN・ドライバ・
  タイムスタンプのどこを疑うべきか現地で切り分けし直しになる

### 2.6 respawn は LiDAR ドライバのみ

判断: urg_node / rfans_driver はステートレス (再起動すれば元に戻る) なので
`respawn=True, respawn_delay=2.0` が安全に効く。epos4_controller / epos4_odometry は
起動時状態を持つため respawn 対象から外した (§1 スコープ外参照)。

## 3. データフロー

```
/robot_speed_cmd (geometry_msgs/Twist)
      │  最終受信時刻を last_cmd_time_ に記録
      ▼
[epos4_controller_node] ── 100 Hz timer_callback
      │  冒頭: now()-last_cmd_time_ > cmd_timeout_sec → 両輪 0 rpm + WARN(1回)
      │
      ├─ 2 s ごと statusMonitorCallback (init_done_ && !free_mode_ && !reenable_active_)
      │     └→ /motor{1,2}/cia402_device_{1,2}/sdo_read (canopen_interfaces/srv/CORead)
      │          index=0x6041 → classify_status() → 変化時のみ ERROR/WARN/INFO
      │
      └→ /motor{1,2}/cia402_device_{1,2}/tpdo (canopen_interfaces/msg/COData, 0x60FF)

/motor1/cia402_device_1/joint_states ─┬─ registerCallback → ++m1_msg_count_
/motor2/cia402_device_2/joint_states ─┼─ registerCallback → ++m2_msg_count_
      (sensor_msgs/JointState)        │
                                      ▼ ApproximateTime 同期
[epos4_odometry_node] onJointStates ── ++pair_count_
      │  |d_left|,|d_right| > pose_jump_threshold → WARN + 基準取り直し + return
      ▼
   /odom (nav_msgs/Odometry) + TF odom→base_link + /joint_states 再発行
      ▲
      └─ 5 s ごと checkInputHealth(): カウンタ増分で
         「両方無音 / motor1 のみ / motor2 のみ / ペア 0 (時刻ずれ)」を WARN
```

## 4. 使い方

追加パラメータはすべて `src/rerobot_bringup/config/params_2d.yaml` / `params_3d.yaml` に
既定値入りで記載済み。通常の bringup コマンドは従来どおりで、何もしなくても有効になる:

```bash
ros2 launch rerobot_bringup rerobot_bringup_2d.launch.py   # or _3d
```

| パラメータ | ノード | 既定値 | 説明 |
| --- | --- | --- | --- |
| `cmd_timeout_sec` | epos4_controller_node | 0.5 | `/robot_speed_cmd` 途絶→0 rpm までの秒数。**0 で無効** |
| `motor_status_period_sec` | epos4_controller_node | 2.0 | statusword(0x6041) SDO 監視周期 [s]。**0 で無効** |
| `pose_jump_threshold` | epos4_odometry_node | 0.5 | 1 更新の車輪移動上限 [m]。超過は棄却。**0 で無効** |

検証コマンド (使い捨てコンテナ。稼働中の `rerobot_env` に触れない):

```bash
docker run --rm -v /home/tumutamu/reRoBot-optimize/src:/workspace/src \
  -w /workspace -e ROS_DOMAIN_ID=77 rerobot-rerobot:latest bash -lc '
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --executor sequential \
  --packages-select maxon_epos4_ros2 epos4_controller epos4_teleop \
                    epos4_vel_ros2 rfans_driver rerobot_bringup
source install/setup.bash
timeout 5 ros2 run epos4_controller epos4_odometry --ros-args \
  --params-file src/rerobot_bringup/config/params_2d.yaml -p gear_ratio:=0.0'
# → [FATAL] invalid chassis params ... で Aborted すれば fail-fast は機能している
```

## 5. 変更ファイル一覧

- `src/epos4_controller/src/epos4_controller.cpp` — デッドマン監視 (timer_callback 冒頭)、statusword 監視 (`classify_status` / `statusMonitorCallback` / ゲート用 atomic 2 本)、車体パラメータ検証 + 起動ログ
- `src/epos4_controller/src/epos4_odometry.cpp` — ジャンプガード、トピック別カウンタ + `checkInputHealth()` 無音監視、車体パラメータ検証 + 起動ログ
- `src/rerobot_bringup/config/params_2d.yaml` — `cmd_timeout_sec` / `motor_status_period_sec` / `pose_jump_threshold` 追加
- `src/rerobot_bringup/config/params_3d.yaml` — 同上 (2D と同値)
- `src/rerobot_bringup/launch/rerobot_bringup_2d.launch.py` — urg_node に `respawn=True, respawn_delay=2.0`
- `src/rerobot_bringup/launch/rerobot_bringup_3d.launch.py` — rfans_driver に同上
- `CLAUDE.md` — 上記パラメータと挙動の記載を追記

## 6. 既知の制限

- **実機 CAN 未検証**: statusword 監視・デッドマン発動・ジャンプガードはコンテナ内
  スモークテスト (起動ログ・fail-fast・launch --print) までしか確認していない。
  実 EPOS4 での FAULT 誘発試験、teleop 切断試験は実機接続時に必須
- statusword 監視は SDO read の応答が返らない場合 (バス断) は WARN
  「could not read statusword」系に落ちるのみで、バス断と FAULT を区別できない
- デッドマン発動中も canopen ドライバ・PDO sync は生きたままなので、EPOS4 は
  「0 rpm を受信し続ける」状態になる (励磁は切らない。切りたいときは `/robot_free_mode`)
- 無音監視の周期 5 s はコード内固定 (パラメータ化していない)。必要になったら
  `pose_jump_threshold` と同様にパラメータ化する
