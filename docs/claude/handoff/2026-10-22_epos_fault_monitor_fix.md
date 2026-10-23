<!-- claude: 2026-10-22 作成。別エージェントへの作業引き継ぎ (EPOS Fault 見逃し対策)。完了したら issue 側に結果を書き、このファイルは消してよい -->
# 引き継ぎ: EPOS Fault (0x8250) を見逃す監視の修正 + 記録の追加

**最初に読むもの**: `CLAUDE.md` → `docs/issue/2026-10-22_epos_fault_8250_pdo_freeze_lidar3d_run.md` (本件、特に §2.3 / §3 / §4 / §6) → `docs/issue/2026-09-19_epos_stop_link_loss_rpdo_timeout_power.md` §10.4〜10.5 (監視を今の形にした経緯) → `docs/features/2026-09-19_epos4_controller_watchdog_fault_monitor.md` (監視の設計)。

## 1. 何が起きたか (3 行)

2026-10-22 18:42:49.6 JST、自律走行中に EPOS が両輪同時に Fault 0x8250 (RPDO timeout) で停止しました。PDO が両方向とも止まり、SDO は生きていました。`epos4_controller` の監視は bag (`bags/raw/2026-10-22_1825_5goukan_lidar3d`) の最後まで「statusword 0x1237・リンク正常」と報告し続けました。Nav2 も 50 s 間 0.4 m/s を出し続け、誰も停止に気づきませんでした。なぜ PDO が止まったか (原因の層) は記録不足で未特定です。

## 2. やってほしいこと (優先順)

### タスク A: 記録を足す (原因の層を次回捕まえるため)

- `ros2_ws_main/src/bringup/rerobot_bringup/launch/auto_run.launch.py:48` の `RECORD_TOPICS` に以下を追加します: `/motor1/cia402_device_1/tpdo` `/motor2/cia402_device_2/tpdo` `/motor1/cia402_device_1/rpdo` `/motor2/cia402_device_2/rpdo`
  - tpdo = controller → driver の速度指令 (100 Hz)。rpdo = driver → 購読者の受信 PDO。
  - 型は `canopen_interfaces/msg/COData`。rpdo がキャッシュ再送かどうかで、量が想定と違う可能性があります (タスク B-1 の結果を見てから判断)。
- 走行中に `candump -l can0` を裏で回す仕組み (`auto_run.launch.py` に ExecuteProcess、または bag と同じディレクトリへ出力)。
  - 見たい ID: SYNC 0x080 / RPDO 0x201・0x202 (PC→EPOS) / TPDO 0x181・0x182 (EPOS→PC) / heartbeat 0x701・0x702 / EMCY 0x081・0x082。
  - 容量は 10 ms SYNC + PDO で数千 frame/s 程度なので、走行 1 時間で数十〜百 MB 程度のはずです (要実測)。
- ⚠️ `docs/manual/05_bag_recording.md` の記録コマンドは**ユーザ執筆なので編集しない** (CLAUDE.md)。追記が必要ならユーザに提案だけします。

### タスク B: 監視の修正 (`ros2_ws_main/src/app/epos4_controller/src/epos4_controller.cpp`)

現状の仕組み:
- `on_rpdo()` (L296 付近) が `~/rpdo` の 0x6041 を受けるたびに `statusword` と `last_ns` を更新します。
- 監視スレッド `poll_motor()` (L654 付近) は `last_ns` の鮮度で `alive` を判定し、Fault ビットの立ち上がりで SDO 0x603F/0x1003/0x2200 を読んで ERROR を出します。
- `alive` が false のときは**ログを出さずに return** します (L670 付近)。
- 監視ループ本体 (L724 付近) は両ノードとも `!alive` で「CAN リンク喪失」と判定します。

問題: 今回は `~/rpdo` が届き続け (= `alive`)、中身は Fault 前の 0x1237 のままでした。キャッシュ再送では生存も Fault も分かりません。

1. **前提の確認 (先にやる)**: ros2_canopen 0.3.4 (apt、`ros-jazzy-canopen-proxy-driver` / `-base-driver` 0.3.4。ローカルにソースは無いので GitHub `ros-industrial/ros2_canopen` の該当タグを読む) で、ProxyDriver / Cia402Driver が `~/rpdo` を**受信のたびにだけ** publish するのか、**周期でキャッシュを** publish するのかを確かめます。
   - 09-19 §10.4 で `joint_states` はキャッシュ周期 publish と確認済みです。`rpdo` も同じなら、受信時刻ベースの判定は根本的に使えません。
2. **生存判定を、キャッシュ再送では変わらない量に置き換えます**。候補 (1 の結果で選ぶ):
   - (a) 低頻度 SDO で 0x6041 を直読みする (例 2 s 周期)。09-19 §10.5 で 0.5 s 周期の sdo_read が driver の INFO ログを毎回出して端末が流れ、ユーザが誤認した前例があるので、周期は長めにし、ログの扱いを考えます。
   - (b) 指令が非ゼロなのに TPDO の位置が一定時間変わらない、を検知する (タスク C と兼ねられる)。
   - (c) 1 の結果、受信時だけ publish だと分かった場合は、現行の判定は正しいことになり、今回の「届き続けた」は別の説明が要ります。issue §2.3 の解釈を直してください。
3. `alive == false` で黙って return する箇所に、少なくとも状態遷移時の WARN を足します。

### タスク C: 指令と実速度の不一致検知

- 「`/robot_speed_cmd` の |v| > 0.1 m/s (または |ω| > 閾値) が出ているのに、`/odom` の twist がほぼ 0 の状態が 1 s 以上続く」を検知し、ERROR + `/diagnostics` + 目標 0 にします。
- `/odom` の twist は `epos4_odometry` が**位置差分**で計算しています (0x606C 非マップのため driver の velocity は常に 0 — CLAUDE.md)。位置がキャッシュで凍結すれば twist は 0 になるので、今回の型を検知できます。
- 置き場所は `epos4_controller` (既に指令ウォッチドッグがある) が自然です。ただし odom を購読させるか、`joint_states` 直読みにするかは任せます。
- 誤検知に注意する場面: 発進直後 (加速中)、壁押し付け、浮かせ試験でタイヤ空転 (浮かせなら位置は動くので問題なし)。

### 対象外 (やらない)

- **Fault からの自動 recover** は入れません。原因不明のまま入れると暴走側の故障を隠す恐れがあります (issue §6-4、09-19 §6-1)。
- 原因の断定 (H1〜H4)。記録を足して次回の再発を待つ方針です。

## 3. 守ること (このリポジトリ固有)

- **ビルドはコンテナ内**: `docker exec rerobot_env bash -c "cd /workspace && source /opt/ros/jazzy/setup.bash && MAKEFLAGS=-j2 colcon build --symlink-install --executor sequential --metas ./src/colcon.meta --packages-select epos4_controller"` (並列は控えめに — 機械が落ちる)。
- 変更前に「どこを・何を・なぜ」を 1〜4 行で説明し (`.claude/skills/explain-edits/SKILL.md`)、変更箇所に `// claude` コメントタグを付けます。clang-format 準拠。
- **モータを動かす検証は「浮かせて確認 → 接地」**。監視の検証は bringup を起動して静止状態でも一部できます。Fault の再現は、例えば controller の tpdo 送出を一時的に止める (ノードを SIGSTOP 等) と 0x8250 を人工的に起こせるはずですが、**実施前にユーザに確認**してください。
- 原因に関する断定を docs に書く前に `critic` subagent を起こします (CLAUDE.md)。
- 返信・ドキュメントは日本語。

## 4. 触らないもの (別作業の未コミット変更が作業ツリーにある)

以下は 10-22 の 3D 自己位置推定の作業で、メインのセッションが扱います。**revert・commit・編集をしないでください**:

- `ros2_ws_main/src/bringup/rerobot_bringup/config/nav2_params.yaml` (RPP `transform_tolerance` 0.2→0.5)
- `ros2_ws_main/src/bringup/rerobot_bringup/config/localization/lidar_localization.yaml` (`map_odom_tf_future_dating_sec: 0.5`)
- submodule `ros2_ws_main/src/localization/lidar_localization_ros2` の 4 ファイル (`map_odom_tf_future_dating_sec` の追加、上流 v1.3.0 に直接変更)
- `docs/issue/2026-10-22_epos_fault_8250_pdo_freeze_lidar3d_run.md` は未コミット。結果の追記は OK

commit する場合は自分の変更ファイルだけを `git add <path>` で指定し、main に直接コミットします (CLAUDE.md の運用)。

## 5. 参考データの場所

- bag: `bags/raw/2026-10-22_1825_5goukan_lidar3d` (Fault = epoch 1792662169.6、bag 開始 1792661136.66)。`/diagnostics` に監視の出力、`/joint_states`・`/odom`・`/robot_speed_cmd` あり。tpdo/rpdo と CAN フレームは**無し**
- ログ: コンテナ内 `/root/.ros/log/device_container_node_831_1792658558688.log`、`epos4_controller_1003_1792658563671.log`
- 解析スクリプト (セッション一時領域、消える可能性あり): `/tmp/claude-1000/-home-nomface-reRoBot/50274584-fe41-41d1-8b08-9fe1a859e377/scratchpad/{cmdodom,gaps,diag,stall}.py`。コンテナ内 `/tmp/` にも同名でコピー済み。使い方は issue §8
- bus.yml: `ros2_ws_main/src/drivers/epos4compact50-5can/maxon_epos4_ros2/config/bus_config_cia402_epos4_vel/bus.yml` (SYNC 10 ms、TPDO/RPDO transmission 0x01、heartbeat 2000 ms、0x60C2 = 10 × 10⁻³ s)。**submodule 内なので変更は submodule 側にコミットが必要** (CLAUDE.md)。今回は変更不要の想定
- メモリ: EPOS4 の RPDO 途絶許容は実測 ≥ 686 ms (10-11 台上実測)。0x60C2 の 10 ms とは一致しないので、許容時間の解釈は鵜呑みにしないこと
