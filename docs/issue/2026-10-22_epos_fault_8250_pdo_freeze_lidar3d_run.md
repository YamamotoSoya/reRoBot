<!-- claude: docs/issue — 未解決問題の調査記録。解決したらステータスを更新すること。2026-10-22 作成。 -->
# 10-22 3D 自己位置推定で自律走行中、EPOS が両輪同時に Fault 0x8250 (RPDO timeout) — PDO が両方向とも凍結、監視は検知できず

- **ステータス: 調査中・原因未特定**。「何が起きたか」は bag で確定 (§2)。「なぜ PDO が止まったか」は記録不足で層を絞れない (§4)。critic 査読は未実施 (原因を断定していないため)
- 日付: 2026-10-22 18:42:49.6 JST。5号館、`nav_lidar3d` (lidar_localization_ros2 + Nav2 RPP) + waypoint_runner で 0.4 m/s 自律走行中
- bag: `bags/raw/2026-10-22_1825_5goukan_lidar3d` (開始 1792662136.66 = 18:25:36、Fault は bag 時刻 ≈ 1033 s = epoch 1792662169.6)。ログ: `rerobot_env:/root/.ros/log/{device_container_node_831_1792658558688,epos4_controller_1003_1792658563671}.log`
- 関連: `docs/issue/2026-09-19_epos_stop_link_loss_rpdo_timeout_power.md` (0x8250 の前例 = CANUSB の USB リンク喪失 / アダプタ内部リセット、監視の設計 §10.4〜10.5)、`docs/issue/2026-09-02_rfans_scan_motor_dropout.md` (同じ run の 1 分前に起きた別事象 §1)

## 0. 全体像

```
18:41:49   ① R-Fans 回転ディップ (30k→128k 点/frame → 5.3 s 欠落)  ← 本 issue の対象外、別事象
           ├─ 3D 推定が止まり map_3d->odom TF 途絶 → RPP が TF 待ちで /cmd_vel 0.5 s 途絶
           ├─ epos4_controller の指令ウォッチドッグが 0 へランプ → abort → 10 s 後に waypoint_runner の再送で走行再開
           └─ EPOS は無事 (この後 1 分間、普通に走っている)
18:42:49.6 ② 0.4 m/s 走行中に両輪同時に Fault  ← 本 issue
           ├─ 車輪は 0.8 s で滑らかに減速・停止 (EPOS 側のフォルト反応のクイックストップと整合)
           ├─ EPOS は 0x8250 RPDO timeout (§2.2)。LED 赤 (ユーザ目視)。非常停止は押していない (ユーザ)
           ├─ epos4_controller の監視は bag 終了まで「statusword 0x1237・リンク正常」と報告 = 見逃し (§3)
           └─ Nav2 は 50 s 間 0.36〜0.40 m/s を出し続けたが車輪は 1 mrad も動かず、誰も止まったと気づかない
18:43:48   bag 終了 → ユーザが Claude に質問 → 手押しで安全な場所へ移動
18:49:49   SDO 直読み: 両ノードとも statusword 0x0208、0x603F = 0x8250 (SDO は通る)
18:51:38   ユーザが recover → enable → cyclic_velocity_mode で復帰 (CAN アダプタの再起動なし)
```

## 1. ユーザ報告 (原文要約)

走行中に急に止まり、見ると EPOS が赤になっていた。そこで Claude を起動して質問し、その後手押しで安全な場所へ移動した。非常停止スイッチは一度も押していない。

## 2. 確定した事実

### 2.1 Fault の瞬間 (bag)

| 項目 | 値 | 出典 |
|---|---|---|
| 車輪角 (`/joint_states`) | 64.0 s まで 0.4 m/s 相当で増加 → 64.6〜65.46 s で滑らかに減速 → 以後完全に凍結 (3604.45 / 3603.66 rad) | `cmdodom.py` / `gaps.py` (下記) |
| 速度指令 `/robot_speed_cmd` | 凍結後も 50 s 間 0.36〜0.40 m/s (Nav2 由来、twist_mux 経由) | 同上 |
| PC 全体の停止 | **無し**。R-Fans 点群・IMU・TF・lidar_localization・costmap は通常周期で継続 (Fault 前後 ±2 s に 0.3 s 超の穴なし) | `gaps.py` |
| ホストのカーネル | USB / CAN のエラーなし (09-19 15:35 の `urb -32` / `port disabled` は無い)。WiFi ローミングが 18:40〜18:48 に 5 回 (18:42:41 を含む) — 頻度が高いので偶然の可能性が高い | `journalctl -k` |
| 時計 | NTP の時計ステップなし (timesyncd のログなし) | `journalctl` |
| CAN I/F (18:44 時点) | ERROR-ACTIVE、rx/tx errors 0、dropped 0、bus-off 0 | `ip -details -statistics link show can0` |
| device_container | Fault 時刻付近に記録なし。33 s 前 (18:42:16.9) に motor1 の SDO 0x6077 応答タイムアウト ×1 (同じ型のタイムアウトは平常時にも散発: 18:29:42 motor2、18:55:42 motor2、18:56:30 motor1) | `device_container_node_831_*.log` |
| epos4_controller | Fault 時刻付近に記録なし (18:41:49 の指令ウォッチドッグの 2 行が最後) | `epos4_controller_1003_*.log` |

### 2.2 Fault の中身 (18:49:49、SDO 直読み、両ノード同一)

| オブジェクト | motor1 | motor2 | 意味 |
|---|---|---|---|
| 0x6041 statusword | 0x0208 | 0x0208 | Fault (bit 3)。出力段は停止中なので bit 4 (電圧印加) = 0 は当然 |
| 0x603F error code | 0x8250 | 0x8250 | RPDO timeout (補間周期内に同期 RPDO を受けられず、通信中断扱い) |
| 0x1001 error register | 0x10 | 0x10 | 通信エラー |
| 0x1003:01 最新履歴 | 0x8250 | 0x8250 | 同上 |
| 0x2200:01 電源電圧 | 24.3 V | 24.1 V | 正常 → 電源系 (0x3220 型) ではない |

### 2.3 監視は Fault を最後まで「正常」と報告した

bag の `/diagnostics` に出ている `epos4_controller/motor{1,2}` は、Fault 後も bag の終わり (122.9 s) まで **statusword 0x1237 (Operation Enabled)** のままでした。`can_link` も OK、`force_zero=false` でした。

- 監視が `alive` と判定している = `~/rpdo` (0x6041) のメッセージは届き続けている。しかし中身は Fault 前の値のまま。
- 実機は SDO で 0x0208。**EPOS→PC の PDO (TPDO1) も止まっていて、ドライバが最後の値を出し続けていた**と解釈するのが自然 (推定: ProxyDriver が受信 PDO のキャッシュを周期 publish する仕組みかは未確認 — 09-19 §10.4 で `joint_states` が同じ振る舞いをすることは確認済み)。
- つまり **PDO は両方向とも止まり、SDO は生きていた** (少なくとも 6 分後には SDO が通り、アダプタを再起動せず recover で復帰した)。

## 3. 構造的な問題 (原因が何であれ効く)

1. **監視がこの故障型に盲目 (確定)**: `epos4_controller` の監視は `~/rpdo` の**受信時刻**で生存を判定するが、ドライバが古い値を出し続けると「生存・0x1237」に見える。Fault ビットも途絶も検知できない。09-19 20:03 に 0x3220 を捕まえられたのは、TPDO が流れ続けていた (電源だけ落ちた) から。
2. **誰も「止まった」と気づかない (確定)**: Nav2 は 50 s 間 0.4 m/s を出し続け、車輪が動かないことを検知する層が無い (odom の速度 0 と指令の不一致を見る層が無い)。
3. **記録不足 (確定)**: PC→EPOS の経路 (`/motorN/cia402_device_N/tpdo` topic、CAN フレーム) を記録していないので、どの層で止まったかを事後に切り分けられない。

## 4. 仮説 (棄却できていないもの)

| # | 仮説 | 合う点 | 合わない点 / 未確認 | 判別法 |
|---|---|---|---|---|
| H1 | マスタ (lely / device_container) の SYNC 送出が止まった (TPDO・RPDO とも transmission 0x01 = SYNC 駆動。bus.yml `sync_period: 10000` µs) | 両方向の PDO 同時停止・SDO は生存・両輪同時・ログ無し | なぜ止まり、なぜ recover 後に PDO が戻ったか (§5 の 18:51:59 以降 "Internal limit active" = statusword 更新が再開) | candump で 0x080 (SYNC) の有無 |
| H2 | CANUSB アダプタの内部リセット / 送受信停止 (09-19 §10.6 の型) | 両方向停止・両輪同時 | 09-19 は SDO も死んでアダプタ再起動が必要だった。今回は 6 分後に SDO が通り、再起動なしで復帰。カーネルにも痕跡なし | candump + `ip -s link` の tx カウンタ推移、slcand の tty 直読み |
| H3 | EPOS が NMT pre-operational / stopped に落ちた (PDO 停止、SDO 生存) | PDO 停止・SDO 生存 | heartbeat 2000 ms があるのに device_container に NMT 状態変化の記録なし (lely が記録するかは未確認)。両ノード同時に落ちる理由がない | 次回 Fault 時に SDO で NMT 状態 / heartbeat を確認 |
| H4 | `epos4_controller` の 100 Hz 送信 (executor) が止まった | RPDO timeout を直接説明 | **EPOS→PC 側の凍結は説明できない** (TPDO はマスタ・EPOS 間の話)。bag では executor 由来の `/diagnostics` 以外の出力 (`/joint_states` は odometry ノード) から controller の生存は判定できない | `/motorN/.../tpdo` を bag に記録 |

- 18:41:49 の R-Fans 事象 (①) との因果は見えていない (1 分の間隔があり、その間は普通に走っている)。
- 18:42:16.9 の motor1 SDO タイムアウトは CAN 上の一時的な不調の示唆になり得るが、同型が平常時にも散発するので決め手にならない。

## 5. 復帰後 (参考)

- 18:51:38 / 18:51:45 に Fault reset → Enable が通り、18:51:59 以降 device_container に "Internal limit active" が再び出る (statusword の更新が再開した印)。
- 18:55:42 / 18:56:30 に SDO 0x6077 タイムアウトが散発。19:06:05 に SIGINT で終了。

## 6. 次の一手 (安い順)

1〜3 は別エージェントに引き継ぎ (2026-10-22): `docs/claude/handoff/2026-10-22_epos_fault_monitor_fix.md`

1. **記録を足す (原因の層を切り分けるため、コマンド追加のみ)**:
   - bag に `/motor1/cia402_device_1/tpdo` `/motor2/cia402_device_2/tpdo` `/motor1/cia402_device_1/rpdo` `/motor2/cia402_device_2/rpdo` を追加する。
   - 走行中は裏で `candump -l can0` を回す (SYNC 0x080 / RPDO 0x201・0x202 / TPDO 0x181・0x182 の途切れ方で H1〜H4 を判別できる)。
2. **監視を直す (§3-1)**: キャッシュ再送では変わらない量で生存を判定する。候補は (a) TPDO の位置が指令非ゼロなのに変わらない、(b) 定期的に SDO で 0x6041 を読む (09-19 §10.5 で INFO ログが流れる問題があったので頻度を下げる)、(c) ProxyDriver が受信のたびにだけ publish しているかをソースで確認して前提を直す。
3. **指令と実速度の不一致検知 (§3-2)**: 「指令 > 0.1 m/s なのに `/odom` の速度が 1 s 以上 0」で ERROR + 停止指令。
4. **Fault からの自動復帰は入れない (当面)**: 原因不明のまま自動 recover すると、暴走側の故障 (09-19 §6-1) を隠す恐れがある。手動 recover の手順は §7。

## 7. 手動復帰の手順 (今回成功したもの)

main コンテナで実行 (bringup は止めない。速度指令が 0 であることを確認してから):

```bash
source /workspace/install/setup.bash
for m in motor1/cia402_device_1 motor2/cia402_device_2; do
  ros2 service call /$m/recover std_srvs/srv/Trigger
  ros2 service call /$m/enable std_srvs/srv/Trigger
  ros2 service call /$m/cyclic_velocity_mode std_srvs/srv/Trigger
done
ros2 run rerobot_bringup bringup_check.py
```

⚠️ 監視が当てにならないので (§3-1)、Fault の確認は `bringup_check.py` (SDO で 0x6041 を読む) で行う。

## 8. 解析スクリプト

`/tmp/claude-1000/-home-nomface-reRoBot/50274584-fe41-41d1-8b08-9fe1a859e377/scratchpad/` (セッション一時領域、消える前提) の `cmdodom.py` (指令と車輪角の 0.5 s 集計) / `gaps.py` (全 topic の 0.15 s 超の穴 + 車輪角) / `diag.py` (`/diagnostics` の epos4 監視) / `stall.py` (点群・TF・照合の時系列)。いずれも `docker exec rerobot_env bash -c "source /workspace/install/setup.bash && python3 /tmp/<script> <bag> 1792662105 <開始s> <終了s>"` で実行 (コンテナ内 /tmp にコピー済み)。残す場合は `bags/exp/2026-10-22_epos_fault/` へ移す。

## 11. 2026-11-04 再発 (5号館 auto_run、Smac 初走行) — 同型、PC 側は無停止

- Fault: **13:45:42.6 JST** に車輪凍結 (bag t = 1063.1 s、bag `bags/raw/2026-11-04_1327_5goukan_chal_auto_SmacHybridA*`)。凍結の 1.3 s 前から 0.4 m/s → 0 へ滑らかに減速 = クイックストップ (10-22 §2.1 と同じ)。途絶開始は減速開始の ≈0.7 s 前 = **≈13:45:40.6**。指令 `/robot_speed_cmd` は凍結後も 0.40 m/s を出し続けた。
- SDO 直読み (13:49): 両ノード statusword 0x0208、0x603F = 0x1003:01 = **0x8250**。`recover` ×2 で再起動なしに復帰 (13:50:03)。
- 凍結 ±60 s で **PC 側の穴なし** (`bags/exp/2026-11-04_epos_8250/gaps.py`): `/imu/data` (USB, 5 ms)・`/joy` (USB, 18 ms)・`/urg_front/scan` (USB, 25 ms)・`/rfans_driver/rfans_points` (Ethernet, 96 ms)・`/tf`・`/odom`・`/cmd_vel` すべて中央値どおり。→ PC 全体の停止でも USB ハブ/ホストコントローラの停止でもない。**CAN 経路だけ**が止まった。
- `/diagnostics` の epos4_controller 監視は凍結後も statusword 0x1237・link OK (10-22 §2.3 の見逃しと同じ)。
- ホスト: CPU 3.9 GHz performance 固定、サーマル throttle カウンタは累積 146 万だがこの時刻の kernel ログなし。NTP 時計ステップなし。
- **WiFi は証拠にならない**: 13:45:25 roam / 13:45:27 lease / 13:45:44 roam と前後にあるが、途絶開始 13:45:40.6 の ±1 s にログ上の事象はない。走行中は AP lost 13 回 + lease 27 回 / 19.6 分 = 約 30 s に 1 回あるので、時刻が近いのは当然。10-22 §2.1 の「偶然の可能性が高い」を維持。トリガ候補 (DDS のインタフェース変化) としては棄却もしていない。
- ログ消失: device_container / epos4_controller の stdout は `docker exec` の pty 行き・`~/.ros/log` はコンテナ内 → `docker compose down` で消えた。**次回は候補識別の決め手になるので永続化が必須**。

### 11.1 3 事象 (10-11 台上・10-22・11-04) を貫く暫定仮説と次の一手

| 候補 | 支持 | 不利 | 識別法 |
|---|---|---|---|
| (A) device_container の lely マスタスレッドが散発的に ≈0.6〜1 s 停止 → SYNC/RPDO 途絶 | 10-11 台上 candump で SYNC 停止 0.6 s + SDO timeout 群を直接観測 (`bags/exp/2026-10-11_amcl_load/bench/README.md`)。10-22/11-04 とも PC・USB 無事でアダプタも生存 | 何がスレッドを止めるかは未特定 (PDO callback 内の DDS publish ブロック / pty への log 書き込みブロック / SDO timeout 処理) | ホストで `candump -t a -l can0` を走行中ずっと並走: 0x080 (SYNC) が止まれば (A)、SYNC は出て 0x181/182 が返らなければ (B)/(C) |
| (B) CANUSB/slcan 経路の一時停止 (09-19 の内部リセットの軽い版) | 09-19 でアダプタ起因の実績あり | 今回はリセットなし (同じソケットで SDO 通る)。10-11 の SYNC 停止はホスト側 candump で見えた = カーネルに届く前に止まった | 同上 + `ip -s -d link show can0` の dropped/bus-off |
| (C) バス側 (EMI・エラーパッシブ) で両 EPOS が同時に取りこぼし | 両輪同時を説明できる | 0x1003 に 0x8120/0x81FD 系なし (09-19 と同じ)。台上でも起きている | candump のエラーフレーム (`-e`) + 0x1003:02〜05 |

原因が何であれ効く緩和 (安い順): ① candump 並走 + ログ永続化 (計装、次走行から)、② epos4_controller に Fault 自動 recover (joint_states 凍結 ∧ 指令≠0 → SDO 0x6041 確認 → recover → 指令を 0 から再ランプ)、③ `ROS_LOCALHOST_ONLY=1` で DDS を WiFi から切る (全ノード同一 PC・host network なので副作用小、WiFi トリガ説の A/B にもなる)。EPOS 側の RPDO timeout 延長は **非推奨** (09-19 15:35 で暴走を止めたのがこの機能)。

### 11.2 監視の見逃し (§2.3) の正体候補: `~/rpdo` がプロセス起動時間の 1/3 遅れて届く (有力・critic 未査読)

11-04 bag で、13:45:42 (bag 1063 s) の Fault が `/diagnostics` の statusword には **13:56:05 (bag 1685 s)** に初めて 0x0208 として現れ、その時点の SDO live 読みは「0x603F = 0・Vcc 24.2 V・車輪は 0.8 m/s で回転中」(`bags/exp/2026-11-04_epos_8250/window.py`)。bag 終端 (1783 s) まで 0x0208 のまま。プロセス起動 (13:24:40) 基準で 1262 s の事象が 1884 s に届いた = 比 **1.49**。

機構: ros2_canopen (jazzy) のドライバは `polling: true` 既定で `period` (bus.yml defaults **50 ms**) ごとに rpdo キューから最大 **10 件**取り出す = 200 件/s。TPDO1 は SYNC **10 ms** ごとに 3 オブジェクト = 300 件/s。差 100 件/s がキューに滞留し、遅れは起動時間の 1/3 で成長 (予測比 1.5 ≈ 実測 1.49)。→ §2.3 の「ドライバがキャッシュを周期 publish」推定は撤回、**キュー遅延**が本命候補。09-19 §10.5 の `~/rpdo` 購読型監視は起動数分後から無効。対策と自動復帰の設計は `docs/features/2026-11-05_epos_auto_recovery_plan.md` §2.1 / §3.2。

## 12. 2026-11-06 3 回目 (5号館 auto_run_amcl、bag `2026-11-06_1201_5goukan_amcl`) — 同型、SYNC は生きていた (critic 査読済み)

観測 (実測):
- 両輪同時 Fault、0x603F = **0x8250**、履歴 0x8250 のみ、Vcc 24.2 / 24.1 V。runner を一時停止 → `recover` 1 発で 0x1237 / CSV 9 に復帰 (12:20 頃)。
- `/joint_states` (bag) で **12:07:19.3** から両輪が 0.35 m/s → 0 を約 0.9 s で滑らかに減速、12:07:20.8 で静止。この間も 50 ms 周期で欠けずに値が更新 (位置は 0x6064 = TPDO マップで dictionary 直読み、§11.2 のキュー遅延を受けない — critic がヘッダで確認)。`/robot_speed_cmd` は 0.33〜0.36 m/s で連続 (隙間 ≤55 ms)。controller の FAULT ログは 12:10:22 (§11.2 の遅延)。
- ホスト journal に 12:07:19 前後の CAN/USB/kernel 異常なし (WiFi ローミングは 12:07:11〜14 で完了済み)。`ip -s link` errors/dropped 0、`tc -s qdisc` dropped 0 / **requeues 86395 / sent 319390**。
- 背景: 12:00 に AC を抜いてバッテリ、power profile **balanced** (performance 固定は外れていた)、全コア 1.9 GHz、load 32.9 (8 スレッド)、ekf 周期未達・planner 0.46 Hz。

critic 判定と表現:
- **言えること**: EPOS の TPDO (SYNC 起動) が減速中も届いていた → 0.6 s 級の「lely マスタ丸ごと停止」は今回の位置データと合わない。
- **区別できていないもの (棄却できない代替仮説)**: (A') lely スレッドの数十 ms の短い停止 (0x60C2 = 10 ms = SYNC 周期で余裕 0、09-19 H1)、(B') slcan/CANUSB の TX 詰まり・まとめ送り (仕様上 timing violation でも 0x8250。詰まると SYNC が先頭で PDO が後ろ)、(C) 目標≠0 のときの EPOS 判定条件が別物 (10-11 の「目標 0 で 686 ms 許容」とは別)。
- requeue 27 % は slcan の設計上平常でも出うる (基準値なし) ので背景要因とは呼ばない。11-04 は performance 3.9 GHz で起きているので CPU 低速は共通要因ではない。
- 減速形状は quick stop 後の惰行と整合 (0x6085 と照合すれば fault reaction 確定)。

次の 1 実験: 台上で車輪を浮かせ **目標≠0** で回しながら `candump -t a -l can0` 並走、device_container を `kill -STOP; sleep X; kill -CONT` で X = 20/50/100/300 ms 止め、Fault する最短 X を測る。20 ms 級なら (A')/(B') = 余裕 0 が主因候補 → 0x60C2 を広げる対策へ。数百 ms なら (A)(A') 棄却で、走行中 candump による (B')/(C) 切り分けへ。解析スクリプト `bags/exp/2026-11-06_epos_8250/` (書き込み中 MCAP をそのまま読める最小パーサ)。
