<!-- claude: docs/features — 設計文書。2026-11-05 作成、2026-11-07 v2 (11-06 の heartbeat 対策と台上試験を受けて再計画)、11-06 夜に実装。実装後に「使い方」「変更ファイル」を実態に合わせて更新すること。 -->
# EPOS 自動復帰 — 一時的な停止 (通信・段差) は自動で走行再開、非常停止後は人が Start するまで走らない

- ステータス: **実装済み・擬似 EPOS の結合試験 25/25 合格・実機未検証・未コミット** (11-06 夜。§11)。計画 v2 (critic 査読 §10)。**11-06 夜にユーザ決定 (§8)**: 0x3220 は両輪停止も自動復帰、非常停止はラッチ式で解除まで最短 3 s → 判定は「Fault 後 1 s 以内に読んだ Vcc ≥ 20 V」の 1 本、0x8250 も自動復帰、DISARMED 中の LB joy は通す。T0 は実装後の T2/T3 に統合。ユーザ要求: 「EPOS 謎停止と、段差乗り上げで DC-DC が供給できなかった電圧低下停止は自動復帰したい。ただし非常停止後の復帰で勝手に走り出さないこと」
- v1 (11-05) からの変更: 通信途絶の主コードが **0x8130 (heartbeat)** になった (11-06 対策 a75f16b)。0x8250 は原因不明の別物として扱う。CANUSB 喪失型は自動復帰の対象外。controller の再起動直後も「走らない」側から始める。`~/rpdo` 遅延の修正は本機能から切り離した。
- 根拠: `docs/issue/2026-10-22_epos_fault_8250_pdo_freeze_lidar3d_run.md` §11〜12.3 (0x8250 ×3 回・台上 67 回・USB 抜去・heartbeat 適用)、`docs/issue/2026-09-19_epos_stop_link_loss_rpdo_timeout_power.md` §10.1/§10.7 (非常停止・段差の実測コードと電圧)、台上手順 `bags/exp/2026-11-06_epos_8250/bench/` (git 管理外)
- 関連: 現行の監視 `docs/features/2026-09-19_epos4_controller_watchdog_fault_monitor.md`、脱力モード `docs/features/2026-06-05_motor_free_mode.md`、運用 `docs/features/2026-10-16_auto_run_joy_mux.md`

## 1. 目的・概要

```
EPOS が止まった
├─ 一時的な停止 → 自動 recover → 指令 0 からランプして走行継続
│   ├─ 0x8130 heartbeat error (マスタが 0.5 s 以上黙った。can0 は同じ)   ← 11-06 から主コード。誤停止もここに来る
│   ├─ 0x8250 RPDO timeout (原因不明。10-22・11-04・11-06 の 3 回とも recover 1 発で復帰)  ← §8-2 で要判断
│   └─ 0x3220 電圧低下 (片輪でも両輪でも) で、Fault 後 1 s 以内に読んだ Vcc が ≥ 20 V (段差で DC-DC が一瞬落ちて戻った)
├─ 非常停止扱い → 電源が戻ったら enable だけする → DISARMED (指令 0 固定) → Start で再開
│   └─ 上以外の 0x3220 (1 s 以内の読みが < 20 V・読めない・読みが 1 s に間に合わない)   ← 迷ったらこちら
├─ プロセス内では戻せない → DISARMED + 「bringup 再起動」を表示
│   └─ 0x8130 + can0 が消えた / 作り直された (CANUSB の USB 喪失。master が旧 socket を掴む)
└─ それ以外 (過電流 0x2310・追従誤差 0x8611・熱・不明) → DISARMED (止まったまま報告)
```

鍵は **「復帰 (motor を enable する)」と「走行許可 (指令を通す)」を分ける**こと。今は指令が EPOS に直通なので、recover した瞬間に Nav2 の残っているゴールで走り出す (§2.2)。

## 2. 前提 (確定していること)

### 2.1 通信途絶で EPOS を止めるのは heartbeat 監視 (11-06 適用・台上確認・実走未検証)

bus.yml (submodule `10557f8`): master `heartbeat_producer: 100` × `heartbeat_multiplier: 5.0` → EPOS の 0x1016 = node 6 / 500 ms、defaults `error_behavior: {1: 0x01}` (0x1029:01)。

```
台上 (11-06、車輪浮かせ・目標≠0)
├─ 対策前: マスタ全停止 20 ms〜10 s・SYNC のみ停止・RPDO 重複/圧縮・NMT Pre-Op → Fault 0 (67 回)
│          回転中に CANUSB を抜く → Fault せず最後の速度で 58 s 回り続けた
└─ 対策後: マスタ停止 ≤400 ms → 回転継続 / ≥600 ms → 0.43〜0.50 s 後に 0x8130 → quick stop
           statusword 0x0208 (NMT Operational のまま) → recover 1 発で 0x1237
           CANUSB 抜去 → 即停止 (LED 赤)、can0 再作成のため recover では戻らない
```

この計画にとっての意味:
- 通信系の停止は今後 **0x8130 が主**。マスタの 0.6 s 級の一時停止 (10-11 台上で稀に観測) でも止まる = **誤停止が実走で出うる**。自動復帰はこれを「一瞬の息継ぎ」に変える役も持つ。
- 0x1029:01 = 1 が前提。既定 0 だと Pre-Operational に落ちて PDO が止まり、recover が届かない (手動なら `cansend can0 000#0100` → recover)。

### 2.2 Nav2 のゴールは停止中も死なない

controller_server の `movement_time_allowance: 3600` (10-15) なので、EPOS が止まっても Nav2 は同じゴールへ速度を出し続ける (11-04 は 4 分間 0.4 m/s)。→ recover した瞬間、または **controller を再起動した瞬間**に走る。一時的な停止にはこれが都合よく、非常停止と再起動には危険。

### 2.3 停止の種類は EPOS に聞けば分かる — ただし 0x3220 の中身は電圧では決まらない

| 種類 | 0x603F | 両輪 | 検知直後の 0x2200:01 (Vcc) | can0 | 出典 |
|---|---|---|---|---|---|
| マスタ一時停止 | 0x8130 | 同時 | 24 V | 同じ ifindex | 11-06 台上 |
| CANUSB 喪失 | 0x8130 | 同時 | 読めない (SDO 不通) | 消滅 / ifindex 変化 | 11-06 台上 |
| 原因不明の通信系 | 0x8250 | 同時 | 24 V | 同じ | 10-22・11-04・11-06 実走 |
| 段差ブラウンアウト | 0x3220 | 片輪ずつ (26 s 差) | 24 V に即復帰 | 同じ | 09-19 §10.7 |
| 非常停止 | 0x3220 | 同時 | 8 V 前後 → 0 V | 同じ | 09-19 §10.1 |

識別力があるのはコード種別 (通信系 0x8130/0x8250 か電源系 0x3220 か) と can0 の ifindex。**0x3220 が段差か非常停止かは、Vcc の読み値では決まらない** (critic、§10):
- 0x3220 は Vcc < 10 V で出る (0x2201:01 = 10000)。段差でも非常停止でも Fault の瞬間は 10 V を割っている。
- だから後で読んだ Vcc が示すのは原因ではなく「読んだ時点で電源がもう戻っているか」。非常停止を短く押して戻せば、読む前に 24 V に戻り、段差に見える。
- 段差の根拠は 09-19 §10.7 の 2 回 (2 回目は 1 輪駆動)、非常停止の 8 V は §10.1 の 1 回 (静止・押しっぱなし)。段差の 0x3220 の原因も H5 (RSD 折り返し) が本命だが接触不良 (A2・上流接点、§10.10 の両輪同時 0x3220) は未棄却。

誤分類のコストは非対称 (段差を非常停止と誤る = Start を 1 回押すだけ、非常停止を段差と誤る = 勝手に走る)。

**ユーザ回答 (11-06 夜): 非常停止はラッチ式で、押してから解除まで早くても 3 s。** これで「後で読む Vcc は解除の速さを測る」という critic の指摘が逆に使える:
- 非常停止中は RSD の入力が切れているので、Fault 後の Vcc は下がる一方 (09-19 §10.1: 0.5 s 以内に 8 V)。解除は 3 s 以上先なので、**Fault から 1 s 以内に読めば必ず < 20 V**。
- 段差ブラウンアウトは負荷が抜けた時点で 24 V に戻る (09-19 §10.7)。1 s 以内の読みで ≥ 20 V なら段差。
- 片輪か両輪かは使わない (ユーザ指示: 両輪停止も自動復帰。09-19 §10.10 の両輪同時 0x3220・24 V もこれで自動復帰側に入る)。

規則:
- `can0 消滅/変化` → 再起動要
- `0x8130` → 一時的 / `0x8250` → 一時的 (別予算、§3.1)
- `0x3220` → **Fault 後 1 s 以内**に両輪の Vcc を読み、Fault した輪がすべて ≥ 20 V なら一時的。1 輪でも < 20 V・読めない・1 s に間に合わない (EMCY を取りこぼして物理トリガで遅れて気づいた等) なら**非常停止扱い**
- その他 → DISARMED

残るリスク: 非常停止を 1 s 未満で解除した場合 (ラッチ式なので通常は起きない)、Vcc の読みが遅れた場合 (1 s 制限で非常停止側に倒す)。

### 2.4 recover 1 発で CSV Operation Enabled に戻る

0x8250 は実走 3 回とも、0x8130 は台上で、`/motorN/cia402_device_N/recover` だけで statusword 0x1237・mode 9 に戻った。enable / cyclic_velocity_mode の呼び直しは不要。

### 2.5 監視の `~/rpdo` は遅れて届く (未修正、機構はコード上確認) → 本機能は使わない

11-04 bag で statusword が起動時間の約 1/2 遅れて届いた (13:45 の Fault が監視には 13:56)。ドライバが 50 ms ごとに最大 10 件 (200 件/s) しか取り出さないのに TPDO は 300 件/s 入るためと推定 (issue §11.2、critic 未査読)。取り出し上限 10 件/周期は jazzy の `node_canopen_base_driver_impl.hpp` で critic が確認した。bus.yml の `period: 50` は 11-06 の変更後も同じ。→ **検出は EMCY の受動受信と SDO で行う** (§3.2)。EMCY は rpdo とは別のキューで量も少ないので遅れない。遅延の修正は `/diagnostics` を正しくするための別作業にする (§8-5)。

## 3. 設計

### 3.1 状態機械 (epos4_controller の監視スレッド)

```
起動 ──► DISARMED ◄─────────────────────────────────────────────────────────┐
           │ ARM (パッド Start / ~/arm)                                        │
           ▼                                                                 │
        RUNNING ──(Fault 検知)──► 分類                                        │
           ▲                       ├─ 一時的 ──► RECOVERING ──(両輪 ready)──┐  │
           │                       │              │ 失敗 / 予算超過 / 5 s 超 ──┼──┤
           │◄──────────────────────┼──────────────┘◄────────────────────────┘  │
           │                       ├─ 非常停止 ──► WAIT_POWER ──(Vcc≥20 V 1 s)──► recover ──┤
           │                       ├─ 再起動要 ─────────────────────────────────────────────┤
           │                       └─ その他 ──────────────────────────────────────────────┘
           └── DISARMED 中: 目標 0 固定 (例外: LB を握った joy、§3.3)
```

- **起動直後は DISARMED** (v2 で追加)。bringup を別端末で再起動したとき、Nav2 の残ゴールで走り出すのを防ぐ。通常運用ではもともと最初に Start を押すので手間は増えない。台上のキーボード teleop は `ros2 service call /epos4_controller/arm ...` を 1 回叩く (params に「起動時から許可」の設定は置かない — 台上用の値のまま本番に持ち込む事故を避けるため)。
- **RECOVERING**: Fault を検知した瞬間に両輪の目標を 0 に固定し、ランプ出力も 0 にリセット (片輪 Fault でもう片輪が回り続けて旋回するのを止める)。recover の**前**に Vcc と 0x603F を読み直し、分類が変わっていれば (途中で非常停止を押した等) その分類へ移る。recover → SDO で 0x6041/0x6061 を読んで両輪 ready → RUNNING の**直前**にもう一度 Vcc を読む。Nav2 のゴールは生きているので、指令は既存のランプ (15000 rpm/s) で 0 から戻る。
- **予算** (critic: 窓だけでは 40 s おきの停止を 1 時間 90 回許す):
  - 0x8130 と 0x3220 (段差、両輪でも): 2 分間に 3 回、かつ 1 回の起動あたり 10 回まで。
  - 0x8250 (原因不明): 1 回の起動あたり 3 回まで、かつ復帰後 30 s 以内に同じコードが再発したら DISARMED。
  - 超えたら DISARMED。0x8130 の誤停止が実走でどれくらい出るかを見て調整する。
- **長期化**: Fault 検知から 5 s 以内に RUNNING へ戻れなければ DISARMED。
- **WAIT_POWER**: Vcc を 1 Hz で読み、20 V 以上が 1 s 続いたら recover して DISARMED へ。モータは enable 済みなので手で押しても動かない (押したいときは既存の脱力 f)。
- **再起動要**: 既存の can0 消滅/ifindex 変化の検知をそのまま使い、DISARMED にして「bringup 再起動が必要」と表示する。EPOS は heartbeat で止まっているので安全。
- **executor の停止監視** (critic の見落とし指摘): controller の executor だけが止まると、device_container は生きていて heartbeat は正常、RPDO は最後の目標速度を送り続け、指令ウォッチドッグも同じ executor にあるので誰も止めない。監視スレッドが executor の tick を見て、0.3 s 以上止まっていたら監視スレッドから目標 0 を直接 publish する。

### 3.2 検出 — EMCY の受動受信を主経路にして ms 級で気づく

```
├─ (1) 主: EMCY の受動受信。controller が can0 に読み取り専用の raw ソケットを開き、0x081 / 0x082 (EPOS の EMCY) だけ受ける
│      EMCY はエラーコードを含むので、届いた瞬間にコードと輪が分かる。lely を経由しないので rpdo 遅延・マスタ停止の影響を受けない
│      (11-06 台上で 0x8130 の EMCY を candump で確認済み。0x3220・0x8250 で EMCY が出るかは T0 で確認)
├─ (2) 裏付け: EMCY を受けたら、両輪に SDO で 0x2200:01 (Vcc) を**最初に**読み (Fault 時刻からの経過を記録、1 s 超は無効)、次に 0x603F・0x6041。0x1003 の履歴は分類の後に回す
│      (現行の読み順は Vcc が最後で、遅いと数 s かかる — critic)
├─ (3) 予備: 物理トリガ。輪ごとに「実速度 / ランプ後の出力指令 < 0.3 が 0.2 s 続く」→ 即 SDO 読み (EMCY を取りこぼしたとき用)
│      片輪だけの停止と quick stop の減速を両方拾う。DISARMED 中は出力が 0 なので発火しない
├─ (4) can0 の消滅・ifindex 変化 (現行どおり) → 再起動要。raw ソケットのエラーも同じ扱いでソケットを開き直す
└─ (5) SDO が両輪とも 2 回続けてタイムアウト → 従来の link_lost (目標 0)
```

定期的な SDO ポーリングはしない。そのため driver の "SDO Read Call" ログは事象のときだけ数行出る程度で、logger level を変える必要もない (WARN に上げると手動 bringup の目印「Switched NMT state to START」まで消えるため、v2 初版の案は取り下げ)。recover は監視スレッド上で同期実行されるので、実行中は両輪とも目標 0 に固定しておく。

### 3.3 「走らない」の保証は controller の DISARMED、runner の cancel は 2 重目

- controller は DISARMED 中、`/robot_speed_cmd` の中身によらず目標 0 を出す。twist_mux より下流なので、Nav2・RViz 単発ゴール・runner の有無に関係なく効く。
- **例外 (要判断 §8-1)**: DISARMED 中は `/robot_speed_cmd` ではなく **`/joy_vel` を直接読み、0.2 s 以内に届いたものだけ**通す。`/joy` も 0.2 s 以内に届いていて LB (`enable_button` 4) か RB (`enable_turbo_button` 5) が押されていることを条件にする (11-06 修正: teleop_twist_joy は RB 単独でも turbo で指令を出すので、LB だけを見ると RB 単独の操作が 0 に潰れた — 20:02 bag で RB のみ 79 回中 4 回しか動かず)。
  - `/robot_speed_cmd` を通すと、LB を握ったままパッドが USB 瞬断したとき (joy_node はゼロを出さず沈黙する、09-19 §6-1) に controller が「LB 押下」を保持し続け、twist_mux が Nav2 に落ちた指令が DISARMED のまま通ってしまう (critic)。
- ARM = `/joy` の Start (`arm_button` 7) の押し下げ、または `~/arm` (std_srvs/Trigger)。controller 自身が `/joy` を読むので、runner を別端末で動かす運用でも効く。押し下げの判定は厳しくする (critic):
  - 受け付けるのは DISARMED 中だけ。WAIT_POWER・RECOVERING 中の押下は捨てる (溜めない)。
  - 「離されている」状態を一度見てから初めて押し下げを数える。パッドの再接続や joy_node 再起動の最初のメッセージで Start が押されていても ARM しない。
  - `/joy` のボタン数・軸数が Xbox (XInput) の構成と違えば `/joy` を無視する (DirectInput で再列挙されるとボタン番号が変わる)。
- 状態は `/epos4_controller/state` (std_msgs/String) に 2 Hz で出す。例: `RUNNING` / `RECOVERING 0x8130 2/3` / `WAIT_POWER 8.1V` / `DISARMED 非常停止 — Start で再開` / `DISARMED CANUSB 喪失 — bringup 再起動`。
- waypoint_runner はこれを読み、`WAIT_POWER` と `DISARMED` で走行中のゴールを取り消して一時停止 (`paused`) に入り、RViz の車体上テキストに理由を出す。この一時停止からは、**controller の状態が `RUNNING` に変わったのを見てから**再開する (Start は controller の ARM にだけ効かせる。ARM は人の Start でしか起きないので、再開は人の操作が起点のまま)。`RECOVERING` では何もしない (ゴールを生かしておく)。

### 3.4 再発進の安全

- 指令は必ず 0 からランプする。RPP の衝突判定と costmap は停止中も回っているので、前に物があれば Nav2 が 0 を出す。
- 自動復帰は症状の緩和で、0x8250 と マスタ停止の原因対策ではない。candump 並走 (issue §11.1) は並行して続ける。

### 3.5 非常停止を直接見る (保留 — ラッチ式なので当面不要)

ユーザ回答で非常停止はラッチ式 (解除まで ≥3 s) と分かったので、§2.3 の 1 s 規則で足りる見込み。T3 で 1 回でも段差に化けたら以下を検討する。0x3220 から推測せず非常停止の状態そのものを読む案:候補: 非常停止スイッチの補助接点、または RSD 入力電圧 (分圧・フォトカプラ経由) を EPOS4 のデジタル入力に入れ、0x60FD などで読む (オブジェクト番号・入力電圧範囲はデータシートで要確認)。配線変更なのでハードの判断が要る。入れば「非常停止中 / 解除済み」が直接分かり、0x3220 は全部「段差 = 一時的」にできる。

## 4. データフロー

```
/joy ─┬─► teleop_twist_joy ─► /joy_vel ─┐
      │                                 ├─► twist_mux ─► /robot_speed_cmd ─► epos4_controller ─► tpdo ─► EPOS
Nav2 ─┼─► /cmd_vel ─────────────────────┘                                     ▲  │  (heartbeat 500 ms は EPOS 側)
      │ (Start / LB)                                                          │  │ SDO 0x6041 2 Hz / 0x603F / 0x2200:01 / recover
      └───────────────────────────────────────────────────────────────────────►│  ▼
waypoint_runner ◄── /epos4_controller/state ◄──────────────────────────── 状態機械 ◄── joint_states (物理トリガ)、can0 ifindex
   └─ cancel_goal + RViz「⛔ 非常停止 — Start で再開」
```

## 5. 使い方 (実装後、オペレータ目線)

- 起動: 今までどおり。最初の Start で走行許可と区間開始が同時に入る。
- 通信・段差で止まった: 何もしない。数秒で再発進する (RViz に `RECOVERING` が一瞬出る)。2 分に 3 回を超えたら止まったまま `DISARMED` になる。
- 非常停止: 押す → 解除 → EPOS は緑 (enable 済み) だが動かない → 周囲を確認して **Start** → 区間の続きから再開。手で動かすときは LB を握って joy。
- CANUSB が抜けた: `DISARMED CANUSB 喪失` → bringup を再起動 → Start。
- 確認と手動操作:
  ```bash
  ros2 topic echo /epos4_controller/state
  ros2 service call /epos4_controller/arm std_srvs/srv/Trigger
  ```

## 6. 変更ファイル (予定)

| ファイル | 変更 |
|---|---|
| `ros2_ws_main/src/app/epos4_controller/src/epos4_controller.cpp` | 監視ループを状態機械に置換、EMCY 受動受信 (raw ソケット) + SDO の読み順変更 + 輪ごとの物理トリガ、分類と recover、executor 停止監視、`/joy` と `/joy_vel` の購読 (Start/LB、鮮度判定)、`~/arm`、`~/state` |
| `ros2_ws_main/src/bringup/rerobot_bringup/config/params.yaml` | `vcc_ok_v 20.0` / `recover_budget {window_s 120, per_window 3, per_run 10}` / `recover_budget_8250 {per_run 3, refault_s 30}` / `long_fault_s 5` / `executor_stall_s 0.3` / `arm_button 7` / `enable_button 4` / `allow_joy_when_disarmed true` / `joy_fresh_s 0.2` / `auto_recover_8250 true` |
| `ros2_ws_main/src/bringup/rerobot_bringup/scripts/waypoint_runner.py` | `/epos4_controller/state` 購読 → cancel + paused、RViz に理由、`RUNNING` を見て再開 |
| `ros2_ws_main/src/app/epos4_teleop/` (キーボード teleop) | コード変更なし。台上では `~/arm` を 1 回叩く旨をコメントに |
| docs (本文書・issue・PROJECT_STATE・CLAUDE.md の controller 節) | 実装後に更新 |

bus.yml (heartbeat) は 11-06 に済み。launch は変更不要 (定期 SDO をやめたのでログ抑制が要らない)。CMakeLists.txt / package.xml も変更不要 (std_msgs・std_srvs・sensor_msgs は依存済み、raw ソケットは Linux 標準ヘッダだけ)。

## 7. 検証計画 (車輪浮かせ → 接地)

旧 T0 (実装前の非常停止押下時間試験) は、ラッチ式の回答を受けて T2/T3 に統合した。T2/T3 では裏でホストの `candump -ta can0,081:7FF,082:7FF,581:7FF,582:7FF` を回し、EMCY 0x3220 が ms 級で出るか・Vcc の読みが Fault から何 ms 後か・その時の値を記録する。

| # | 試験 | 期待 |
|---|---|---|
| T1 | Nav2 ゴール中に `kill -STOP <device_container PID>; sleep 1; kill -CONT <PID>` | 0x8130 → ≤2 s で `RECOVERING 0x8130 1/3` → `RUNNING`、車輪が 0 からランプして再回転 |
| T2 | Nav2 ゴール中に非常停止 → 5 s 後に解除 | `WAIT_POWER` → runner がゴール取り消し・表示 → 解除後 enable (緑) だが回らない → Start で再開 |
| T3 | 非常停止を押して 3 s 後に解除 × 5 回 (走行中・静止中) | 全回とも非常停止扱い。ログの「Fault → Vcc 読み」の経過が 1 s 未満・値 < 20 V。1 回でも一時的に化けたら §3.5 へ |
| T4 | T1 を 60 s に 4 回 | 4 回目で `DISARMED 予算超過` |
| T5 | DISARMED 中に LB + スティック / スティックだけ | 前者は動く、後者は動かない |
| T6 | 回転中に CANUSB を抜く | 即停止 (heartbeat)、`DISARMED CANUSB 喪失`、自動 recover しない |
| T7 | Nav2 ゴールを生かしたまま bringup だけ再起動 | 起動後 DISARMED で回らない。Start で走る |
| T7b | DISARMED 中に LB を握ったままパッドの USB を抜く / Start を押したままパッドを挿し直す | 前者: 0.2 s で指令 0。後者: ARM しない |
| T7c | controller の executor だけ止める (テスト用の重い callback を一時的に仕込む) | 0.3 s で目標 0 |
| T8 | 実走: 0.3 m/s で段差 (09-19 §10.7 と同条件)、bag + candump | `RECOVERING 0x3220` → 再発進 |
| T9 | 実走: 30 分以上の自律走行 | 0x8130 の誤停止回数を数え、予算と `heartbeat_multiplier` を決める |

## 8. 決めてほしい点 (推奨を先頭に)

**ユーザ決定 (11-06 夜)**: 1 = 通す、2 = 自動復帰させる、0x3220 は両輪停止も自動復帰、非常停止はラッチ式 (解除まで ≥3 s)。6・7 は 7 の回答で当面不要。3〜5 は推奨値で進める。

1. **DISARMED 中に LB を握った joy を通すか** — **決定: 通す** (鮮度 0.2 s の条件付き、§3.3)。
2. **原因不明の 0x8250 も自動復帰させるか** — **決定: させる**。別予算 (1 起動 3 回、復帰後 30 s 以内の再発で停止)。
   - 注意: issue 2026-10-22 §6-4 は「原因不明のまま自動 recover すると暴走側の故障を隠す恐れがあるので当面入れない」としていた。予算と再発検知で縛る、というのがこの計画の答え。
   - 実績の 3 回は Fault の 4〜13 分後に静止状態で手動 recover したもので、**直後の自動 recover は一度も試されていない** (台上で 0x8250 を再現できないため、実走が初回になる)。
3. **予算と長期化の閾値** — 推奨: §3.1 の値と 5 s。T9 の結果で見直す。
4. **起動直後を DISARMED にするか** — 推奨: する。通常運用は最初に Start を押すので手間は増えず、別端末で bringup を再起動したときの暴発を防げる。
5. **`~/rpdo` 遅延の修正 (本機能とは別作業)** — 本機能は rpdo を使わないので急がない。修正案は 3 つあり、どれも副作用がある:
   - `transmission: 0x05` (TPDO を SYNC 5 回に 1 回): joint_states の周期は 20 Hz のままだが、位置の古さが最大 50 ms になり、publish タイマーとの位相ずれで odom の速度に 0 ↔ 2 倍のスパイクが乗る (EKF が odom の twist を融合しているので A/B が要る)。
   - `period: 10` / `polling: false`: joint_states が 100 Hz になり odometry・bag に波及する。
   - 推奨: 先に台上で遅延を実測してから選ぶ。
6. **非常停止を直接見るハード改造 (§3.5)** — 当面しない。T3 で化けたら再検討。
7. **非常停止スイッチはラッチ式か** — **回答: ラッチ式、解除まで最短 3 s**。§2.3 の 1 s 規則の根拠。

## 9. 既知の制限

- 0x8250 とマスタ一時停止の根本原因は未解決。自動復帰は緩和。
- CANUSB 喪失は自動復帰しない (bringup 再起動が要る)。
- 非常停止の判定は「ラッチ式で解除まで ≥3 s」と「Fault 後 1 s 以内に Vcc を読める」の 2 つに依存する。非常停止を別の型 (押している間だけ切れる型) に替えたら成り立たない。電源構成 (RSD-60L-24 の出力コンデンサ) を変えたら T2・T3 をやり直す。
- 段差で Vcc の戻りが 1 s より遅い場合は非常停止扱いになり、Start が 1 回要る (安全側)。
- 両輪同時の 0x3220 で Vcc が戻っているものは、上流接点の瞬断 (09-19 §10.10、非常停止系統を含む) でも自動復帰する (ユーザ判断)。接点が切れたままなら Vcc は戻らないので非常停止扱いになる。
- 0x8130 の誤停止頻度は未知 (T9)。多ければ `heartbeat_multiplier` 5 → 8 (監視 800 ms、0.4 m/s で 0.32 m) を検討。

## 10. critic 査読の記録 (2026-11-07、v2 初版に対して)

| 主張 | 判定 | 計画への反映 |
|---|---|---|
| 1. Vcc で段差と非常停止を分ける | **棄却**: 0x3220 は Vcc < 10 V で出るので、後で読む Vcc は原因でなく解除の速さを測る。短い押下・15〜20 V の保留帯・RECOVERING 中の押下で非常停止が段差に化けうる。両輪同時 0x3220 (09-19 §10.10) は上流接点の瞬断が本命寄り | §2.3 を安全側の規則に変更 (片輪だけ・他輪健在・最初から ≥20 V のときだけ一時的)、Vcc を最初に読む、recover 前と RUNNING 前に読み直す、T0 を実装前に追加、§3.5 ハード案 |
| 2. DISARMED と LB 例外 | **弱める**: LB を握ったままのパッド瞬断で古い LB が残り Nav2 が通る、再接続直後の偽の Start、WAIT_POWER 中の Start の扱いが未定義 | `/joy_vel` 直読み + 0.2 s 鮮度、Start は DISARMED 中の押し下げのみ・離れを見てから・ボタン数検査、runner は RUNNING を見て再開 |
| 3. 0x8130 / 0x8250 の自動復帰 | 0x8130 は持ちこたえる。0x8250 は**弱める** (直後の recover は未試験、10-22 §6-4 と対立、窓予算は長時間の繰り返しを縛らない)。executor だけ止まる型を誰も止めない | 0x8250 は別予算・再発検知、1 起動あたりの上限、§8-2 に対立を明記、executor 停止監視を追加 |
| 4. SDO 2 Hz + 物理トリガ | **弱める**: 両輪凍結条件は片輪 Fault の旋回を拾わない、物理トリガは 2 Hz より遅い、SDO 連鎖が遅い、logger WARN は NMT ログも消す | EMCY 受動受信を主経路に、輪ごとの速度比トリガ、定期 SDO と logger 変更をやめる |
| 5. transmission 0x05 でも 20 Hz | 周期は持ちこたえる。位置の鮮度悪化と odom 速度のスパイクを見落とし | §8-5 の表現を修正 |

棄却できなかった代替: 段差の 0x3220 は RSD 折り返し (H5) か接触不良 (A2・上流接点) か未決。0x8250 の原因は未解明。非常停止スイッチがラッチ式か、RSD の再起動時間は未確認。

## 11. 実装 (2026-11-06 夜)

| ファイル | 中身 |
|---|---|
| `ros2_ws_main/src/app/epos4_controller/src/epos4_controller.cpp` | `~/rpdo` 購読と poll_motor を削除。状態機械 (`state_`、DISARMED 起動)、EMCY 受動受信スレッド (`emcy_loop`、raw ソケット 0x081/0x082)、分類と自動復帰 (`handle_fault` → `do_recover`、Vcc を最初に読む・recover 前と RUNNING 前に読み直す)、予算、`wait_power_step`、`handle_arm`、物理トリガ、executor 停止監視 (`publish_zero_direct`)、`/joy`・`/joy_vel`・joint_states 購読、`/epos4_controller/arm`・`/epos4_controller/state`、`/diagnostics` に state 項目と emcy_socket |
| `ros2_ws_main/src/bringup/rerobot_bringup/config/params.yaml` | `link_loss_timeout_s` 削除、`monitor_period_s` 0.05、新パラメータ (コメント付き) |
| `ros2_ws_main/src/bringup/rerobot_bringup/scripts/waypoint_runner.py` | `/epos4_controller/state` 購読、DISARMED / WAIT_POWER で一時停止、走行許可が出るまで再開しない、許可がない間は自動再送しない |
| `ros2_ws_main/src/bringup/rerobot_bringup/scripts/epos_state_log.py` + `launch/auto_run.launch.py` + `CMakeLists.txt` | auto_run が状態ログのノードを 0 s で起動: `/epos4_controller/state` の変化と `/diagnostics` の Fault 詳細 (コード・履歴・Vcc) を `[EPOS 時刻]` の 1 行で画面 (tee のログ) と `/workspace/log/epos_state_<日時>_<tag>.log` (bag と同名、`epos_log_dir` で変更可) に書く。bag の記録トピックに `/epos4_controller/state` を追加。controller は Fault 分類直後に `/diagnostics` を即 1 回出す (0.3 s で終わる自動復帰も詳細が残る)。擬似 EPOS 試験と同時に動かして確認済み |
| `ros2_ws_main/src/app/epos4_controller/test/mock_recovery_test.py` | 擬似 EPOS ×2 + vcan0 の EMCY での結合試験 (下記) |

試験 (モータ不要、rerobot_env 内、実機スタックと混ざらないよう別ドメイン):
```bash
ip link add dev vcan0 type vcan; ip link set up vcan0
cd /workspace && source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash
ROS_DOMAIN_ID=177 python3 src/app/epos4_controller/test/mock_recovery_test.py
```
11-06 夜の結果 25/25 PASS: 起動 DISARMED・Nav2 遮断・LB joy 通過・パッド瞬断で 0・再接続直後の Start 無視・Start で RUNNING・片輪/両輪 0x3220 (24 V) 自動復帰・0x8130 自動復帰・非常停止 (8 V) → WAIT_POWER → 電源復帰で DISARMED (走らない) → Start・予算超過で DISARMED・0x8250 自動復帰と再発で DISARMED・EMCY なしを物理トリガで検出・EMCY なしの 0x3220 は非常停止扱い。擬似 EPOS では recover が 0.2〜0.3 s で終わる (実機はもっと遅い)。

未試験: executor 停止監視 (T7c)、waypoint_runner の一時停止と再開 (Nav2 が要る)、実機の EMCY・Vcc の時間関係 (T2/T3)、CANUSB 抜去 (T6)。次は §7 の T1〜T7 を台上 (車輪浮かせ) で。
