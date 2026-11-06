<!-- claude: docs/features — 設計文書。2026-11-05 作成。ステータス: **計画 (未実装)**。実装後に「使い方」「変更ファイル」を実態に合わせて更新すること。 -->
# EPOS 自動復帰 — 通信停止 (0x8250) と段差ブラウンアウト (0x3220) は自動で走行再開、非常停止後は人が Start するまで走らない

- ステータス: **計画・未実装** (2026-11-05)。ユーザ要求: 「EPOS 謎停止と、段差乗り上げで DC-DC が供給できなかった電圧低下停止は自動復帰したい。ただし非常停止後の復帰で勝手に走り出さないこと」
- 根拠にした事象: 11-04 13:45 の 0x8250 (`docs/issue/2026-10-22_epos_fault_8250_pdo_freeze_lidar3d_run.md` §11)、11-04 の段差停止 (ユーザ報告、bag 上は未同定)、09-19 の非常停止・段差試験 (`docs/issue/2026-09-19_epos_stop_link_loss_rpdo_timeout_power.md` §10.1/§10.7)
- 関連: 現行の監視・ウォッチドッグ `docs/features/2026-09-19_epos4_controller_watchdog_fault_monitor.md`、脱力モード `docs/features/2026-06-05_motor_free_mode.md`

## 1. 目的・概要

```
停止の種類                         EPOS のコード      望む振る舞い
├─ 通信停止 (マスタ側 PDO 途絶)     0x8250 両輪同時    自動 recover → そのまま走行再開
├─ マスタ heartbeat 途絶 (11-06 追加) 0x8130 両輪同時   (案) 通信停止と同じ扱い。ただし USB 喪失型は can0 再作成 = bringup 再起動が要り、recover では戻らない
├─ 段差ブラウンアウト (RSD 折り返し) 0x3220 片輪ずつ    自動 recover → そのまま走行再開
├─ 非常停止 (人が押した)            0x3220 両輪同時    Vcc が戻ったら motor は enable するが **走らない** → Start で再開
└─ それ以外 (過電流・追従誤差・熱)   0x2310/0x8611/…   自動復帰しない (止まったまま報告)
```

鍵は **「復帰 (motor を enable する)」と「走行再開 (指令を通す)」を分ける**こと。今は指令が EPOS に直通なので、recover した瞬間に Nav2 の残っているゴール (後述: ゴールは 3600 s 死なない) で走り出す。

## 2. 前提として今日確定・判明したこと

### 2.1 現行監視の情報源 `~/rpdo` は **起動時間の 1/3 だけ遅れて届く** (有力・critic 未査読)

11-04 bag: 13:45:42 (bag 1063 s) の Fault が、監視の `/diagnostics` には **13:56:05 (bag 1685 s)** に初めて 0x0208 として現れ、その時点の SDO live 読みは「エラー 0・24.2 V・車輪は 0.8 m/s で回転中」。つまり statusword の値が 622 s 古い。プロセス起動 (13:24:40) からの経過で見ると 1262 s の事象が 1884 s に届いた = **比 1.49 ≈ 3/2**。

機構 (ros2_canopen jazzy の `node_canopen_base_driver_impl.hpp`): ドライバは `polling: true` (既定) で `period` (bus.yml defaults = **50 ms**) ごとに rpdo キューから **最大 10 件**しか取り出さない = 200 件/s。一方 TPDO1 は SYNC 10 ms (bus.yml `sync_period: 10000` us) ごとに 3 オブジェクト (0x6041/0x6064/0x6061) = **300 件/s** 生成。差分 100 件/s がキューに溜まり、遅れが起動時間の 1/3 で増える (予測 1.5 倍と実測 1.49 倍が一致)。

帰結:
- 09-19 §10.5 で「SDO ポーリング → `~/rpdo` 購読」に作り直した監視は、**起動数分後から Fault 検知にも link 生存判定にも使えない** (10-22 §2.3 の見逃しもこれで説明できる — 「ドライバがキャッシュを周期 publish」ではなくキューの遅延)。
- 直し方は 2 択: bus.yml `period: 10` (取り出し 1000 件/s) か `polling: false` (SYNC ごとに取り出し)。いずれも submodule `epos4compact50-5can` 側の変更 → 親で gitlink 更新。**この計画の検出は SDO ポーリングを主にして、rpdo の修正に依存しない**。

### 2.2 Nav2 のゴールは停止中も死なない

`nav2_params.yaml` controller_server の `movement_time_allowance: 3600` (10-15 に 10 → 3600) なので、EPOS が止まっても Nav2 は同じゴールに 0.4 m/s を出し続ける (11-04 も 13:45〜13:50 の 4 分間出し続けた)。→ recover した瞬間に走る。自動復帰させたい 2 種にはこれが都合よく、非常停止には危険。**非常停止系はゴールを runner 側で cancel する + controller 側で指令を 0 に固定する、の 2 重**にする。

### 2.3 3 種の停止は EPOS から見て区別できる (09-19 の実測)

| 種類 | コード | 両輪同時? | 検知時 (≤0.5 s 後) の 0x2200:01 (Vcc) | その後の Vcc |
|---|---|---|---|---|
| 通信停止 | 0x8250 | 同時 | 24 V (正常) | 正常 |
| 段差ブラウンアウト | 0x3220 | **片輪ずつ** (09-19 §10.7: 26 s 差) | **24 V に即復帰** (§10.7 読み戻し) | 正常 |
| 非常停止 | 0x3220 | 同時 | **8 V 前後** (§10.1: RSD 出力コンデンサ放電中) → 0 V | 人が解除するまで低い |

→ 分類規則: `0x8250` = 通信 / `0x3220 ∧ Vcc ≥ 20 V` = ブラウンアウト / `0x3220 ∧ Vcc < 15 V` = 電源断 (非常停止)。両輪同時かどうかは補助 (単独では使わない: ブラウンアウトが両系統同時に起きる可能性を排除できない)。

### 2.4 `recover` サービス 1 発で CSV Operation Enabled に戻る

11-04 13:50: Fault 中に `/motorN/cia402_device_N/recover` → statusword 0x1237、mode 9 (CSV) 維持。enable / cyclic_velocity_mode の再呼び出しは不要 (脱力モード復帰とは違う経路)。

### 2.5 11-06 台上で判明した通信途絶時の EPOS の挙動 (実測、heartbeat 監視を追加済み)

詳細 `docs/issue/2026-10-22_epos_fault_8250_pdo_freeze_lidar3d_run.md` §12.1〜12.3。**この計画の前提のうち「通信停止 = 0x8250」は崩れた** ので、実装前に読むこと。

```
11-06 の台上結果 (車輪浮かせ・目標≠0・candump で確認)
├─ heartbeat 対策の前 (旧 bus.yml)
│   ├─ マスタ全停止 20 ms〜10 s / SYNC のみ 4 s / RPDO の重複・圧縮 / NMT Pre-Op → Fault 0 (67 回)
│   ├─ 回転中に CANUSB を手で抜く → Fault せず最後の速度で 58 s 回り続けた
│   └─ ⇒ 0x8250 は「PDO 途絶」では出ない。実走の 0x8250 の原因は未解明のまま (台上に無い要素が要る)
└─ heartbeat 対策の後 (bus.yml: master heartbeat 100 ms × 5、EPOS 0x1016 = node 6 / 500 ms、0x1029:01 = 1)
    ├─ マスタ停止 ≤400 ms → 何も起きない (回転継続)
    ├─ マスタ停止 ≥600 ms → 停止から 0.43〜0.50 s で EMCY 0x8130 → quick stop (約 1 s で 0 rpm)
    │   statusword 0x0208 (NMT Operational のまま)、0x603F = 0x1003:01 = 0x8130 → recover 1 発で 0x1237
    └─ 回転中に CANUSB を抜く → 即停止・LED 赤、0x8130。can0 が作り直されるので recover では戻らず bringup 再起動が要る
```

この計画への影響:
- **分類 (§1・§2.3) に 0x8130 (heartbeat error) を加える**。通信途絶系の主コードは今後 0x8130。0x8250 は「原因未解明の別物」として扱う (自動 recover の対象にするかは要判断)。0x8130 は「マスタの一時停止」と「CANUSB 喪失」の 2 型があり、見分け方は can0 の ifindex (現行 link_lost 監視) — 喪失型は recover でなく再 launch 系。
- **0x1029:01 = 1 が前提** (bus.yml defaults `error_behavior: {1: 0x01}`)。既定 0 だと heartbeat 途絶で EPOS が **NMT Pre-Operational に落ち (heartbeat 0x7F)、PDO が止まって recover (PDO の fault reset) が届かない**。その状態からの手動復帰は `cansend can0 000#0100` (NMT Start 全ノード) → recover。
- statusword の見え方: 0x1029:01 = 1 なら Fault は **0x0208** (bit 9 remote = Operational)、0x1029 既定だと **0x0008**。
- **0x8130 の後に指令経路が生きていれば Nav2 の残ゴールで即再発進する** のは 0x8250 と同じ (§2.2)。
- **誤停止のリスク**: マスタの 0.6 s 級の一時停止 (10-11 台上で稀に観測) は今後 0x8130 で止まる。自動復帰の「予算」(§3.1 の 3 回/120 s) は、これが実走でどれくらい出るかを見てから決める。
- §3.2 の「物理トリガ (指令あり・車輪不動)」は、heartbeat 型では EPOS 側が先に止めるので補助になる。逆に heartbeat が届くのに PDO だけ届かない型 (11-06 12:07 の 0x8250 の可能性) は EPOS が止めないので、物理トリガが唯一の検出手段として残る。
- 台上の再現手順 (自動復帰の検証にそのまま使える): `docker exec rerobot_env bash -c 'kill -STOP <device_container PID>; sleep 1; kill -CONT <PID>'` で 0x8130 を 1 発作れる (`bags/exp/2026-11-06_epos_8250/bench/stop_test.sh`、bags/ は git 管理外なのでローカルのみ)。

## 3. 設計の勘所

### 3.1 状態機械 (epos4_controller の監視スレッドに置く)

```
RUNNING ──(Fault 検知)──► CLASSIFY ─┬─ TRANSIENT (0x8250 / 0x3220∧Vcc≥20V) ──► RECOVERING ──(両輪 ready)──► RUNNING  (指令 0 からランプ)
   ▲                                │                                          │ 失敗 or 予算超過 (例 3 回/120 s)
   │                                ├─ POWER_CUT (0x3220∧Vcc<15V) ──► WAIT_POWER ──(Vcc≥20V が 1 s 続く)──► recover ──► DISARMED
   │                                └─ OTHER (0x2310/0x8611/熱/不明) ─────────────────────────────────────────────► DISARMED
   │                                                                                                                  │
   └──────────────────────────────────── ARM (Start ボタン / ~/arm サービス) ◄────────────────────────────────────────┘
                                                                                     DISARMED 中: 目標 0 固定 (例外は §3.3 の LB)
```

- **RECOVERING** では `force_zero` を立て、ランプ出力 `m*_cmd_` を 0 にリセットしてから recover → `motor_ready` (SDO 0x6041/0x6061) で確認 → 解除。Nav2 のゴールは生きているので指令は自動で戻り、ランプ (15000 rpm/s) で滑らかに再発進する。
- **予算**: TRANSIENT の自動復帰は `auto_recover_max` 回 / `auto_recover_window_s` (案 3 回 / 120 s)。超えたら DISARMED (通信が本当に死んでいる・段差で何度も落ちる = 人が見るべき)。
- **長期化**: どの分類でも Fault 検知から `long_fault_s` (案 5 s) 以内に RUNNING に戻れなければ DISARMED (周囲に人が集まっている想定)。
- **POWER_CUT** は Vcc を 1 Hz で SDO 読みし、`vcc_ok_v` 以上が 1 s 続いたら recover して **DISARMED** に入る (motor は enable 済み = ブレーキ保持・押せない状態。押して動かしたい場面は既存の脱力 f キー)。
- **OTHER** は安全側で自動復帰しない。手動は従来どおり (recover サービス / 再起動)。
- 現行の `link_lost` (can0 消滅・ifindex 変化) はそのまま (再 launch が必要な型で自動復帰の対象外)。

### 3.2 検出は SDO ポーリングを主にする (`~/rpdo` は §2.1 の遅延があるので補助)

```
検出 (監視スレッド, 0.5 s 周期)
├─ (1) SDO 0x6041 を両輪 2 Hz で読む (4 SDO/s。09-19 §10.5 で嫌った driver の "SDO Read Call" INFO は
│      driver ノードの logger level を WARN にして消す)。Fault bit (bit 3) 立ち上がりで 0x603F / 0x1003:01〜05 / 0x2200:01 を読む (現行どおり)
├─ (2) 物理トリガ: |指令| > 0.05 m/s なのに両輪の joint_states 位置が 0.5 s 変化しない → 即 SDO 読み (ポーリング周期を待たない)
│      (joint_states は driver がオブジェクト辞書を直接読むので遅延しない — 11-04 bag で凍結時刻が正確だった)
├─ (3) リンク喪失: 両輪の SDO が 2 回連続タイムアウト → 従来の link_lost 扱い (目標 0)
└─ (4) `~/rpdo` は bus.yml の period/polling 修正後に副次情報として復活させる (修正前は値を信用しない)
```

### 3.3 「走らない」の保証は controller 側の DISARMED ラッチ、runner の cancel は 2 重目

- controller は DISARMED 中、`/robot_speed_cmd` の値によらず目標 0 を出す (twist_mux の上流が何であれ効く = RViz 単発ゴールや手動 run でも保証)。
- **例外 (推奨・要判断)**: `/joy` の LB (deadman, `enable_button`) が押されている間だけ通す。人がパッドを握って動かすのは「勝手に」ではないので、非常停止解除後に手で退避させられる。LB を離せば twist_mux は Nav2 に落ちるが DISARMED なので 0 のまま。
- ARM = `/joy` の Start (`start_button`, joy_teleop.yaml の waypoint_runner 節と同じ番号) の立ち上がり、または `~/arm` (std_srvs/Trigger)。controller 自身が `/joy` を購読する (runner が居ない手動運用でも効く)。
- controller は状態を `/epos4_controller/state` (std_msgs/String: `RUNNING` / `RECOVERING 0x8250 2/3` / `WAIT_POWER 8.1V` / `DISARMED 非常停止 — Start で再開`) に 2 Hz で出す。
- waypoint_runner はこれを購読: `WAIT_POWER` / `DISARMED` を見たら走行中のゴールを cancel して「一時停止」と同じ状態 (`paused`) に入り、RViz の車体上テキストに理由を出す。Start で controller が ARM、runner が同じ Start で区間を再送 (既存の再開経路)。`RECOVERING` では何もしない (ゴール継続)。

### 3.4 自動復帰で再発進するときの安全

- 指令は必ず 0 からランプ (既存 `ramp_toward`、`m*_cmd_` を 0 リセット)。
- RPP の衝突判定・costmap は停止中も回っているので、前に物があれば Nav2 側が 0 を出す。
- 0x8250 の原因 (lely スレッド停止、10-22 §11.1) は未解決のままなので、**自動復帰は症状の緩和であって原因対策ではない**。candump 並走 (同 §11.1) は並行して入れる。

## 4. データフロー

```
/joy ──────────────┬─► teleop_twist_joy ─► /joy_vel ─┐
                   │                                  ├─► twist_mux ─► /robot_speed_cmd ─► epos4_controller ─► tpdo ─► EPOS
Nav2 ─► /cmd_vel ──┼──────────────────────────────────┘                                       ▲   │
                   │ (Start/LB)                                                               │   │ SDO 0x6041 (2 Hz) / 0x603F / 0x2200:01 / recover
                   └─────────────────────────────────────────────────────────────────────────►│   ▼
waypoint_runner ◄── /epos4_controller/state ◄───────────────────────────────────────────── 状態機械 (監視スレッド)
   │ cancel_goal (WAIT_POWER / DISARMED)      joint_states (物理トリガ) ──────────────────────┘
   └─ RViz マーカー「⛔ 非常停止 — Start で再開」
```

## 5. 使い方 (実装後の運用。オペレータ視点で変わる点だけ)

- 通信停止・段差停止: **何もしない**。数秒で勝手に再発進する (RViz テキストに `RECOVERING` が一瞬出る)。3 回/2 分を超えたら止まって `DISARMED` → 原因を見る。
- 非常停止: 押す → 解除 → EPOS 緑 (enable 済み) だが動かない → 周囲確認 → **Start** → 区間の続きから再開。手で退避したいときは LB を握って joy。
- 再確認コマンド: `ros2 topic echo /epos4_controller/state`、手動 ARM: `ros2 service call /epos4_controller/arm std_srvs/srv/Trigger`

## 6. 変更ファイル (予定)

| ファイル | 変更 |
|---|---|
| `ros2_ws_main/src/app/epos4_controller/src/epos4_controller.cpp` | 監視ループを状態機械に置換、SDO 2 Hz ポーリング + 物理トリガ、recover 経路、`/joy` 購読 (Start/LB)、`~/arm` サービス、`~/state` publisher |
| `ros2_ws_main/src/bringup/rerobot_bringup/config/params.yaml` | 新パラメータ: `fault_poll_period_s 0.5` / `vcc_ok_v 20.0` / `vcc_cut_v 15.0` / `auto_recover_max 3` / `auto_recover_window_s 120` / `long_fault_s 5` / `arm_button 7` / `enable_button 4` / `allow_joy_when_disarmed true` |
| `ros2_ws_main/src/bringup/rerobot_bringup/scripts/waypoint_runner.py` | `/epos4_controller/state` 購読 → cancel + paused、RViz テキストに理由 |
| `ros2_ws_main/src/bringup/rerobot_bringup/launch/rerobot_bringup.launch.py` | driver ノードの logger level (SDO INFO 抑制) |
| `ros2_ws_main/src/drivers/epos4compact50-5can/.../bus.yml` (submodule) | `period: 10` または `polling: false` (§2.1 の遅延修正、別コミット・gitlink 更新) |
| `docs/issue/2026-10-22_…md` §11 / 本文書 / PROJECT_STATE | 記録 |

## 7. 検証計画 (台上・車輪浮かせ → 接地)

| # | 試験 | 期待 |
|---|---|---|
| T1 | Nav2 ゴール中に `kill -STOP <device_container pid>; sleep 1.0; kill -CONT` (0x8250 を人工再現、10-11 の手法) | ≤2 s で `RECOVERING 0x8250 1/3` → `RUNNING`、車輪が 0 からランプして再回転 |
| T2 | Nav2 ゴール中に非常停止を押す → 5 s 後に解除 | ≤1 s で `WAIT_POWER` (Vcc < 15 V)、runner がゴール cancel・テキスト表示。解除後 motor enable (緑) だが **回らない**。Start で再開 |
| T3 | 非常停止を 0.3 s だけ叩く | T2 と同じ分類になること (検知時 Vcc がコンデンサ放電で < 15 V)。TRANSIENT に化けたら「両輪同時 0x3220 は POWER_CUT」の規則を追加 |
| T4 | T1 を 60 s に 4 回 | 4 回目で `DISARMED 予算超過` |
| T5 | DISARMED 中に LB + スティック / LB なしスティック | 前者は動く (allow_joy_when_disarmed)、後者は動かない |
| T6 | 段差ブラウンアウトは台上で再現不能 → 実走で 0.3 m/s の段差乗り上げ (09-19 §10.7 と同条件)、bag + candump で分類を確認 | `RECOVERING 0x3220` → 再発進 |
| T7 | bus.yml 修正後: `ros2 topic echo /motor1/cia402_device_1/rpdo` の statusword 変化が candump の TPDO 変化から ≤0.1 s | 遅延解消 |

## 8. 決めてほしい点 (推奨を先頭に)

1. DISARMED 中の LB (joy) を通すか — **推奨: 通す** (非常停止後に手で退避できる。「勝手に」ではない)。
2. 自動復帰の予算 — **推奨: 3 回 / 120 s**。段差で 3 回連続は人が見るべき。
3. `long_fault_s` — **推奨: 5 s**。通信停止の recover は ≤2 s で終わる。
4. bus.yml の遅延修正は `period: 10` か `polling: false` か — **推奨: `polling: false`** (SYNC 同期で無駄なタイマなし)。ただし driver の poll_timer が消えるので台上で joint_states 周期を確認。

## 9. 既知の制限

- 0x8250 の根本原因 (lely マスタ停止) は未解決。自動復帰は緩和。
- CANUSB の内部リセット・USB 再列挙 (09-19 型) は自動復帰しない (master が旧 socket を掴む。再 launch が必要)。
- 非常停止の判定は Vcc 閾値に依存。電源構成 (RSD-60L-24 の出力コンデンサ) を変えたら T2/T3 を再試験。
- `~/rpdo` 遅延 (§2.1) は critic 未査読。実装前に T7 の前半 (修正前の遅延測定) で確定させる。
