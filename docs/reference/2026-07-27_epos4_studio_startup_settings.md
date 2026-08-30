<!-- claude: docs/reference — リポジトリ外に存在する設定 (EPOS4 NVM 等) の記録。実機設定を変えたら必ず更新すること。-->
# EPOS4 Studio Startup Wizard 設定記録 (Node 1)

- **出典**: `~/Pictures/epos4 setting/` のスクリーンショット 13 枚 (撮影 2026-07-27 21:37–21:39)
- **対象**: EPOS4 CAN [Node 1] = `/motor1/cia402_device_1` = **右輪** (物理配線は motor1 = 右)
- **記録日**: 2026-07-29
- ✅ **2026-07-29 SDO read で裏取り済み**: 両ノードとも 0x3010:01 = **256**、0x3000:05 = **1024** を実機 NVM から確認。Node 2 もエンコーダ設定は Node 1 と同一 (下記 2 つの旧懸念は解消。他項目の Node 2 同一性は未確認のまま)。
- ~~⚠️ Node 2 (左輪) の設定は未確認~~ → エンコーダ分解能については解消。
- ~~⚠️ スクショはウィザード画面のため NVM 書込は判別不能~~ → SDO read で書込済みを確認:
  ```bash
  ros2 service call /motor1/cia402_device_1/sdo_read canopen_interfaces/srv/CORead "{index: 0x3010, subindex: 1}"
  ```
  (※ NVM = 電源を切っても消えない EPOS4 内蔵の設定保存領域)

## 1. 設定値一覧

### Motor (Drive System)

| 項目 | 設定値 | 備考 |
|------|--------|------|
| Motor type | maxon DC motor | ブラシ付き DC |
| Nominal current | 4260 mA | Max continuous current に自動反映 |
| Torque constant | 31.100 mNm/A | 「Identify during mechanical system identification」に✓ → チューニング時に同定値で上書きされる |
| Thermal time constant winding | 43.0 s | |
| Max speed | 7720 rpm | モータ軸基準 |

### Gear (Drive System)

| 項目 | 設定値 | 備考 |
|------|--------|------|
| System with gear | **OFF (チェックなし)** | 物理には 5:1 減速機あり。ギヤ比は ROS 側 (`gear_ratio`) で扱う設計なので EPOS 側未設定は運用と整合。ただし EPOS 内の速度・加速度の単位/制限はすべて**モータ軸基準**になる点に注意 |
| (Absolute reduction) | 1 : 5 (グレーアウト) | 無効状態の表示値。奇しくも実ギヤ比と同じ |

### Sensors (Drive System)

| 項目 | 設定値 | 備考 |
|------|--------|------|
| X4 - Hall sensor | None | |
| X5 - Encoder | Digital incremental encoder 1 / On motor shaft | |
| Number of pulses | **256 pulses/revolution** | ⭐ 最重要値。4 逓倍後 **1024 counts/rev** — issue 2026-07-07 の実測逆算 (1024 counts/rev) と一致 |
| Type | Encoder with index (3-channel) | |
| Direction | **Inverted** | モータ極性との整合は取れている (現に正常動作) |
| X6 - Sensor | None | |

### Regulation (Controller)

| 項目 | 設定値 |
|------|--------|
| Current | PI current controller |
| Velocity | PI velocity controller (low-pass filter) |
| Position | PID position controller |
| Main sensor | X5 - Digital incremental encoder 1 |

### Units (Controller)

| 項目 | 設定値 | bus.yml との整合 |
|------|--------|------------------|
| Position Unit | 1 inc | `scale_pos_from_dev = 2π/4096` は **4096 inc/rev 前提 → 実機 1024 inc/rev と 4 倍不整合** (issue 2026-07-07 の根本原因) |
| Velocity Unit | 1 rpm | `scale_vel_from_dev = 2π/60` (rpm→rad/s) と整合 ✓ |
| Acceleration Unit | 1 rpm/s | |
| Current Unit | 1 mA | |
| Torque Unit | 0.001 mNm | |

### Limits (Controller)

| 項目 | 設定値 | 備考 |
|------|--------|------|
| Max continuous current | 4260 mA (自動) | Nominal current 由来。EPOS4 Compact 50/5 の連続 5 A 以内 ✓ |
| Max output current | ~~15000~~ → **10000 mA** | 2026-08-30 ハードニングで引き下げ (電流スパイク事故対策、docs/issue/2026-08-11 参照)。SDO 0x3001:02 で両ノード裏取り済み |
| Max acceleration | ~~4294967295~~ → **30000 rpm/s** | 2026-08-30 有限化 (§2-3 の対応完了)。新チャシス (gear 92.25, tire 0.25 m) で車体 ≈ 4.3 m/s²。ソフトリミッタ 15000 rpm/s の 2 倍 = 安全網役。SDO 0x60C5 で両ノード裏取り済み |
| Max profile velocity | 7720 rpm | 2026-08-30 車体改修 (ギヤ 92.25:1, タイヤ φ0.25 m) 後は車体換算 ≈ **1.10 m/s** = 物理上限そのもの。旧チャシス時代の「実質ノーガード」状態は解消 |
| Following error window | 2000 inc | 速度制御運用では実質未使用 |
| Software position limit | OFF | 移動ロボットなので妥当 |
| Max temperature power stage | 105.0 °C | デフォルト |
| Power supply undervoltage limit | 10.000 V | 供給バッテリ電圧との整合を要確認 → §3 |
| Power supply overvoltage limit | 48.000 V | 同上 (回生時の跳ね上がり考慮) |

### Device Control (Controller)

| 項目 | 設定値 |
|------|--------|
| Shutdown option code | Disable drive function |
| Disable operation option code | Slow down on deceleration |
| Quick stop option code | Slow down on quick stop deceleration → quick stop active |
| Fault reaction option code | Slow down on quick stop deceleration |
| Abort connection option code | Quick stop command |
| Profile deceleration | ~~10000~~ → **30000 rpm/s** (2026-08-30、0x6084。新チャシスで車体 ≈ 4.2 m/s²) |
| Quick stop deceleration | ~~10000~~ → **30000 rpm/s** (2026-08-30、0x6085。同上) |

### Windows (Controller)

| 項目 | 設定値 | 備考 |
|------|--------|------|
| Use standstill window | ON | |
| Standstill window | **429496729 rpm** | 実質無制限 (デフォルト) → 常に「停止中」と判定される → §2-4 |
| Standstill window time | 2 ms | |
| Standstill window timeout | ON / 1000 ms | |
| Use position window | OFF | |

### Touch Probe (Controller)

Switched Off / First Trigger Only / Digital Input / No Edge。未使用 (画面の Info「TouchProbe 用の digital input 未定義」は未使用なら無害)。

### Inputs / Outputs

| 端子 | 機能 | 極性 |
|------|------|------|
| Digital input 1 | **Negative limit switch** | High active |
| Digital input 2 | **Positive limit switch** | High active |
| Digital input 3 | **Home switch** | High active |
| Digital input 4 | General purpose A | High active |
| High-speed digital input 1–4 | None | High active |
| Digital output 1 / 2 | General purpose A / B | High active |
| High-speed digital output 1 | None | High active |
| Analog input 1 / 2 | General purpose A / B | — |
| Analog output 1 / 2 | General purpose A / B | — |

## 2. 判定 — おかしなところ / 要対応

### 2-1. ⭐ エンコーダ 256 ppr が確認できた (issue 2026-07-07 の未検証事項 1 を解消)

スクショの 0x3010:01 相当値 = **256 pulses/rev** は、実測から逆算したエンコーダ分解能
(1024 counts/rev = 256 パルス × 4 逓倍) と一致する。つまり **EPOS4 側の分解能設定は正しく、
4 倍ズレの根本原因は `bus.yml` の `scale_pos_from_dev = 2π/4096` (4096 counts/rev 前提) 側で確定**に近づいた。

**副作用として issue §3 の「速度 4 倍相殺」仮説は否定される**: EPOS4 が正しく 256 ppr を知っているなら
速度ループの誤認は無く、`gear_ratio 1.25` で計算した速度指令はモータ必要回転数の **1/4 のまま届いているはず**。
→ **2026-07-29 実測で確定**: candump で 0x60FF 生値 = controller の rpm そのまま (raw tpdo 無スケール)、
走行中 0x606C ≈ 指令 rpm に追従。相殺要因は無く、**実車は指令速度の 1/4 で走行している**
(詳細は issue の「実測記録 2026-07-29」)。

また、このスクショが「既存値の確認」なのか「2026-07-27 に値を修正した」のかで意味が変わる。
もし 7/27 に 1024→256 へ直したのなら、それ以前と以後で速度応答が変わっているはず。要記憶確認。

### 2-2. ⚠️ リミットスイッチ※2 割当が残っている (Digital input 1–3)

DI1=Negative limit switch, DI2=Positive limit switch, DI3=Home switch (すべて High active) は
EPOS4 の工場デフォルト割当のまま。本機は配線されていないため通常は Low=非アクティブで実害が出ていないが、
ノイズや断線気味の配線が High に浮くと**該当方向の駆動が突然ブロック (quick stop)** され、
原因究明が非常に困難な「時々止まる」症状になる。未使用なら **3 つとも None に変更推奨**。

### 2-3. ~~⚠️~~ ✅ Max acceleration が無制限 (4294967295 rpm/s) → 2026-08-30 に 30000 rpm/s へ有限化済み

CSV モードでは `epos4_controller` が 100 Hz で階段状の速度目標を送るため、ドライバ側の加速度制限が
唯一のランプ整形要素になる。無制限のままだと指令ジャンプがそのままトルク衝撃になり、
機体の揺れ・ハンチング評価のノイズ・バッテリ電流スパイクの一因になり得る。
有限値 (例: まず 20000–50000 rpm/s 程度から実験) の設定を検討。

### 2-4. ⚠️ Standstill window が無意味な巨大値 (429496729 rpm)

「モータが停止した」と判定する速度幅が実質無制限 = **常に停止扱い**。
Disable operation の「Slow down on deceleration」が即座に完了扱いになり、
減速ランプを経ずに脱力する動作になり得る (free mode `f` の切替感触に関係)。
意図した減速停止が欲しいなら現実的な値 (例: 100 rpm / 数十 ms) に設定する。

### 2-5. ℹ️ 確認事項 (設定ミスではないが記録)

- **電源電圧しきい値 10–48 V**: バッテリ公称電圧・回生時ピークがこの窓に収まるか要確認 (Compact 50/5 の絶対最大は 50 V)。
- **モータカタログ照合**: Nominal current 4260 mA / max speed 7720 rpm / 熱時定数 43 s / kT 31.1 mNm/A がモータ型番のカタログ値と一致するか未照合 (スクショに型番情報なし)。
- **Node 2 が同一設定か** (特に 256 ppr と Direction Inverted — 左右の回転方向差は ROS 側 `invert_left/right` で処理するので EPOS 側は左右同一のはず)。

## 3. 関連

- `docs/issue/2026-07-07_wheel_odometry_encoder_scaling_4x.md` — 4 倍ズレ問題 (本記録が未検証事項 1 を解消)
- `ros2_ws_main/src/drivers/epos4compact50-5can/maxon_epos4_ros2/config/bus_config_cia402_epos4_vel/bus.yml` — スケーリング係数の所在
- `ros2_ws_main/src/bringup/rerobot_bringup/config/params_2d.yaml` — `gear_ratio: 1.25` (対症療法)
