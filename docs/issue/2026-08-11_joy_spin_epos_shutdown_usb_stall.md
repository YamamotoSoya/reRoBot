<!-- claude: docs/issue — 未解決問題の調査記録。解決したらステータスを更新すること。-->
# joy 操作中の急回転で EPOS 停止 + CANUSB が USB ストールする問題 (3 回目)

- **ステータス: 未解決** (復旧手順は確立済み。根本対策 = EPOS 側ハードニングが未適用、原因フォルトコードも未確認)
- 日付: 2026-08-11 (初発 2026-07-31 ×2 → 本件で 3 回目)
- 環境: Docker `rerobot_env` / ROS 2 Jazzy / ros2_canopen / EPOS4 ×2 over can0 (LAWICEL CANUSB, slcand)
- 関連ファイル:
  - `scripts/can_up.sh` (stale-slcand 検出を 08-11 に追加済み)
  - `ros2_ws_main/src/app/epos4_controller/src/epos4_controller.cpp` (slew rate limiter, 07-31 対策)
  - `ros2_ws_main/src/bringup/rerobot_bringup/config/params.yaml` (`max_motor_accel/decel_rpm_per_s: 2000`)
  - `/etc/udev/rules.d/99-hokuyo-devices.rules` + `canusb-up.service` (ホスト側 CAN 自動起動)

## 症状 (再現 3 回、共通シグネチャ)

激しい速度変化 (今回: joy 操作中の意図しない急回転) の瞬間に:
1. EPOS が停止 (赤ランプ / PDO 送信不能)
2. **CANUSB アダプタが USB ごとストール** — kernel log に `ftdi_sio ttyUSBx: urb stopped: -32` 連発
3. モータ電流/回生スパイク → 電源・USB 巻き添えという 07-31 診断と同型

## 今回 (08-11) の実測ログ

- 16:50:14 `ftdi_sio ttyUSB0: urb stopped: -32` ×3 (急回転の瞬間、CANUSB=1-2.3)
- 16:53:24 ハブ 1-2.4 に「同一シリアル LW3RNRBM の CANUSB」が出現 — 物理は 1 台のみ (ユーザ確認)
  のため**幽霊列挙の疑い** (ioctl 完全無応答の wedged デバイス。実害は tty 番号占有のみ、PC 再起動で消える見込み)
- 17:09:51 `xHC error in resume / root hub lost power` (ノート PC のサスペンド復帰) → 全デバイスリセット。
  実体 CANUSB (1-2.3) は ttyUSB2 として復活、`/dev/ttyCANUSB` も追従したが **can0 は DOWN** (`slcan off ttyUSB0`)

## 確立した復旧手順

1. `./scripts/can_up.sh` — can0 DOWN なら service 再起動、**UP に見えても slcand が旧 tty を掴む「見かけ上 UP」を検出して再接続** (08-11 追加。slcand の実 fd と /dev/ttyCANUSB の実体を照合)
2. EPOS 電源再投入 (フォルトラッチのクリア) → bringup 再起動
3. 疎通確認: `docker exec rerobot_env bash -c 'candump can0 -n 5 -T 3000'` でハートビート確認

## 未解決事項 (根本対策)

- [ ] ~~**EPOS Error History の確認**~~ → **未確認のまま 2026-08-30 の設定作業を通過 (Clear された可能性あり) — 8/11 事故の履歴は失われた見込み**。原因確定 (0x3210 過電圧 / 0x2310 過電流 / 0x81FD bus-off) は次回再発時に**最優先で** Error History を読むこと
- [x] **EPOS 側ハードニング適用済み (2026-08-30、EPOS Studio)**: Max acceleration 0x60C5 / Quick stop decel 0x6085 / Profile decel 0x6084 = **30000 rpm/s**、Max output current = **10 A** (連続 4260 mA は変更なし)。両ノードとも SDO 直読みで裏取り済み。値は同日の車体改修 (gear 92.25 / tire 0.256 m — 車体 1 m/s² ≈ 6882 rpm/s、30000 ≈ 4.4 m/s²) 基準で、ソフトリミッタ 15000 rpm/s の 2 倍 = 安全網役。⚠️ NVM 永続 (Save All) の効きは**次回電源再投入後の SDO 再読みで要確認**
- [ ] ソフト側の追加候補: joy turbo スケール低減 / `max_motor_decel_rpm_per_s` 低減 (2026-08-30 車体改修で 2000 → 15000 に再換算済み — 車体加速度換算では 3.1 → 2.1 m/s² と実質強化。再発したらここをさらに絞る)
- [ ] **ロボット運用中のノート PC サスペンド無効化** — 17:09 の xHC リセットは走行中なら can0 即死。電源設定で対策する

## 教訓

- can0 が `ip link` で UP でも slcand が死んだ fd を掴んでいることがある — 「UP = 正常」と判定しない
- USB ストール事故の後は同一デバイスの幽霊列挙が起こり得る — `/sys/bus/usb/devices/` の実体と tty の ioctl 応答で切り分ける
