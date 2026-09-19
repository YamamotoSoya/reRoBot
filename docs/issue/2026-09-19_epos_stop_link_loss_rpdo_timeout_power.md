<!-- claude: docs/issue — 未解決問題の調査記録。解決したらステータスを更新すること。2026-09-19 作成。 -->
# 新電源構成 (harness v2) での EPOS 停止・暴走 — 初のフォルトコード取得 (0x8250 RPDO timeout / 0x3220 低電圧)

- **ステータス: 調査中・critic 査読済み (09-19 ×2)**。事象別の確度は §4 の表。**原因は 1 つではなく最低 3 系統が混在** (非常停止 SW の断線 = 確定、PC↔CAN リンク喪失 → 0x8250 = 15:35 は確定、モータ電源 RSD-60L-24 の容量不足 → 0x3220 = 候補)
- 日付: 2026-09-19。5号館 2 周 bag 収録後、バッテリ交換 (2 本直列のまま新品へ) 以降に発生。同日午後に再現実験 + ライブ SDO 読みで初めてコードを取得
- 環境: harness v2 (`~/Downloads/reRobot_harness_v2.drawio.png`、§2)。Docker `rerobot_env` / ROS 2 Jazzy / ros2_canopen / EPOS4 Compact 50/5 ×2 / LAWICEL CANUSB (slcand -s8) / ELECOM JC-U4113S ゲームパッド (usb 1-1 直結) / UGREEN 4 ポートハブ (1-2.x)
- 関連: `docs/issue/2026-08-11_joy_spin_epos_shutdown_usb_stall.md` (同署名 `urb -32` ×3 事象、コード無し)、`docs/issue/2026-09-13_decel_epos_stop_12v_loss.md` (旧配線 harness v1 の電源事故)、`docs/reference/wiring_diagram.mmd` (**v1 = 旧配線。v2 は未転記**)、`docs/reference/2026-07-27_epos4_studio_startup_settings.md`

## 1. ユーザ報告 (原文要約)

1. 5号館 2 周 bag 収録 → バッテリ交換。この時点では 0.5 m/s 急発進・段差ともに問題なし。
2. 交換後、アスファルトのわずかな段差で停止 (赤)。再起動して走行再開 → 反射板の 2 cm 段差を越えた後に**暴走** (コントローラを受け付けず止まらない) → 非常停止で電源遮断。以後、わずかな段差・急発進・急停車で頻繁に停止。
3. 帰着後: EPOS 緑点滅 (Disabled) だが launch の瞬間に赤 → 「非常停止スイッチが壊れていた」ことが判明、修理。
4. 再現実験 (joy、足でタイヤに負荷): (1) 負荷中は EPOS・DSR とも緑。足を離すと joy と無関係に止まらず、launch 側にエラー → 非常停止で赤。(2) 電源再投入後、低速 (~0.1) は問題なし、一気に 0.5 m/s で EPOS 赤・モータ停止、DSR 緑。(3) さらに再起動後は「どれだけ急な指令・負荷でも止まらない」→ 数分後に「止まった。EPOS 赤、モータ停止、指令が効かない」。

## 2. 電源構成 (harness v2、ユーザ提供図 2026-09-19。**回路図 mmd は v1 のまま = 要更新**)

```
IDX DUO-C198 ×2 直列 (28.8 V nominal, 25〜33.6 V) ─ switch ─┬─ EPOS ×2 の Logic +Vc (直結、非常停止を通らない)
                                                          ├─ emergency switch ─┬─ RSD-60L-24 (絶縁, 24 V 2.5 A 60 W) ─ DSR 50/5 (27 V mode) ─ EPOS R +Vcc
                                                          │                    └─ RSD-60L-24 (同上)                  ─ DSR 50/5 (27 V mode) ─ EPOS L +Vcc
                                                          └─ XW1572 (15〜72 V → 12 V 5 A) ─ UTM-30LX + R-Fans16
PC (ThinkPad) ─ USB hub ─ CANUSB ─ CAN ─ EPOS R ─ EPOS L      (CAN GND = EPOS GND ピン = モータ帰還電流ノード)
             └ hub ─ UTM-30LX USB / Ether-to-USB (R-Fans) / BNO086     (UTM 経由で PC GND ↔ 12 V GND の第 2 経路)
             └ usb 1-1 直結 ─ ゲームパッド
```

| 部品 | 本件に効く事実 | 出典 |
|---|---|---|
| RSD-60L-24 | 絶縁 DC-DC。出力 24 V **2.5 A・60 W**。過負荷 105〜135% で**定電流モード**に折り返し (電圧が下がる)、除去後自動復帰 | Mean Well RSD-60 spec |
| EPOS4 (設定) | Max output current **10 A** (0x3001:02、08-30 設定)。低電圧限界 10 V (0x2201:01 = 10000、09-19 SDO 実読)。過電圧 48 V。0x60C2 補間周期 **10 ms** (bus.yml, 06-19 `ad4be36` で 50 → 10)、RPDO1 同期型、0x6007 Abort connection option = **3 (quick stop)**、0x6061 = 9 (CSV) | 09-19 SDO 実読 |
| EPOS4 (仕様) | 0x8250 RPDO timeout: 「サイクリックモードで補間周期内に PDO を受信できず補間中止。マスタが通信を中断した場合 (同期 PDO の timing violation 含む) にも発生」。0x3220: 「電源が加速電流を供給できない」 | Firmware Spec §7.2.43 / §7.2.7 |
| DSR 50/5 | 27 V モード。24 V 母線 < 27 V < 48 V で説明書の条件を満たす (v1 で問題だった 56 V 設定は解消)。RSD 出力は回生を吸えないので DSR は必須 | 図 + 09-13 issue §3 |
| 0x2200:01 実読 | **24.1 V** (15:30, 15:45) = RSD 出力。09-13 の 30.7 V は v1 のバッテリ母線で比較対象ではない | SDO |
| bus.yml | EPOS 側 consumer heartbeat (0x1016) 未設定。slave heartbeat 2 s。boot SDO で毎起動 0x6040 = 0x80/0x06/0x07 (Fault reset → Switch On) | bus.yml |

## 3. 時系列 (JST。コンテナ `~/.ros/log` は UTC +9 h。kernel = `journalctl -k`)

| 時刻 | 事象 | 出典 |
|---|---|---|
| 14:21〜14:25 | launch ×3。Node 1 は 14:21:58〜14:22:39 に statusword 0x0208 (Fault)。14:22:42 / 14:23:51 にハブ全体が切断・再列挙 (ユーザ抜き差し) | `epos4_controller_{875,1053}_*.log`, kernel |
| **14:31** | スタック停止中に cansend で直読: **両ノード 0x2200:01 = 0 V** (静止・Disabled 状態)。Node 2: 0x1001 = 0x04、0x1003:01 = **0x3220**、0x6041 = 0x0208。Node 1: 履歴 0、0x6041 = 0x0221。statusword bit 4 (Voltage enabled) 両ノード 0 (正常時 0x1237 は 1) | candump 生フレーム `581#4B 00 22 01 00 00 00 00` |
| 14:37 / 14:39 | launch ×2、Node 1 が 0x0208 (Fault) | `epos4_controller_{1300,1491}_*.log` |
| 14:43:12 | ハブ切断 → can0 消滅 | kernel |
| 〜14:50 | **ユーザ: 非常停止スイッチが壊れていたと判明、修理** → 以後 +Vcc 復帰 (15:30 に 24.1 V) | ユーザ |
| 14:55:08 | launch (device_container pid 2047)、両ノード 14:55:16 Enabled (0x1237)。14:55:43 joy 起動 (pid 2207、15:35 まで生存) | ログ |
| **14:57:06** | **両ノード SDO 同時無応答開始** (最初の timeout 1789797426.745)。kernel に USB エラー無し | `device_container_node_2047` |
| 14:57:19〜22 | ゲームパッド usb 1-1 切断 (-19) → 再列挙 (XInput)。joy_node 再オープン成功 | kernel, `joy_node_2210` |
| (この間) | **再現 1 = 暴走**: 足を離しても止まらず → 非常停止 | ユーザ |
| 15:00:42 | UTM (12 V 系) 切断 = 主電源 OFF (0x1003 喪失) | kernel |
| 15:02:47 | ハブ全体切断 (`clear tt error -71`) → 15:02:49 再列挙、can0 再作成 | kernel |
| 15:02:52 | launch (pid 2562)、15:03:02 両ノード Enabled | ログ |
| 15:03:06 | ゲームパッド切断・再列挙 | kernel |
| **15:03:42** | **両ノード SDO 同時無応答開始** (1789797822.865)。kernel に USB エラー無し。ユーザ: 「一気に 0.5 m/s で EPOS 赤・モータ停止、DSR 緑」= **再現 2** | `device_container_node_2562` |
| 15:04:32 | UTM 切断 = 主電源 OFF (0x1003 喪失) | kernel |
| 15:30:12 | launch (pid 2929)、以後ユーザが急指令・負荷試験 → 「止まらなくなった」。15:31:06〜14 に `Internal limit active` 8 s。15:30 SDO 実読 (ROS service): motor1 0x2200:01 = **241 (24.1 V)** | ログ |
| **15:35:10.608〜10.645** | kernel: `ftdi_sio ttyUSB0: read_bulk urb stopped: -32` ×2 → `write_bulk urb stopped: -32` → **`usb 1-2-port3: disabled by hub (EMI?), re-enabling...`** → `can0 (unregistered): slcan off ttyUSB0` → 10.645 device_container 最初の SDO timeout。15:35:11 canusb-up.service が can0 再作成、**lely master は旧 ifindex に張り付き** 15:49 まで 200 件/10 s の timeout 継続 (新 can0 の RX/TX = 0) | kernel, `device_container_node_2929` |
| (直後) | **再現 3**: ユーザ「止まった。EPOS 赤、モータ停止、指令が効かない」 | ユーザ |
| **15:45** | ユーザが別端末で `sudo systemctl restart canusb-up.service` → cansend 直読 (両ノード同一): **0x603F = 0x8250、0x1001 = 0x10、0x1003:00 = 1、0x1003:01 = 0x8250**、0x6041 = 0x0208、0x2200:01 = 241 (24.1 V)、**0x60FF (最後に受けた目標速度) = 0xFFFFF736 = −2250 rpm (非ゼロ)**。Node 1: 0x60C2 = 10 / −3、0x6007 = 3、0x1400:02 = 1、0x1005 = 0x80、0x6061 = 9、0x2201:01 = 10000 | candump 生フレーム `581#4B 3F 60 00 50 82 00 00`, `581#43 03 10 01 50 82 00 00`, `581#43 FF 60 00 36 F7 FF FF` |

## 4. 事象別の判定 (critic 査読後の表現)

| 事象 | 判定 | 根拠 / 残る代替 |
|---|---|---|
| 午前〜14:31: launch の瞬間に赤、+Vcc 0 V | **確定 (その瞬間の状態として)**: モータ電源 +Vcc が無い状態で boot SDO の Switch On / enable → 0x3220。原因は非常停止スイッチの断 (RSD 入力側)。**静止・Disabled 状態で 0 V** を読んでいるので RSD 折り返し (負荷中のみ) ではない。修理後 24.1 V に復帰 | 「launch 前は緑点滅 → launch で赤」は boot SDO が毎回 Fault reset → Switch On を書くため電源再投入なしでも起きる (0x1003 の件数 = 最後のロジック電源投入以降) |
| 午前の走行中「わずかな段差で頻発」 | **未識別 (候補 3)**: (a) 非常停止 SW の接触劣化 (進行して最終的に完全断 = 整合) / (b) RSD-60L-24 の定電流折り返し → 0x3220 / (c) PC↔CAN リンク喪失 → 0x8250。(a)(b) は同じコード・同じ赤・負荷中は同じ 0 V 読みになる | コード未取得 (電源断で喪失)。区別点 = 負荷相関 (SW は負荷と無関係にランダム、折り返しは段差で毎回) |
| 14:57 再現 1 = 暴走 | **本命 (時間制約付き)**: PC 受信側のリンク喪失 (14:57:06、層不明) → SDO 死・joint_states 凍結、しかし SYNC/RPDO は届き続け EPOS は緑のまま回る。14:57:19〜22 のパッド USB 瞬断で joy_node が publish 停止 → teleop_twist_joy も停止 → **`epos4_controller` が最後の非ゼロ Twist を保持し 10 ms 周期で送り続ける** (受信タイムアウト無し、`timer_callback` L253 / `cmdSpeedCallback` L290) | ⚠️ この説明では暴走は**最長 ~4 s** (パッド復帰 14:57:22 で LB 非押下の最初の Joy にゼロが出る)。ユーザが「5 秒以上走った」なら LB を握り続けていたか、説明が壊れる。**ユーザ確認待ち**: 暴走の秒数 / LB を離していたか / RViz の車体モデルが 14:57:06 頃から凍っていたか。リンク喪失の層は H2 (アダプタ CAN 側) / H3 (lely 受信 wedge) が USB 層より近い (kernel 無記録、`read_bulk` の -EPIPE は必ず dev_err に出る) |
| 15:03 再現 2 = 急加速で赤 + SDO 同時死 | **未識別 (候補 2)**: (1) リンク喪失 → 0x8250 (15:35 と同型、単一原因) / (2) 同じ加速過渡が独立に 2 結果 — リンク喪失 + RSD 折り返し 0x3220。ロジックはバッテリ直結なので +Vcc 崩落だけでは SDO は死なない = **どちらでもリンク喪失が別途必要** | コード未取得。0x1003 は 5 エントリ持つので次回は :01〜:05 を読めば順序ごと確定 |
| 15:35 再現 3 = 赤 + 指令が効かない | **確定**: USB リンク喪失 (`urb -32` → `port3 disabled by hub`) → 同期 RPDO 途絶 → EPOS が補間周期 10 ms 内の未受信で **0x8250** → 0x6007 = 3 の**クイックストップで停止**。通信断の瞬間の目標は −2250 rpm (非ゼロ) = **EPOS の自衛が効かなければ暴走していた局面**。この run の 0x1003 に 0x3220 / 0x8120 / 0x81FD は無い = 電源崩落も EPOS 側 CAN エラーも無しで、純粋に「フレームが届かない」だけで止まった | USB 喪失の**原因**は未識別 (§5 H4/EMI)。kernel の「(EMI?)」は hub.c の推測文言で証拠ではない。ハブ電源不足はポート 3 単独 disable のため不利 |
| 08-11 / 07-31 の 3 事象 | **類推**: 08-11 16:50:14 `urb -32` ×3 + 赤は 15:35 と kernel 署名が一致 → 直接機構は 0x8250 と類推 (当時のコードは無い)。当時の「電流/回生スパイク → 電源巻き添え」は根拠の無かった診断 | USB 喪失の原因は当時も今も未識別 |

## 5. 仮説 (棄却できていないものを列挙)

| # | 仮説 | 今の証拠との関係 | 棄却実験 |
|---|---|---|---|
| H1 | **PC 側ジッタで 0x8250** (sync 10 ms = 0x60C2 10 ms、マージン 0) | 純ジッタなら「赤だが SDO 可・joint_states 継続」になるはず。今日の 3 事象は全てリンク死同伴 → 今日の説明にはならない。ただし 06-19 以降「赤 → 電源断」運用でコード喪失していたので、埋もれていた可能性は否定できない | 0x60C2 を 20〜30 ms に広げて運用し「赤だが SDO 可」が消えるか |
| H2 | **アダプタ CAN 側の死** (GND シフト由来ビットエラーでエラーパッシブ/バスオフ、RX FIFO overrun 後の受信停止。slcand は `-f` 無しでエラーフラグを読まず、slcan にバスオフ自動復帰無し) | 14:57 (RX のみ死)・15:03 (TX+RX 死) を kernel 無記録のまま説明できる。15:35 は USB 層なので別 | 次回 SDO 死 (kernel 無記録) 時、slcand 再起動**前**に status flags を読む: `killall slcand; stty -F /dev/ttyCANUSB raw; (cat /dev/ttyCANUSB &); printf 'F\r' > /dev/ttyCANUSB` (bit0 RX FIFO full / bit3 overrun / bit5 error passive / bit7 bus error)。`cat /sys/class/net/can0/statistics/rx_packets` を 2 回読み heartbeat 分増えているか |
| H3 | **lely (ros2_canopen) 受信側ソフト wedge** — SYNC/RPDO はタイマで出続け、受信処理だけ止まる | 14:57 の署名 (TX 生存・RX 死・両ノード同時) に一致。トリガ不明 | 常時 `candump -ta -l can0` (§7-1)。SDO timeout 中に TPDO/heartbeat が見えれば H3、見えなければ H2/USB |
| H4 | **機械的振動・接触不良** (足で車体を揺らす・段差・急加減速は電流過渡と機械衝撃が**完全交絡**) | パッドは今日 5 回以上、いずれもユーザが車体/ケーブルを扱っている時刻に電源瞬断型 (DirectInput ↔ XInput) 再列挙。08-11 の幽霊列挙・tty 移動も差し直しの証拠。UTM も同じハブで不安定 (`2026-08-11_utm30lx_usb_instability.md`) → **EMI と同程度に有力** | (A) 車輪を浮かせ・ハブとケーブルを固定して急加減速 + 手ブレーキ (電流のみ)。(B) EPOS 主電源 OFF・ロジック ON (heartbeat 流れる) でハブ・CANUSB・パッドのコネクタを叩く/揺らす。A だけ → 電気、B だけ → 機械、両方 → 両方 |
| H5 | **RSD-60L-24 の定電流折り返し → +Vcc 崩落 → 0x3220** (60 W / 2.5 A per EPOS vs Max output current 10 A。段差で速度が乗った状態で 10 A 制限に当たると 100 W 超) | spec 7.2.7 の cause 文言「Power supply cannot supply required acceleration current」がそのまま。RSD は 09-14 の配線図以後に導入 = 「段差で頻発」が新症状なら強く整合。**ただし 15:30〜15:35 run では 0x1003 に 0x3220 無し** (`Internal limit active` 8 s でも折り返していない)。停動 (足ブロック) 時は母線電流 ≈ I × duty ≈ 2 A で折り返さない = 14:57 に緑のままと整合 | **0x2200:01 を 10 Hz ポーリング** (09-13 §6-3 のワンライナー) しながら同じ段差・0.5 m/s ステップ。+Vcc が 15 V 未満に落ちれば確定。RSD **入力側**をテスタで同時に見る (入力が保たれていれば折り返し、0 V なら非常停止 SW/入力側) |
| (EMI/GND) | 非絶縁 CANUSB 経由で PC GND がモータ帰還ノードに接続、UTM USB で第 2 経路 = **GND ループが存在する** (harness v2)。モータ過渡の一部が USB ケーブル GND を流れ USB 信号の同相基準を揺らす経路として成立 | 「物理的に可能」であって「強める証拠」ではない。H4 と現証拠では識別不能 | 絶縁型 USB-CAN アダプタ (または CAN 側絶縁) で再発が消えるか — H2 と USB 側 EMI の両方に効く (対策としては良いが原因識別には H4 の A/B を先に) |

## 6. 構造的な問題 (原因が何であれ効く対策の根拠)

1. **通信断で止まらない層がある (確定)**: EPOS 側は 10 ms の RPDO timeout (0x8250 → quick stop) で守られるが、**PDO が届き続ける形の故障** (受信側のみ喪失・パッド喪失) では `epos4_controller` が最後の指令を無期限に送り続ける。joy_node はデバイス喪失時にゼロ Joy を出さず publish 停止 (ros-drivers/joystick_drivers `handleJoyDeviceRemoved`)、teleop_twist_joy は Joy 受信時のみ publish。EPOS 側 consumer heartbeat (0x1016) はこの型 (TX 生存) には無効。
2. **フォルトコードが残らない運用 (確定)**: ros2_canopen は EMCY を `/diagnostics` にしか流さない (端末に出ない)、0x1003 は電源断で消える、boot SDO が毎起動 Fault reset。今日、主電源を切った 2 回 (15:00, 15:04) でコードを失った。**非常停止のみ (ロジック直結で生きる) → 0x1003:01〜05 読み → 電源断** の順に変える。
3. **can0 再作成を master が追従しない (確定)**: `can0 (unregistered)` 後に canusb-up.service が can0 を再作成しても lely は旧 ifindex に張り付く (15:35:11〜15:49 で実証)。「can0 UP = 正常」判定は無効 (08-11 の教訓の再確認)。
4. **モータ電源容量 (候補 H5 だが設計として要見直し)**: RSD-60L-24 ×2 = 120 W に対し EPOS 2 台の Max output current 10 A × 24 V = 480 W 級のピーク要求。折り返しが実証されなくても設計マージンが無い。
5. **0x60C2 = sync = 10 ms でマージン 0** (H1)。0x6077 (torque) の SDO ポーリング (~10 Hz × 2 ノード) が常時走りバス/slcan 負荷を上げている (PDO 未マップのため)。
6. CANUSB の tty が 9600 baud 表示 (`stty -F /dev/ttyCANUSB`、slcand に `-S` 無し) — FT245 系なら無関係だが未確認。走行中の frame/s (推定 550〜600) を一度 `ip -s link show can0` で測るべき。

## 7. 次の一手 (安い順・判別力順)

1. **常時 `candump -ta -l can0` を回してから走行** (0 円)。(i) EPOS の EMCY フレーム (0x081/0x082) に error code が乗るので電源断で失われない、(ii) EPOS フォルトとリンク死の順序がミリ秒で決まる、(iii) SDO timeout 中に TPDO/heartbeat が見えれば H3、SYNC/RPDO (loopback) はあるが EPOS 側フレームが無ければ H2/USB、両方無ければ TX 死。赤になったら**電源を切る前に** 0x1003:01〜05 を読む:
   ```bash
   candump -ta can0,581:7FF,582:7FF &
   for s in 01 02 03 04 05; do cansend can0 601#400310${s}00000000; cansend can0 602#400310${s}00000000; sleep 0.2; done
   cansend can0 601#4000220100000000; cansend can0 602#4000220100000000   # 0x2200:01 電源電圧 (0.1 V)
   ```
2. **0x2200:01 を 10 Hz ポーリングしながら同じ段差・0.5 m/s ステップ** + RSD 入力側テスタ (H5、20 分)。落ちれば「段差で頻発」は通信ではなく電源容量問題に分類が変わる。
3. **機械 vs 電気の分離** (H4 の A/B、各 10 分)。
4. **ユーザ確認**: 14:57 暴走の秒数 / LB を離していたか / RViz 凍結の有無。DB9 で CAN GND が結線されているか。ハブの電源元。
5. 設定衛生 (原因が何であれ有効、ただし「効いた = 原因確定」とは書かない): 0x60C2 を 20〜30 ms、`epos4_controller` に Twist 受信ウォッチドッグ、fault ビット監視 → 0x1003 即読みログ、can0 再作成検知。

## 8. 対策候補

| 層 | 対策 | 備考 |
|---|---|---|
| ソフト (最優先) | `epos4_controller` に `/robot_speed_cmd` 受信タイムアウト (0.5 s 途絶でゼロへランプ) | 14:57 型を止める唯一の層。H2/H3/H4/EMI のどれが真でも有効 |
| ソフト | statusword Fault ビット監視 → 0x1003 即読み → ERROR ログ + `/diagnostics`。candump ロガーの常時起動 (scripts に組込み) | §6-2 |
| ソフト | lely の can0 ifindex 追従不能の検知 → 全 SDO timeout 連続でスタック停止を促す | §6-3 |
| 設定 | 0x60C2 20〜30 ms (反応時間は人間には無害)。0x6077 を TPDO に載せる or ポーリング停止 | H1 / 負荷低減 |
| ハード (電源) | A. バッテリ直結 + **DSR 70/30 を 38〜42 V** (2 本直列 25〜33.6 V < しきい値 < EPOS 48 V)。DUO-C198 放電上限 14 A に対し EPOS 2 台 × 10 A は 28.8 V 換算で ~17 A → Max output current を 7 A 程度に下げるか BMS 遮断を許容するかを決める。 B. 絶縁 DC-DC を 300 W 級以上 (24 V 12.5 A 以上) に替え DSR 27 V モードは維持 | v2 の長所 (12 V 系を絶縁で回生から守る) は残す。H5 の実証結果で優先度を決める |
| ハード (通信) | 絶縁型 CAN アダプタ (PCAN-USB opto / Kvaser Leaf 等) または CANUSB 手前に USB アイソレータ。CAN ケーブルはシールド付きツイスト、両端 120 Ω (CAN_H–CAN_L 約 60 Ω)、モータ配線と束ねない。PC↔車体の GND 接続点を 1 か所以下に | H2/EMI 両方に効く。H4 (機械) には効かないので A/B 実験を先に |
| ハード (機械) | ゲームパッドのケーブル/コネクタ交換、ハブとコネクタの固定、外部給電ハブ | H4 |
| 運用 | 電源対策が終わるまで急指令・足で負荷・段差の試験は中止。走行が必要なら 0.25 m/s・turbo なし。フォルト後は非常停止のみ → 0x1003 読み → 電源断 | |

## 9. critic 査読の記録 (2026-09-19)

- **第 1 回 (午前の 0 V)**: 主担当「+Vcc 喪失 → 0x3220、回生ではない」→ 判定 = 本命。修正: (a) ノード別ログ (`~/.ros/log/epos4_controller_*.log`) は残っており Node 1 も Fault していた、(b) boot SDO の Fault reset で「launch の瞬間に赤」は電源再投入なしで起きる、(c) 「+Vcc ラインが死んでいる」は SW OFF / 断続 / DSR 焼損と未識別 → その後ユーザが非常停止 SW の故障を発見・修理し、24.1 V 復帰で SW 説が確定。代替 4 (低電圧限界の変更) は 0x2201:01 = 10000 で棄却。
- **第 2 回 (USB → 0x8250 連鎖)**: 判定 = 15:35 単体は確定級、**「全事象の主機構 = USB ストール」への一般化は棄却**。修正: (a) 14:57・15:03 は「リンク喪失」までが事実で層 (USB / アダプタ CAN 側 / slcan / lely) とコードは未特定、(b) 14:57 の複合説明は暴走 ~4 s 以内という時間制約を持つ → ユーザ確認、(c) 「EMI」は kernel の推測文言で証拠ではなく、機械振動 (H4) と完全交絡、(d) 08-11 の診断は「類推」に留め上書きしない、(e) RSD 折り返し (H5) は候補のまま (当該 run に 0x3220 無し)、(f) 午前 0 V は静止時測定なので SW で確定して良い、(g) 0x60C2 マージン 0 (H1)・0x6077 ポーリング負荷・9600 baud 表示・lely の ifindex 張り付きを見落としとして追加。
