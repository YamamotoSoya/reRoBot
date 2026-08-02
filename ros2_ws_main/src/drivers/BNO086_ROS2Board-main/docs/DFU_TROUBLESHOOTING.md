# DFU書き込みトラブルシューティング記録 (2026-08-02)

## 症状

`dfu-util`でファームウェアを書き込もうとすると、進捗が **0%のまま止まる**、または
`dfu-util: dfuse_download: libusb_control_transfer returned -9` (`LIBUSB_ERROR_PIPE`)
で失敗する。

## 環境

- PC: tumutamu-IdeaPad-Slim-5-14IMH9 (Ubuntu, カーネルUSBスタック `xhci_hcd`)
- 対象ボード: BNO086 ROS2 IMU Board (fTomo-robot製、STM32)
- 通常時のUSB ID: `0483:5740`（STM32 Virtual COM Port、`ttyACM0`として認識）
- DFUモード時のUSB ID: `0483:df11` (`STM32 BOOTLOADER`, DFU version 011a)
- 書き込みコマンド:
  ```bash
  dfu-util -a 0 -s 0x08000000:leave -D build/fw/bno086_ros2board.bin
  ```
- ファームウェアサイズ: 19920 bytes（対象フラッシュ領域: `0x08000000` 開始, 32 x 1KB = 32KB, 属性 `rew`）

## これまでに切り分け・実施したこと

### 1. Ctrl-Cによる中断で `dfuERROR`/`dfuDNBUSY` 状態に固着
- 最初の試行でCtrl-Cにより中断 → 次回実行時に `state = dfuDNBUSY` や `dfuERROR` から始まるようになった。
- **これ自体は正常動作**: dfu-utilは起動時に自動で `dfuERROR, clearing status` → `dfuIDLE` にクリアしてくれる。根本原因ではなかった。

### 2. USB接続の物理的不安定性(解消済み)
- `journalctl --dmesg` を確認したところ、当初は **約10秒おきにUSBが切断・再接続を繰り返す**現象があった。
  ```
  usb 3-4: USB disconnect, device number 15
  usb 3-4: new full-speed USB device number 16 ...
  （以降10秒間隔で繰り返し）
  ```
- ハブ(VIA Labs / Genesys Logicの多段ハブ)経由 → PC本体ポートへ直結しても解消せず。
- `power/control` は `on`（オートサスペンド無効）を確認済みで、これも原因ではなかった。
- **USBケーブルを交換したところ、この切断ループは解消**。以降は数分間安定して接続維持できることを確認済み（`lsusb -t` でも `xhci_hcd` 配下、`480M`の通常USB2速度ポートとして安定認識）。
- → **ケーブル不良が切断ループの原因だった**。この問題は解決済み。

### 3. 転送そのものが進まない(未解決・現在の本�題)
ケーブル交換後、USB接続自体は安定した状態で以下を確認:

- **クリーンな `dfuIDLE` から開始した場合**: `Download [ ] 0% 0 bytes` のまま、エラーも出さず**無限にハング**する（数分待っても変化なし、libusbのタイムアウトも発生しない）。
- **`dfuDNBUSY`/`dfuERROR` から開始した場合**（前回のハングをkillした直後など）: 即座に
  ```
  dfu-util: dfuse_download: libusb_control_transfer returned -9
  ```
  で失敗する。
- `-t 1024`（転送サイズ縮小）を試したが改善なし。
- **Download(書き込み)だけでなく Upload(読み出し) `-U` でも同じ箇所で `-9` エラー**が発生することを確認。
  → 読み出しはフラッシュ書き込み保護(WRP)や読み出し保護(RDP)の影響を受けないはずの操作なので、**フラッシュ保護が原因ではない**と考えられる。
  → DfuSe特有の「特殊コマンド」（アドレスポインタ設定・消去など、5バイト程度の小さいDNLOAD control OUT転送）が、Download/Uploadどちらの場合でも共通して失敗している。

### 現時点の仮説
`dfu-util` 0.9 が発行するDfuSe特殊コマンド（小さなcontrol転送）と、このPCの `xhci_hcd`(xHCI/USB3コントローラ)ドライバの組み合わせに起因する既知の互換性問題である可能性が高い。ネット上でも `dfu-util` + STM32 DfuSe + xHCIコントローラで同種の `libusb_control_transfer returned -9` 報告が複数ある。ケーブル不良の切断ループとは**別の問題**として現在も未解決。

## 試していない・次に試すべき対策

1. **STM32CubeProgrammer（STMicro純正ツール）で書き込みを試す**
   dfu-utilよりxHCI環境での互換性が高いとされる。要インストール。
2. **ST-Link/SWD経由での書き込み**（ボードにSWDIO/SWCLKピンがあれば）
   USB DFUプロトコル自体を回避できるため、根本的に問題を避けられる可能性が高い。
3. **別のPCや、USB2.0専用ポート（EHCIコントローラ）で試す**
   xHCI固有の問題であれば、古いUSB2.0コントローラ搭載機や、実機のUSB2.0専用ポートがあれば切り分け可能。
4. （未検証）`dfu-util`のバージョン違い（他バージョンやビルドオプション違い）で改善するか。

## 参考: 現在のUSB状態確認コマンド

```bash
lsusb | grep -i 0483
lsusb -t
journalctl --dmesg --no-pager -n 30
```
