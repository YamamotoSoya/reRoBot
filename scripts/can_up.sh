#!/usr/bin/env bash
# claude: can0 の状態確認 + 復旧 (ホスト側で実行)。
# 通常は USB-CAN アダプタ (FTDI 0403:6001) を挿すだけで
#   udev (/etc/udev/rules.d/99-hokuyo-devices.rules) → /dev/ttyCANUSB 作成
#   → systemd (canusb-up.service) → slcand -s8 (1 Mbps) + ip link set can0 up
# まで自動で行われるので、このスクリプトの手動実行は不要。
# can0 が落ちているときの再確認・復旧用に残している。
# 注意: can0 は slcan なので `ip link set ... type can bitrate ...` は使えない
#       (ビットレートは slcand の -s8 で決まる)。
set -eu

if ip link show can0 2>/dev/null | head -1 | grep -qE '[<,]UP[,>]'; then
  echo "[can_up] can0 is already up (canusb-up.service による自動起動)"
  ip -details link show can0 | head -3
  exit 0
fi

if [ ! -e /dev/ttyCANUSB ]; then
  echo "[can_up] ERROR: /dev/ttyCANUSB がありません — USB-CAN アダプタ未接続です。" >&2
  echo "[can_up] アダプタを挿せば udev + canusb-up.service が自動で can0 を上げます。" >&2
  exit 1
fi

echo "[can_up] can0 が落ちています — canusb-up.service を再起動します (要 sudo)"
sudo systemctl restart canusb-up.service
sleep 1
ip -details link show can0 | head -3
echo "[can_up] can0 up (slcan, 1 Mbps)"
