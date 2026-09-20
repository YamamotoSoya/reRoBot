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
  # claude: can0 が UP でも安心できない — アダプタが USB ストール (urb -32) で再列挙されると、
  # boot 時の slcand が「死んだ旧 ttyUSB」を掴んだまま can0 だけ UP に見える (2026-08-11 の
  # 急回転事故で実証: /dev/ttyCANUSB は新デバイスを指すが slcand は旧デバイスに接続したまま)。
  # slcand が実際に開いている tty と /dev/ttyCANUSB の実体を突き合わせて検出する。
  slcand_pid="$(pgrep -x slcand | head -1 || true)"
  slcand_tty=""
  if [ -n "${slcand_pid}" ]; then
    slcand_tty="$(readlink "/proc/${slcand_pid}/fd"/* 2>/dev/null | grep -m1 '^/dev/ttyUSB' || true)"
  fi
  canusb_tty="$(readlink -f /dev/ttyCANUSB 2>/dev/null || true)"
  if [ -n "${slcand_pid}" ] && [ -n "${canusb_tty}" ] && [ "${slcand_tty}" != "${canusb_tty}" ]; then
    echo "[can_up] ⚠️ can0 は UP だが slcand が旧デバイス (${slcand_tty:-不明}) を掴んだまま。"
    echo "[can_up]    現在の CANUSB は ${canusb_tty} — canusb-up.service を再起動します (要 sudo)"
    sudo systemctl restart canusb-up.service
    sleep 1
    ip -details link show can0 | head -3
    echo "[can_up] can0 re-attached (slcan, 1 Mbps)"
    exit 0
  fi
  # claude_watchdog (2026-09-19): can0 UP・slcand も正しい tty でも、USB 再列挙の同一秒に slcand が
  # 起動した場合はアダプタの CAN 側が開かず「送信は出るが受信ゼロ」の半死状態になる (同日 2 回再現、
  # 15:35 / 17:16。手動 restart で毎回復帰)。EPOS の heartbeat (2 ノード × 0.5 Hz) を 3 s 待ち、
  # 送信が出ているのに受信が 1 フレームも無ければ service を再起動する。
  # EPOS のロジック電源が切れているときも同じ見え方になるので、その場合は再起動しても無害。
  rx0="$(cat /sys/class/net/can0/statistics/rx_packets)"
  tx0="$(cat /sys/class/net/can0/statistics/tx_packets)"
  sleep 3
  rx1="$(cat /sys/class/net/can0/statistics/rx_packets)"
  tx1="$(cat /sys/class/net/can0/statistics/tx_packets)"
  if [ "$rx1" -eq "$rx0" ] && [ "$tx1" -gt "$tx0" ]; then
    echo "[can_up] ⚠️ can0 は UP だが 3 s 間 受信 0 フレーム (送信は ${tx0}→${tx1})。"
    echo "[can_up]    アダプタの CAN 側が開いていない可能性 (USB 再列挙直後の slcand 起動競合) — canusb-up.service を再起動します (要 sudo)"
    sudo systemctl restart canusb-up.service
    sleep 3
    echo "[can_up] rx_packets: $(cat /sys/class/net/can0/statistics/rx_packets) (0 のままなら EPOS のロジック電源か CAN 配線を確認)"
    echo "[can_up] ⚠️ can0 の ifindex が変わったので、稼働中の ROS スタックは scripts/stop.sh → 再 launch が必要"
    exit 0
  fi
  if [ "$rx1" -eq "$rx0" ]; then
    echo "[can_up] ℹ️ 3 s 間 送受信ともフレーム無し (スタック停止中で EPOS も無応答?)。EPOS のロジック電源が入っていれば heartbeat が 2 s 周期で見えるはず"
  fi
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
