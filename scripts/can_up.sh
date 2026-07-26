#!/usr/bin/env bash
# claude: CAN インターフェース起動 (ホスト側で実行、要 sudo)。
# EPOS4 ×2 が乗る can0 を 1 Mbps で up する。bringup 前に 1 回だけ実行。
set -eu
sudo ip link set can0 up type can bitrate 1000000
ip -details link show can0 | head -3
echo "[can_up] can0 up (1 Mbps)"
