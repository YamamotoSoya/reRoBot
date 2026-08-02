# reRoBot
Autonomous navigation robot for the Tsukuba Challenge

## Setup
### clone
Please add '--recursive' option when you clone this repository
```bash
$ git clone --recursive https://github.com/YamamotoSoya/reRoBot.git
```

If the symbolic link is broken, please recreate it manually
```bash
$ cd ~/reRoBot/src
$ ln -s external/maxon_epos4_ros2_repo/maxon_epos4_ros2 .
```
### docker
```bash
$ xhost +local:docker # set host before activate docker
$ docker compose up --build # activate
$ docker exec -it rerobot_env bash # enter
$ docker copose down # deactivate
```
### colcon build
```bash
$ cd /workspace
$ rosdep update
$ rosdep install --from-paths src --ignore-src --simulate # check dependencies

$ colcon build --symlink-install --executor sequential
$ source install/setup.bash
```
### Hardware setup
* **USB-CAN アダプタ (CANUSB) — can0 は挿すだけで自動起動**

セットアップ済みのマシンでは、USB-CAN アダプタ (FTDI FT232, `0403:6001`) を挿すと
udev ルール → `/dev/ttyCANUSB` 作成 → systemd サービス `canusb-up.service` → `slcand` (1 Mbps) + `can0` up
まで全自動で行われる。**手動操作は不要**。状態確認・復旧は `./scripts/can_up.sh`。

```bash
$ ip link show can0        # 動作確認 (UP になっていれば OK)
$ ./scripts/can_up.sh      # 落ちているときの確認 + 復旧
```

<details>
<summary><b>新しいマシンをセットアップする手順 (初回のみ)</b></summary>

1. can-utils のインストール
```bash
$ sudo apt-get update && sudo apt-get install can-utils
```

2. カーネルモジュールの自動ロード設定 — `/etc/modules-load.d/can.conf` に以下を記述
```
can
can_raw
slcan
```

3. udev ルール — `/etc/udev/rules.d/99-hokuyo-devices.rules` (新規なら任意の `.rules` ファイル) に追記。
アダプタに固定名 `/dev/ttyCANUSB` を与え、挿入時に systemd サービスを起動させる
```
# CANUSB
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", SYMLINK+="ttyCANUSB", MODE="0666", TAG+="systemd", ENV{SYSTEMD_WANTS}="canusb-up.service"
```

4. systemd サービス — `/etc/systemd/system/canusb-up.service` を作成
```ini
[Unit]
Description=Setup CANUSB interface (can0)
After=dev-ttyCANUSB.device
BindsTo=dev-ttyCANUSB.device

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart=/bin/bash -c 'slcand -o -c -s8 /dev/ttyCANUSB can0 && ip link set can0 up'
ExecStop=/bin/bash -c 'ip link set can0 down; killall slcand'

[Install]
WantedBy=dev-ttyCANUSB.device
```

5. 有効化と動作確認
```bash
$ sudo systemctl daemon-reload
$ sudo systemctl enable canusb-up.service
$ sudo udevadm control --reload
# アダプタを挿し直して:
$ ip link show can0
```

※ `slcand -s8` が 1 Mbps 指定。can0 は slcan インターフェースなので、ビットレートを
`ip link set ... type can bitrate ...` で変更することはできない (slcand 側で決まる)。
</details>