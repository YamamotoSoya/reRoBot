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

* **3D LiDAR (R-Fans-16, Ethernet) — 挿すだけで自動接続**

セットアップ済みのマシンでは、LiDAR 用 USB-Ethernet アダプタ (`enxcce1d5021885`) を挿すと
NetworkManager プロファイル `rfans` が自動適用され、ホストに静的 IP `192.168.0.100/24` が付く。
LiDAR 実機は `192.168.0.3` で、データは UDP 2014 へブロードキャストされる (ホスト側 IP を LiDAR に教える必要はない)。
コンテナは `network_mode: host` なので、ホストに IP が付けば追加設定なしで届く。

```bash
$ ip -br addr show enxcce1d5021885   # 192.168.0.100/24 が付いていれば OK
$ ping -c 3 192.168.0.3              # LiDAR 疎通確認
```

<details>
<summary><b>新しいマシン / 別のアダプタをセットアップする手順 (初回のみ)</b></summary>

NetworkManager に「このアダプタが現れたら固定 IP を付ける」プロファイルを 1 度作るだけ。
`ip addr add` と違い再起動・抜き差しをまたいで自動適用される。

1. LiDAR 用アダプタを挿し、インターフェース名を確認 (`enx` + アダプタの MAC。個体固有で、
   どの USB ポート/ハブ経由でも同じ名前になる)

```bash
$ ip -br link      # 例: enxcce1d5021885
```

2. プロファイル作成 (`ifname` は上で確認した名前に差し替える)

```bash
$ nmcli connection add type ethernet \
    con-name rfans \
    ifname enxcce1d5021885 \
    ipv4.method manual \
    ipv4.addresses 192.168.0.100/24 \
    connection.autoconnect yes \
    connection.autoconnect-priority 100
$ nmcli connection up rfans   # 初回のみ即時有効化 (以後は挿すだけで自動)
```

3. 疎通確認

```bash
$ ip -br addr show <ifname>   # 192.168.0.100/24 が付いていること
$ ping -c 3 192.168.0.3
```

※ gateway を指定していないのでデフォルトルートは奪わず、Wi-Fi のインターネット接続と共存できる。
※ `ifname` 紐付けは MAC 由来名なので「この個体 = LiDAR 係」の専用化が前提。アダプタを
別個体に交換したら手順 1〜2 をやり直す (`nmcli con delete rfans` で旧設定を消してから)。
※ `autoconnect-priority 100` は、ifname 指定なしの汎用「Wired connection」系プロファイルとの
競合時に rfans を勝たせるための指定。
</details>

* **2D LiDAR (HOKUYO UTM-30LX) — 挿すだけで固定名 `/dev/ttyUSB-utm-30lx`**

udev ルールにより、挿すと固定名 `/dev/ttyUSB-utm-30lx` (権限 0666) が作られる。
この名前は `rerobot_bringup_2d.launch.py` の `serial_port` デフォルト値と一致しているので、
**挿すだけで 2D bringup がそのまま動く** (systemd サービス等は不要 — シリアル直結なので
名前と権限さえ固定できればよい)。

```bash
$ ls -l /dev/ttyUSB-utm-30lx    # symlink が ttyUSB* を指していれば OK
```

<details>
<summary><b>新しいマシンをセットアップする手順 (初回のみ)</b></summary>

CANUSB と同じ `/etc/udev/rules.d/99-hokuyo-devices.rules` に追記:

```
# UTM-30LX
SUBSYSTEM=="tty", ATTRS{idVendor}=="15d1", ATTRS{idProduct}=="0000", SYMLINK+="ttyUSB-utm-30lx", MODE="0666"
```

```bash
$ sudo udevadm control --reload
# 挿し直して:
$ ls -l /dev/ttyUSB-utm-30lx
```

※ `MODE="0666"` は「dialout グループに入っていなくても読み書きできる」ようにするため。
これが無いと urg_node が Permission denied で落ちる。
</details>

* **IMU (BNO086 ボード) — 挿すだけで固定名 `/dev/bno086`**

udev ルールにより、通常動作 (firmware 書き込み済み) のボードを挿すと固定名 `/dev/bno086`
(権限 0666) が作られる。番号が変動する `/dev/ttyACM*` を追いかける必要はない。
`bno086_imu_driver` の `port` パラメータ既定値は `/dev/ttyACM0` なので、起動時は
`port:=/dev/bno086` を指定する。

```bash
$ ls -l /dev/bno086             # symlink が ttyACM* を指していれば OK
```

<details>
<summary><b>新しいマシンをセットアップする手順 (初回のみ)</b></summary>

ルールはドライバ同梱の `ros2_ws_main/src/drivers/BNO086_ROS2Board-main/tools/99-bno086.rules`
と同内容。`/etc/udev/rules.d/99-hokuyo-devices.rules` に追記でも、同梱ファイルの
`sudo cp` でもよい:

```
# bno086
# Running firmware: USB CDC. Note that 0483:5740 is ST's generic CDC pair, so
# any other STM32 CDC device would match too - drop the SYMLINK line if you
# have more than one plugged in at once.
SUBSYSTEM=="tty", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE="0666", SYMLINK+="bno086"

# ROM bootloader (S2 BOOT + S1 RESET), so STM32_Programmer_CLI / dfu-util can
# flash without sudo.
SUBSYSTEM=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="df11", MODE="0666"

# ST-Link, for SWD flashing via OpenOCD / st-flash.
SUBSYSTEM=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="374b", MODE="0666"
SUBSYSTEM=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="3748", MODE="0666"
```

```bash
$ sudo udevadm control --reload
# 挿し直して:
$ ls -l /dev/bno086
```

3 ブロックの役割:

| ブロック | ボードの状態 | 用途 |
|----------|--------------|------|
| `0483:5740` (CDC) | 通常動作 (firmware 実行中) | IMU データ読み取り。`/dev/bno086` を作る |
| `0483:df11` (DFU) | ROM ブートローダ (S2 BOOT + S1 RESET で起動) | `STM32_Programmer_CLI` / `dfu-util` での firmware 書き込みを sudo なしで |
| `0483:374b` / `3748` (ST-Link) | — (書き込み器側) | OpenOCD / st-flash での SWD 書き込みを sudo なしで |

※ `0483:5740` は ST 汎用の CDC ID なので、**別の STM32 CDC デバイスを同時に挿すと
そちらにも `/dev/bno086` が付き得る**。複数挿す運用になったら SYMLINK 行を外すか
シリアル番号 (`ATTRS{serial}`) で絞る。
</details>