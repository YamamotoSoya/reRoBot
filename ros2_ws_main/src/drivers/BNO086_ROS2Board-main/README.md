# BNO086_ROS2Board

[![CI](https://github.com/MechanicalGirlDev/BNO086_ROS2Board/actions/workflows/ci.yml/badge.svg)](https://github.com/MechanicalGirlDev/BNO086_ROS2Board/actions/workflows/ci.yml)
[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)
[![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy-blue.svg)](https://docs.ros.org/en/jazzy/)

BNO086 (9軸IMU) + STM32F042 の自作基板を、ROS 2 の IMU センサとして使うための
一式です。回路図・基板データからファームウェア、ROS 2 パッケージまで全部入りです。

```
┌──────────────┐   SPI    ┌──────────────┐   USB CDC / UART   ┌─────────────┐
│   BNO086     ├─────────►│  STM32F042C6 ├───────────────────►│   ROS 2     │
│  (SH-2/SHTP) │          │  (firmware/) │   独自バイナリ      │  (ros2_ws/) │
└──────────────┘          └──────────────┘                    └─────────────┘
                                                    sensor_msgs/Imu
                                                    sensor_msgs/MagneticField
```

| ディレクトリ | 内容 |
|---|---|
| `PCB/` | KiCad の回路図・基板データ (既存) |
| `firmware/` | STM32F042C6 用ファームウェア (CMake + STM32 HAL) |
| `ros2_ws/` | ROS 2 ワークスペース (`bno086_imu_driver` パッケージ) |
| `tools/` | udev ルール、シリアル↔TCP 中継 |
| `docker/` | ROS 2 コンテナ (macOS 用。[付録](#付録-macos-で動かす)) |
| `docs/` | [ハードウェア仕様](docs/HARDWARE.md) / [プロトコル仕様](docs/PROTOCOL.md) |

## 動作環境

**Ubuntu 24.04 + ROS 2 Jazzy をネイティブに入れた PC** を前提にしています。
基板は USB CDC で `/dev/ttyACM*` として見えるので、追加のハードウェアは要りません。

| | 用途 | 備考 |
|---|---|---|
| **ROS 2 Jazzy** | ドライバの実行 | 他のディストリでも動くはずですが未検証 |
| **python3-serial** | シリアル通信 | `rosdep` で入ります |
| **ARM ツールチェーン** | ファームウェアのビルド | 書き込み済みの基板を使うだけなら不要 |
| **書き込み手段** | 基板への書き込み | USB だけで済む DFU、または ST-Link |

macOS でも動きますが、Docker Desktop が USB を渡せないため中継が要ります。
開発中の検証用なので[付録](#付録-macos-で動かす)にまわしました。

## クイックスタート

書き込み済みの基板と ROS 2 Jazzy が手元にある場合。

```bash
git clone https://github.com/MechanicalGirlDev/BNO086_ROS2Board.git
cd BNO086_ROS2Board

# シリアルポートの権限 (これを忘れると Permission denied になります)
sudo cp tools/99-bno086.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger

source /opt/ros/jazzy/setup.bash
cd ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install && source install/setup.bash

ros2 launch bno086_imu_driver bno086.launch.py port:=/dev/bno086
```

`ros2 topic hz /imu/data` で 100 Hz 出ていれば成功です。
ROS 2 をまだ入れていない場合は [3. PC 側 ROS 2 環境](#3-pc-側-ros-2-環境) から。

| 手順 | 節 |
|---|---|
| ファームウェアをビルドする | [1](#1-ファームウェアのビルド) |
| 基板に書き込む | [2](#2-書き込み) |
| ROS 2 とこのパッケージを入れる | [3](#3-pc-側-ros-2-環境) |
| ノードを起動してトピックを見る | [4](#4-実行) |
| 座標系と実装角度を理解する | [座標系](#座標系) |
| キャリブレーションする | [5](#5-キャリブレーション) |
| うまくいかないとき | [トラブルシューティング](#トラブルシューティング) |
| 既知の問題を確認する | [実機での検証状況](#実機での検証状況) |
| macOS で動かす (検証用) | [付録](#付録-macos-で動かす) |

## 設計方針

**micro-ROS は使っていません。** STM32F042C6 は RAM 6 KiB / Flash 32 KiB しかなく、
micro-ROS (最低でも RAM 30 KiB 程度) は載りません。代わりに軽量な独自バイナリ
プロトコルで PC に送り、PC 側の ROS 2 ノードが `sensor_msgs/Imu` に変換します。

同じ理由で、CEVA 公式の `sh2` ライブラリではなく、必要な部分だけを実装した
軽量 SHTP/SH-2 クライアント (`firmware/App/bno08x.c`) を使っています。
MCU 側は浮動小数点演算を一切行わず、BNO086 の固定小数点値をそのまま転送します。

現在の使用量: **Flash 19.5 KiB / 32 KiB (61%)、RAM 4.6 KiB / 6 KiB (78%)**

---

## 1. ファームウェアのビルド

書き込み済みの基板を使うだけならこの節は飛ばせます。

```bash
sudo apt install gcc-arm-none-eabi binutils-arm-none-eabi \
                 libnewlib-arm-none-eabi cmake ninja-build

cmake -S firmware -B build/fw -G Ninja
cmake --build build/fw
```

`build/fw/bno086_ros2board.{elf,bin,hex}` ができます。
依存する ST のライブラリ (HAL / CMSIS / USB Device) は CMake の FetchContent が
自動取得するので、初回ビルドのみネットワークが必要です。

ツールチェーンをホストに入れたくない場合は Docker でも同じことができます。

```bash
docker build -t bno086-fw firmware/
docker run --rm -v "$PWD":/work -w /work bno086-fw bash -c \
  'cmake -S firmware -B build/fw -G Ninja && cmake --build build/fw'
```

### 出力先の切り替え

USB と UART は排他です (どちらか一方が詰まってもう一方が止まる事故を防ぐため)。

```bash
# USB CDC (デフォルト)
cmake -S firmware -B build/fw -G Ninja

# USART1 (J4) に出す場合
cmake -S firmware -B build/fw-uart -G Ninja -DHOST_LINK_USB=OFF -DHOST_LINK_UART=ON
```

UART のボーレートは `-DUART_BAUD=921600` で変更できます。

## 2. 書き込み

J3 (SWD) に ST-Link を接続してください。

```bash
# OpenOCD
openocd -f interface/stlink.cfg -f target/stm32f0x.cfg \
        -c "program build/fw/bno086_ros2board.elf verify reset exit"

# あるいは st-flash
st-flash write build/fw/bno086_ros2board.bin 0x08000000
```

### USB DFU で書き込む (ST-Link 不要)

STM32F042 は DFU ブートローダを内蔵しているので、USB ケーブルだけで書き込めます。
**S2 (BOOT) を押しながら S1 (RESET) を押して離す** と DFU モードに入ります
(`lsusb` に `0483:df11` が出ます)。

```bash
sudo apt install dfu-util
dfu-util -a 0 -s 0x08000000:leave -D build/fw/bno086_ros2board.bin
```

書き込みのたびにこのボタン操作が必要です。udev ルール
(`tools/99-bno086.rules`) を入れておくと `sudo` なしで書き込めます。

macOS で STM32CubeProgrammer を使う場合 (実機での書き込みはこの経路で確認済み):

```bash
CLI=/Applications/STM32CubeIDE.app/Contents/Eclipse/plugins/\
com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.macos64_*/tools/bin/STM32_Programmer_CLI
$CLI -c port=usb1 -w build/fw/bno086_ros2board.hex -v -g
```

### 書き込み前の確認

**JP1 / JP2 はどちらもオープンのまま** にしてください。
BNO086 の SPI モードは **PS1=1 / PS0=1** で選択されます。PS1 は R7 のプルアップ、
PS0 はファームウェアが PB0 を High に保つことで実現します (両方が NRST 解除前から
High である必要があります)。JP1 を短絡すると PB0 が GND と衝突します。
詳細は [docs/HARDWARE.md](docs/HARDWARE.md)。

### LED の意味

| LED | 状態 | 意味 |
|---|---|---|
| LED1 (PB12) | 1 Hz 点滅 | 正常動作中 |
| LED2 (PB13) | 点灯 | BNO086 と通信できている |
| 両方 | 高速点滅 | 致命的エラー (`board_error()`) |

## 3. PC 側 ROS 2 環境

### ROS 2 Jazzy を入れる (Ubuntu 24.04)

すでに入っている場合は次の「[このパッケージをビルドする](#このパッケージをビルドする)」へ。

```bash
sudo apt install software-properties-common curl
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
     -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install ros-jazzy-ros-base python3-colcon-common-extensions python3-rosdep
```

(公式手順は [ROS 2 Jazzy のインストール](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
を参照してください。上記はその要約です。)

### このパッケージをビルドする

```bash
source /opt/ros/jazzy/setup.bash

# 依存 (rclpy, sensor_msgs, tf2_ros, python3-serial ...) をまとめて入れる
sudo rosdep init      # 初回のみ
rosdep update
cd ros2_ws && rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install
source install/setup.bash
```

`~/.bashrc` に次を足しておくと毎回の source が要らなくなります。

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "source $PWD/install/setup.bash"   >> ~/.bashrc
```

`--symlink-install` を付けているので、Python を書き換えたときの再ビルドは
不要です (`launch` / `config` を追加したときだけ `colcon build` し直します)。

### シリアルポートの権限

素の状態では `/dev/ttyACM0` を開けず `Permission denied` になります。
同梱の udev ルールを入れてください。

```bash
sudo cp tools/99-bno086.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

USB に挿し直すと **`/dev/bno086`** ができます。ポート番号に依存しない名前なので、
他の USB シリアル機器を抜き挿ししても変わりません。launch では
`port:=/dev/bno086` と書いてください。

udev を使わない場合は `sudo usermod -aG dialout $USER` でも通ります
(**再ログインが必要**)。

## 4. 実行

```bash
ros2 launch bno086_imu_driver bno086.launch.py port:=/dev/bno086
```

publish されるトピック:

| トピック | 型 |
|---|---|
| `/imu/data` | `sensor_msgs/Imu` |
| `/imu/mag` | `sensor_msgs/MagneticField` |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` |

確認:

```bash
ros2 topic hz /imu/data
ros2 topic echo /imu/data --once
```

## 座標系

publish されるデータは **基板の座標系** (`frame_id: imu_link`) です。
右手系で、Z が基板の部品面側 (上) を向きます。X / Y は基板レイアウトの
X / Y 軸に一致します。

姿勢クォータニオンは **body → world** (基板座標系から重力・地磁気基準の世界座標系へ)
の変換です。実機で確認済み (誤差 0.128 / 逆向き解釈だと 1.247)。

> **重要:** BNO086 (U4) はこの基板上で **180° 回転して実装** されています
> (KiCad 上で `回転=180°`)。そのためチップの +X / +Y は基板の −X / −Y を向きます
> (+Z は一致)。ROS 2 ノードは `mount_yaw_deg: 180.0` でこれを補正してから
> publish するので、**利用側で符号を反転する必要はありません**。
> センサの生の座標系がほしい場合は `mount_yaw_deg: 0.0` にしてください。
> 詳細は [docs/HARDWARE.md](docs/HARDWARE.md) の「実装角度」。

機体に対する取り付け向き (前後・左右) は、このパラメータではなく
ロボット側の static transform (`imu_link` → `base_link`) で表現してください。
[REP-103](https://www.ros.org/reps/rep-0103.html) / [REP-145](https://www.ros.org/reps/rep-0145.html)
に沿った運用です。

### 主なパラメータ

`ros2_ws/src/bno086_imu_driver/config/bno086.yaml` を参照。

| パラメータ | 既定値 | 内容 |
|---|---|---|
| `port` | `/dev/ttyACM0` | シリアルデバイス。udev ルールを入れたら `/dev/bno086` |
| `frame_id` | `imu_link` | メッセージの frame_id |
| `imu_rate_hz` | `100.0` | IMU の報告レート (最大400程度) |
| `mag_rate_hz` | `25.0` | 地磁気の報告レート |
| `use_device_time` | `true` | 基板側クロックでタイムスタンプする |
| `publish_tf` | `false` | 姿勢を TF として配信する |
| `mount_yaw_deg` | `180.0` | U4 の実装角度の補正 ([座標系](#座標系)) |
| `auto_tare` | `all` | 起動・リセット時の自動原点合わせ (`off` / `yaw` / `all`) |
| `auto_tare_delay` | `2.0` | 自動 tare までの待ち時間 [s] |
| `use_game_rotation_vector` | `false` | 地磁気を使わない Game RV に切り替える |
| `angular_velocity_stddev` | `0.005` | 共分散に使う標準偏差 [rad/s] |
| `linear_acceleration_stddev` | `0.05` | 同 [m/s²] |

### 自動原点合わせ (auto_tare)

`auto_tare` を設定しておくと、**ストリームが立ち上がるたび**(ノード起動、S1 での
基板リセット、`reset` サービス) に、そのときの姿勢を原点にします。既定は `all` です。

| 値 | 動作 |
|---|---|
| `off` | センサ本来の基準のまま publish する |
| `yaw` | 方位だけゼロにする。roll / pitch は重力基準のまま |
| `all` | 3軸すべてゼロにして (0, 0, 0) から始める |

融合が落ち着くまで `auto_tare_delay` 秒待ってから実行します。

> YAML では `off` が真偽値に解釈されるため、コマンドラインでは
> `-p auto_tare:=off` のように文字列として渡しても動くようにしてあります。
> `use_game_rotation_vector: true` のときは tare が効かないため機能しません。

### 自動再接続

S1 (RESET) を押すと MCU が再起動して USB CDC が一度切れますが、ノードは
1 秒ごとに再接続を試み、復帰したらレート設定を送り直します。基板側の
マイクロ秒カウンタも 0 に戻るため、時刻推定もリセットしています。

```
[ERROR] serial error: read failed: socket disconnected
[INFO]  reconnected
[INFO]  requested 100 Hz IMU / 25 Hz magnetometer
```

実測で **約4秒で復帰** します (TCP 中継を挟んだ場合。中継側も基板が USB から
消えている間は待機して再オープンします)。

### サービス

**`tare` と `reset` は別物です。** 姿勢をゼロにしたいときは `tare` を使ってください。
`reset` は BNO086 を再起動するだけで、tare は解除され地磁気基準の方位に戻ります
(S1 ボタンも同じです)。

```bash
# 方位(yaw)だけをゼロにする ← ロボット用途はたいていこちら
# roll/pitch は重力基準のまま残ります
ros2 service call /bno086_imu_driver/tare_yaw std_srvs/srv/Trigger

# 3軸すべてをゼロにする
ros2 service call /bno086_imu_driver/tare std_srvs/srv/Trigger

# 動的キャリブレーション結果を保存
ros2 service call /bno086_imu_driver/save_calibration std_srvs/srv/Trigger

# BNO086 を再起動 (tare は解除される)
ros2 service call /bno086_imu_driver/reset std_srvs/srv/Trigger
```

### RViz2 で見る

Ubuntu ネイティブなら RViz2 がそのまま使えます。

```bash
sudo apt install ros-jazzy-rviz2 ros-jazzy-imu-tools

# 姿勢を TF として出す
ros2 launch bno086_imu_driver bno086.launch.py port:=/dev/bno086 \
  --ros-args -p publish_tf:=true

ros2 run rviz2 rviz2
```

Fixed Frame を `odom` にして TF を追加するか、`imu_tools` の
`rviz_imu_plugin` で `/imu/data` を直接表示できます。

### ブラウザで可視化

RViz2 を入れずに済ませたいとき用に、ブラウザで見られる可視化ノードもあります
(元は X サーバのない環境向けに書いたものです)。ROS 2 のトピックを購読して
いるので、「ROS 2 で動いていること」がそのまま確認できます。

```bash
ros2 run bno086_imu_driver imu_viz
# visualiser on http://localhost:8080
```

ブラウザで <http://localhost:8080> を開くと、姿勢の 3D 表示、roll/pitch/yaw、
角速度・加速度・地磁気の生値、受信数、診断メッセージが 30 fps で更新されます。

### ROS 抜きでの動作確認

基板の立ち上げ時はこちらが便利です。

```bash
ros2 run bno086_imu_driver serial_monitor --port /dev/bno086
# 100 Hz imu /  25 Hz mag  rpy=(  +0.31,  -1.02, +45.77)deg  acc=3 resets=0 ...

# 全フレームをダンプ
ros2 run bno086_imu_driver serial_monitor --port /dev/bno086 --raw
```

## 5. キャリブレーション

BNO086 は動的キャリブレーションを行うので、初回は基板を各軸まわりに
ゆっくり回してください。`/diagnostics` の `orientation_accuracy` が
`high` になったら `save_calibration` サービスで保存します。

`/diagnostics` が `unreliable` のままの場合、地磁気が乱れた場所
(モータやスチール机の近く) が原因のことが多いです。

## トラブルシューティング

まず [`serial_monitor`](#ros-抜きでの動作確認) を実行してください。ROS を挟まずに
基板と直接話すので、問題が基板側かホスト側かを切り分けられます。

| 症状 | 原因と対処 |
|---|---|
| ポートが見つからない | `ls /dev/ttyACM*` で確認。何も出なければ USB ケーブルが充電専用の可能性があります (データ線のあるものを使う)。`dmesg \| tail` に `cdc_acm` が出ているかも見てください |
| `/dev/bno086` ができない | udev ルールを入れたあと、**USB を挿し直す**必要があります。`udevadm info -a -n /dev/ttyACM0 \| grep idVendor` が `0483` かも確認 |
| `Permission denied` | udev ルールを入れる ([シリアルポートの権限](#シリアルポートの権限))。`dialout` グループに入れた場合は再ログインが必要 |
| LED1 が点滅しない | ファームウェアが動いていません。書き込みからやり直してください |
| LED1 は点滅するが LED2 が消灯 | BNO086 と通信できていません。**JP1 / JP2 がオープンか**を確認 (SPI 選択には PS1=1 / PS0=1 が必要)。`serial_monitor` が到達段階と SPI モード自動走査の結果を表示します |
| トピックが出ない / `topic hz` が無反応 | `ros2 node list` にノードがいるか確認。いるのにデータが来ない場合はログの `serial error` を確認 |
| `rosdep: command not found` | `sudo apt install python3-rosdep`、`sudo rosdep init`、`rosdep update` |
| `ModuleNotFoundError: serial` | `sudo apt install python3-serial` |
| `ModuleNotFoundError: bno086_imu_driver` | `source install/setup.bash` を忘れています。テストを直接動かす場合は `ros2_ws/src/bno086_imu_driver` に移動してから実行してください |
| yaw がリセットで 0 に戻らない | 仕様です ([理由](#yaw-がリセットで-0-に戻らない理由))。`auto_tare` か `tare_yaw` を使ってください |
| 姿勢が実機と逆に動く | `mount_yaw_deg` が `0.0` になっていないか確認 ([座標系](#座標系)) |
| `/diagnostics` が `unreliable` のまま | 地磁気の乱れです。場所を変えるか `use_game_rotation_vector: true` |
| 100 Hz より遅い / 取りこぼす | 200 Hz 超は SHTP が飽和します ([実測レート](#実測レート)) |

## テスト

**52件、実機不要**です。擬似端末 (pty) を使って、フレーム生成 → pyserial →
パーサ → ROS メッセージまでを通しで検証しています。

```bash
source /opt/ros/jazzy/setup.bash
cd ros2_ws/src/bno086_imu_driver && python3 -m pytest test -q
```

push / PR ごとに [GitHub Actions](.github/workflows/ci.yml) が、
ファームウェアのビルド (USB / UART の両構成) とこのテストを実行します。

## 実機での検証状況

> **検証はすべて macOS + Docker + TCP 中継の環境で行いました。**
> ROS 2 の層より下 (ファームウェア、SPI/SHTP、フレーム、レート、tare) は
> 環境に依存しないのでそのまま通用しますが、**Ubuntu ネイティブでの実機確認は
> まだです**。udev ルールと `/dev/ttyACM0` 直結の部分は初回に確認してください。

### 動作確認済み (実機)

| 項目 | 結果 |
|---|---|
| USB CDC 列挙 | `/dev/cu.usbmodem*` として認識。48 MHz クロック正常 |
| DFU 書き込み | STM32CubeProgrammer でベリファイまで成功 |
| BNO086 SPI 通信 | 段階4 (reset complete)、リセット試行1回で確立 |
| クォータニオン | \|q\| = 1.0000 |
| 加速度 | \|a\| = 9.6 m/s² (重力と一致) |
| 地磁気 | 25 Hz で受信 |
| 通信品質 | 長時間受信で CRC エラー 0 / 再同期 0 |
| レート制御 | `SET_RATE` が反映される |
| ROS 2 ノード | macOS の Docker + TCP 中継で `/imu/data` を 100.2 Hz で publish |
| tare (3軸) | 0.3 秒で姿勢が (0,0,0) になることを確認 |
| tare (yawのみ) | yaw だけゼロ化、roll/pitch の変化 0.03° |
| 姿勢の安定性 | 静置 45 秒でドリフト −0.45 °/分、精度 high |
| 地磁気 | 静置時 \|B\| = 38.1 µT (地磁気の正常範囲) |
| 自動再接続 | 切断から約 4 秒で復帰 |
| 自動 tare | S1 リセット後、再接続して自動で (0,0,0) に復帰 |
| 実装角度の補正 | `mount_yaw_deg: 180.0` 適用後、加速度 \|a\| = 9.62 m/s² / 100.1 Hz |
| クォータニオンの向き | body → world であることを確認 (誤差 0.128 / 逆向き 1.247) |
| 可視化 | ブラウザ表示が実機の回転と一致 |

### 実測レート

`SET_RATE` を変えながら、姿勢・角速度・加速度が 1 フレームに揃っている割合を
測定した結果です。

| 要求レート | 実測 | 全項目揃い |
|---|---|---|
| 100 Hz | 100 Hz | **100 %** |
| 200 Hz | 192 Hz | **99.7 %** |
| 250 Hz | 150 Hz | 92.9 % |
| 400 Hz | 189 Hz | 46 % |

**実用上の上限は 200 Hz 程度** です。既定値の 100 Hz なら常に全項目が揃います
(ロボティクス用途では 100 Hz あれば十分なことがほとんどです)。

200 Hz を超えると SHTP パケットの取得が **約 800 パケット/秒で飽和** します。
地磁気を無効にしても変わらないため、BNO086 のセンサ帯域ではなくファームウェア側
の取得経路 (1 ループ 1 パケット、CS ごとのオーバーヘッド、`board_micros()` の
ソフトウェア除算) が律速していると考えられます。**根本原因は未特定** です。
400 Hz が必要な場合は、1 ループで複数パケットを読む、SysTick 由来の除算を除く、
といった最適化が要ります。

### 立ち上げで踏んだ問題 (すべて解決済み)

1. **PS0 ストラップの誤り** — SPI 選択は `PS1=1, PS0=1` で、**両方が NRST 解除前から
   最初の H_INTN アサートまで High** である必要があります (データシート 1.2.4 /
   Figure 1-20)。当初 PS0 を Low にしていたため UART-RVC で起動し、SPI に一切
   応答しませんでした。
2. **SPI 待ちループのタイムアウト欠如** — SPI が応答しないとメインループが停止し、
   USB は列挙されたままデータだけ止まる状態になっていました。
3. **不正な SHTP ヘッダの受理** — `len=17792` のようなノイズを真に受けて大量に
   クロックを出していました。チャンネルと長さを検証するようにしました。

立ち上げ用の診断機能は残してあります。IMU が起動しない場合、`serial_monitor` が
到達段階・SPI モード自動走査 (4モード × 2クロック)・MISO/H_INTN のピンレベル判定を
表示します。

### yaw がリセットで 0 に戻らない理由

Rotation Vector は**地磁気で北を基準にした絶対方位**です。電源投入時からの相対角
ではないため、リセットしても同じ物理的方位を測り直すだけで 0 にはなりません。
0 にしたいときは `tare` / `tare_yaw` を使ってください。

`use_game_rotation_vector: true` にすると地磁気を使わない Game Rotation Vector に
切り替わります。磁気外乱に強い一方、**tare が効きません**(下記)。屋内静置での
実測では磁気RV/Game RV ともドリフトは 0.5 °/分以下、精度は high でした。
モーターや鉄骨で地磁気が乱れる環境でなければ、既定の磁気RV のままで問題ありません。

> 測定は基板を静置して行ってください。持ち運びながらの測定では地磁気が
> 数百 µT に振れ、姿勢精度も 0 のままになります。

### そのほか未検証

- **Save DCD (`0x06`)** — CEVA の SH-2 リファレンスに基づく実装で、実機応答は
  未確認です。
- **Game Rotation Vector では tare が効きません** — `use_game_rotation_vector: true`
  のとき、tare を送っても yaw が変化しません (実測 +172.12 → +172.14)。SH-2 の
  tare basis を Gaming Rotation Vector (1) に切り替えても改善しませんでした。
  原因は未特定です。tare が必要な場合は既定の磁気RV をお使いください。
- **tare の永続化が効きません** — `tare` / `tare_yaw` は姿勢を確実にゼロにしますが
  (実機確認済み)、「Persist Tare」(サブコマンド 1) が効いておらず、IMU をリセット
  すると tare が失われて地磁気基準の方位に戻ります。**起動のたびに tare し直す
  運用**でご利用ください。既定の `auto_tare: all` がこれを自動で行います。

  原因は未特定です。BNO086 は Tare コマンドに対して SH-2 コマンドレスポンスを
  返さないため (実機で 0 件)、受理されたか否かを応答から判定できませんでした。
  tare 本体は効いているので、コマンド自体は届いています。
- **CAN** — 回路はありますがファームウェアは未実装です。加えて U5 の `Vio` が
  +5 V に配線されている点は要確認です ([docs/HARDWARE.md](docs/HARDWARE.md))。

### 実装を見送ったもの

動作中のファームから ROM ブートローダへジャンプする `ENTER_DFU` コマンドを
試しましたが、この石では BOOT0 が Low だとブートローダがユーザーコードへ
戻ってしまい DFU として列挙しませんでした。削除済みです。書き込みのたびに
S2 (BOOT) + S1 (RESET) のボタン操作が必要です。

また `PCB/BNO086_ROS2Board.ioc` は基板と一致していません (割り込みが `PF11` に
なっている等)。本ファームウェアは CubeMX 生成に依存しないため実害はありませんが、
CubeMX を使う場合は先に修正してください。

## 付録: macOS で動かす

**開発中の検証に使った構成**です。常用するなら Ubuntu ネイティブをおすすめします。

macOS には ROS 2 をネイティブに入れられないのでコンテナを使いますが、
Docker Desktop for Mac は USB デバイスをコンテナに渡せません。そこで
**シリアルを TCP で中継** します。pyserial の `socket://` URL に対応済みなので、
ドライバ側は URL を渡すだけです。

ターミナル1 (macOS ホスト側):

```bash
pip3 install pyserial
ls /dev/cu.usbmodem*                  # 基板が見えているか確認

python3 tools/serial_bridge.py --port '/dev/cu.usbmodem*' --listen 5555
# [bridge] opened /dev/cu.usbmodem205B386A48581
# [bridge] listening on 0.0.0.0:5555
```

ターミナル2 (コンテナ内):

```bash
cd docker
docker compose build                  # 初回のみ。10分程度かかります
docker compose run --rm --service-ports ros2

colcon build --symlink-install && source install/setup.bash
ros2 launch bno086_imu_driver bno086.launch.py \
    port:=socket://host.docker.internal:5555
```

`--service-ports` はブラウザ可視化 (8080番) を使うときに必要です。
`ros2_ws/` はホストからマウントされているので、ホスト側のエディタでの編集が
そのまま反映されます。

これで `/imu/data` が 100 Hz で流れます (実機で確認済み)。

同じ仕組みで、**別マシンに挿した基板をネットワーク越しに読む**こともできます
(`--bind` と `socket://<ホスト>:5555`)。Ubuntu 機からノートPCに挿した基板を
読む、といった使い方ができます。

うまく繋がらないときは、中継が動いているか (`[bridge] listening on ...`)、
`--service-ports` を付けたか、`host.docker.internal` が引けるか
(`docker-compose.yml` の `extra_hosts`) を確認してください。

Linux ホストで同じコンテナを使う場合は中継は不要で、`docker-compose.yml` の
`devices:` を有効にして `port:=/dev/ttyACM0` を直接渡せます。

> ROS 2 を macOS にネイティブで入れたい場合は
> [RoboStack](https://robostack.github.io/) (conda-forge 版) が使えます。
> その場合は中継なしで `/dev/cu.usbmodem*` を直接指定できます。

## ライセンス

[MIT](LICENSE)。ただし `firmware/` がビルド時に取得する STMicroelectronics の
ライブラリはそれぞれのライセンス (Apache-2.0 / BSD-3-Clause 等) に従います。

PCB データ (`PCB/`) も同じ MIT で公開しています。基板を起こす場合は
[docs/HARDWARE.md](docs/HARDWARE.md) の既知の注意点 (U5 `Vio` の配線、`.ioc` の
不一致) を先に確認してください。
