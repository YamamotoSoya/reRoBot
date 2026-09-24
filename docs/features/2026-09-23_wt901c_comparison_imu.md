<!-- claude: 2026-09-23 作成 -->
# 比較用 IMU WITmotion WT901C-TTL を BNO086 と同時に bag へ記録する手順

**ステータス: 手順書 (2026-09-23)。準備ファイル作成済み。§2 のセンサ設定は 2026-09-23 に Windows 公式ソフト(新 UI、自動保存) で 115200 / 200 Hz / 帯域 98 Hz に変更し、電源断後も生フレーム計測で 4 種 ≈201 Hz・加速度フレームの連続同値 3% を確認 (保存済み・帯域有効)。§3 (submodule 追加・Qt5 apt・ビルド) と §4 (単体起動 200.5 Hz) も同日完了。**09-24: 機体搭載 → 静置・傾け・旋回で BNO086 と同時比較 (§5 の同時起動を単体ドライバ 2 本で実施)。WT901C は roll=π 取付、xyz (−0.065, 0, 0.57746) で `imu_wit_link` を URDF 4 ファイルに追加。副産物として BNO086 の auto_tare 問題・上下逆取付・accel バイアスを発見 (`docs/issue/2026-09-24_bno086_auto_tare_rotates_motion_outputs.md`)。§6〜7 (走行 bag・比較解析) は未実施。**

## 目的

GLIM の z ドリフト・roll 歪み調査 (`docs/issue/2026-09-10_glim_z_drift_not_vangle.md`) で
「BNO086 の出力そのものが怪しくないか」を切り分けるため、別メーカの IMU (WITmotion
WT901C-TTL, 9 軸・姿勢出力あり) を **同じ車体に同時搭載**し、同一走行の bag に
`/imu/data` (BNO086) と `/imu_wit/data` (WT901C) を並べて残す。

## 現状確認結果 (2026-09-23、ホスト実測)

| 項目 | 結果 |
|---|---|
| WT901C の USB-TTL 変換 | Prolific PL2303 `067b:23a3` → `/dev/ttyUSB0` (`/dev/serial/by-id/usb-Prolific_..._BOCKb2A9708-if00-port0`) |
| センサ現設定 | **9600 baud で 0x55 0x51/52/53/54 フレーム受信を確認** (出荷設定 = 9600 / 10 Hz / 帯域 20 Hz)。115200 では無応答 |
| ホストユーザ | `dialout` 未所属 → ホストで直接読むには sudo。コンテナは privileged (root) で問題なし |
| コンテナ | WITmotion 系 apt パッケージ無し、pip も無し、pyserial 3.5 あり、Qt5 SerialPort は apt 候補あり (5.15.13) |
| BNO086 | 本日は未接続 (`/dev/ttyACM*` なし)。udev rule 導入済みなら `/dev/bno086` |

## 構成

```
車体
├── BNO086 (自作基板, USB CDC)  ──/dev/ttyACM0 (or /dev/bno086)──▶ bno086_imu_driver ──▶ /imu/data      (frame imu_link,     100〜200 Hz, デバイス時計→ROS 時刻)
└── WT901C-TTL (比較用)         ──USB-TTL /dev/ttyUSB-wt901 115200──▶ witmotion_ros    ──▶ /imu_wit/data  (frame imu_wit_link, 200 Hz,       受信時刻 stamp)
                                                                                        ──▶ /imu_wit/mag
                                       ros2 bag record ◀── 両 topic + LiDAR + odom + tf
```

- ドライバは **ElettraSciComp/witmotion_IMU_ros (ros2 branch)** を採用。C++、Qt5 SerialPort 依存。
  ROS 2 用に整備されている唯一の WITmotion ドライバで、Humble/Jazzy の動作報告 issue あり。
  Python 製の代替 (Ericsii/ros_wit_imu_node 等) はセンサ設定機能も同じく無く、
  メンテ状況で劣るため採用しない。
- 準備済みファイル (本手順のために 2026-09-23 追加):

| ファイル | 役割 |
|---|---|
| `tools/99-wt901.rules` | `/dev/ttyUSB-wt901` の安定名 |
| `rerobot_bringup/config/wt901.yaml` | ドライバ設定 (topic `/imu_wit/data`, frame `imu_wit_link`, `use_native_orientation: false`) |
| `rerobot_bringup/launch/wt901_imu.launch.py` | ドライバ起動 launch |
| `docker/Dockerfile_main` | `qtbase5-dev libqt5serialport5-dev` 追加 (再ビルド時に反映) |

## 手順

### 0. 物理取付

- WT901C は **BNO086 の直近に、軸を揃えて**固定する (筐体印字: X 前・Y 左・Z 上、右手系)。
  軸が揃っていれば gyro / accel の生値を回転変換なしで直接重ねられる。
- 取付位置 (base_link 基準) を実測してメモする。URDF への `imu_wit_link` 追加は
  bag 記録には不要 (tf_static に無いだけ)。GLIM を WT901C で回す段で `T_lidar_imu` に使う。
- USB-TTL 変換は車載 PC の USB に直挿し (ハブ経由はレイテンシとフレーム落ちの原因)。

### 1. udev rule (ホスト、1 回だけ)

```bash
sudo cp tools/99-wt901.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
ls -l /dev/ttyUSB-wt901      # → ttyUSB0 への symlink
```

### 2. センサ設定を 115200 / 200 Hz / 帯域 ≈100 Hz に書き換える (1 回だけ、不揮発) — **実施済み 2026-09-23**

出荷設定 9600 baud では 1 セット (acc+gyro+angle+mag = 44 byte) が最大 ≈21 セット/s しか
通らず、レートだけ上げても本体側で自動的に間引かれる。**baud も同時に上げる**。
帯域 (内部フィルタ) は出力レート未満だと同じ値が繰り返し出る (マニュアル 2.4.8:
帯域 20 Hz + 出力 100 Hz で 5 回同値) ので ≈100 Hz に上げる。

Windows 公式ソフト (WitMotion PC Software, https://www.wit-motion.com/file.html の MiniIMU.exe) で行う。
変換器は Prolific PL2303GC なので Windows Update でドライバが入る。

1. Port = COMx, Baud = 9600 (出荷値) で Open → Data タブの数値が動くことを確認。
2. Config → Read Config。**Output Rate = 200Hz** → **Band Width = 98Hz** (以上) → 最後に **Baud Rate = 115200**
   (ボーレートは変えた瞬間に通信が切れるので最後)。新 UI は Save ボタンが無く選択時に自動保存。
3. Close → Baud 115200 で Open → 数値が動くことを確認。
4. **USB を抜き差し (電源断) → 115200 で再度 Open できれば保存成功。** Read Config でレート・帯域も保持を確認。
5. Calibrate 欄 (加速度校正・磁気校正・Z 軸リセット) は触らない。Algorithm は既定 9-axis (yaw の磁気飛びが
   気になれば 6-axis)。Gyro Auto Calibrate は既定 ON (バイアス比較をしたいなら OFF)。

Linux 側での確認はドライバ起動後に `ros2 topic hz /imu_wit/data` が ≈200 Hz であること (§4)。
実施結果 (2026-09-23): 電源断後に 115200 で 4 種フレーム各 ≈201 Hz、加速度フレームの連続同値 3%
(帯域 20 Hz のままなら約 90%) を生フレーム計測で確認。

### 3. ドライバをワークスペースに追加してビルド — **実施済み 2026-09-23** (submodule = `d1ab57d`、Qt5 はコンテナに apt 済み、`witmotion_ros` ビルド 41 s 警告のみ)

```bash
# ホスト: submodule 追加 (他ドライバと同じ src/drivers/ 直下)
git submodule add -b ros2 https://github.com/ElettraSciComp/witmotion_IMU_ros.git \
    ros2_ws_main/src/drivers/witmotion_ros
git submodule update --init --recursive ros2_ws_main/src/drivers/witmotion_ros
ls ros2_ws_main/src/drivers/witmotion_ros/witmotion-uart-qt   # 空なら recursive 失敗 → README 参照
```

⚠️ ディレクトリ名は **`witmotion_ros` 固定** (CMakeLists がパッケージ名と一致を前提)。
内部の `witmotion-uart-qt` も名前固定。

ビルド依存 Qt5 は Dockerfile_main に追記済みだが、イメージ再ビルド (`./scripts/build.sh images`)
はこの PC では重い。まず稼働中コンテナに直接入れて試し、動いたら後日イメージを再ビルドする:

```bash
docker exec -it rerobot_env bash -c 'apt-get update && apt-get install -y qtbase5-dev libqt5serialport5-dev'
./scripts/build.sh main          # colcon は 1 パッケージずつ (既定)。witmotion_ros も含めて全体ビルド
```

(コンテナを `docker compose down` で作り直すと apt 分は消える → その時はイメージ再ビルドが必要)

### 4. 起動と単体確認 (BNO086 なしで先に) — **実施済み 2026-09-23**: `port:=ttyUSB0` で 200.5 Hz、静置 accel (0.25, 1.31, 9.75)、stamp 間隔 0〜10 ms (5 ms polling 由来)。⚠️ Ctrl-C 終了時に `terminate called after throwing 'std::system_error'` で abort するが終了時のみで運用に影響なし

```bash
docker exec -it rerobot_env bash -c \
  'source /workspace/install/setup.bash && ros2 launch rerobot_bringup wt901_imu.launch.py'
# 別端末
docker exec -it rerobot_env bash -c 'source /opt/ros/jazzy/setup.bash && ros2 topic hz /imu_wit/data'
docker exec -it rerobot_env bash -c 'source /opt/ros/jazzy/setup.bash && ros2 topic echo --once /imu_wit/data'
```

確認点:
- `hz` が ≈200 Hz。**topic があるのに 1 件も来ない**場合は `wt901.yaml` の
  `use_native_orientation` が true になっていないか (WT901C は quaternion を出さないので
  true だとドライバが永遠に publish しない)。
- 静置で `linear_acceleration.z ≈ +9.8`、`angular_velocity` ≈ 0。BNO086 と同様に z が
  +9.8 なら軸の向きは URDF の imu_link と同じ (正立)。
- `header.stamp` の間隔が 5 ms 刻みで揃っていること (polling_interval=5)。

### 5. 両 IMU 同時起動

```bash
# BNO086 込みの通常 bringup (LIO 用に IMU 200 Hz)
IMU=true IMU_RATE=200 ./scripts/bringup3d.sh        # 3D のみ。2D+3D なら実体 launch を直接:
# docker exec -d rerobot_env bash -c 'source /workspace/install/setup.bash && ros2 launch rerobot_bringup rerobot_bringup.launch.py lidar_2d:=true lidar_3d:=true imu:=true imu_rate:=200 ekf:=false'
# WT901C
docker exec -d rerobot_env bash -c \
  'source /workspace/install/setup.bash && ros2 launch rerobot_bringup wt901_imu.launch.py > /workspace/log/wt901.log 2>&1'
# 両方見えるか
docker exec -it rerobot_env bash -c 'source /opt/ros/jazzy/setup.bash && ros2 topic hz /imu/data & ros2 topic hz /imu_wit/data; wait'
```

BNO086 の USB CDC (`ttyACM0`) と WT901C の `ttyUSB-wt901` は別デバイスなので競合しない。
`/imu/data` を出すのは BNO086 だけ (realsense_imu.launch.py は同時に起こさない)。

### 6. bag 記録

`docs/manual/05_bag_recording.md` の標準 topic 群に `/imu_wit/data` `/imu_wit/mag` を足す。
センサ構成ディレクトリ名は `bags/README.md` の規約に従い IMU 2 個を明示する
(例: `2d3d_imu_wit`)。

```bash
docker exec -it rerobot_env bash -c 'source /opt/ros/jazzy/setup.bash && \
  ros2 bag record -s mcap -o /workspace/bags/5goukan/2d3d_imu_wit/online/rosbag/$(date +%F_%H%M) \
    /rfans_driver/rfans_points /scan /imu/data /imu_wit/data /imu_wit/mag \
    /odom /tf /tf_static /diagnostics /robot_speed_cmd'
```

記録前後のチェック:
- 走行前に **両 IMU を 30 s 以上静置**した区間を入れる (バイアス・|g| スケール比較用。
  BNO086 の |g| +3% 問題 = `acc_scale: 0.9705` の妥当性を WT901C で裏取りできる)。
- 記録後 `ros2 bag info` で両 IMU の message count が「秒数 × 200」に近いか確認。
  WT901C 側だけ少なければ USB-TTL のフレーム落ち (帯域 94 Hz は関係なし、baud か USB 経路)。
- bag 中の時計ステップに注意 (`ntp-clock-step`: WiFi 再接続後の timesyncd 同期で全 topic に
  同時の穴)。走行中は `sudo timedatectl set-ntp false` を検討。

### 7. 比較の見方 (最低限)

| 見るもの | 方法 | 期待 |
|---|---|---|
| gyro_z の一致 | 両 topic を CSV 化して重ね描き。車輪 odom の yaw rate も重ねる | 同符号・同振幅。ズレは時刻オフセット (WT901C は受信時刻 stamp なので数 ms 遅れる) |
| 静置 accel の \|g\| | 30 s 静置区間の平均ノルム | BNO086 ≈ 10.1 (+3%) vs WT901C ≈ 9.8 なら BNO086 側スケール問題が確定 |
| roll/pitch の静置値 | orientation → RPY | 取付面が同一なら両者一致。差 = 取付ズレ or BNO086 較正 |
| GLIM への入力差替え | `config_ros.json` の `imu_topic` を `/imu_wit/data`、`imu_frame_id` を `imu_wit_link`、`T_lidar_imu` を WT901C の取付から再計算、`acc_scale` を 0 に戻して offline 実行 | z ドリフト A が変わらなければ IMU 由来説はさらに弱まる (09-20 の結論と整合) |

## 落とし穴まとめ

1. **topic が空**: `use_native_orientation: true` (上流既定 yml が true)。本リポの yaml は false 済み。
2. **10 Hz しか出ない**: センサ設定を書き換えていない / baud だけ上げてレートを上げていない (§2)。
3. **stamp が 50 ms 刻みで団子**: `polling_interval` が既定 50 ms。本リポは 5 ms。
4. **同じ値が数回連続**: 帯域 (BANDWIDTH) が出力レート未満。94 Hz 以上にする。
5. **`/dev/ttyUSB-wt901` が無い**: udev 未導入 or 別の PL2303 が先に挿さっている。`ls /dev/serial/by-id/`。
6. **ビルドで witmotion-uart-qt が見つからない**: `--recursive` 漏れ。手動で `ElettraSciComp/witmotion_IMU_QT` を `witmotion_ros/witmotion-uart-qt` に clone。
7. **コンテナ再作成後にビルドが落ちる**: apt で入れた Qt5 が消えた。`./scripts/build.sh images` (main のみ 1 本) で Dockerfile 版に。

## 参考

- WITmotion 公式 SDK レジスタ定義: `WitStandardProtocol_JY901/Arduino/Arduino_sdk/REG.h`
  (RRATE 0x03: 200 Hz=0x0B, BAUD 0x04: 115200=6, BANDWIDTH 0x1F: 94 Hz=2, KEY 0x69=0xB588, SAVE 0x00)
- WT901C TTL manual v0513 §2.4.2 (出力レート既定 10 Hz・最大 200 Hz、高レートは 115200 推奨) / §2.4.8 (帯域と繰り返し値)
- ElettraSciComp/witmotion_IMU_ros ros2 branch README、issue #42 #48 #49 (topic 空)
