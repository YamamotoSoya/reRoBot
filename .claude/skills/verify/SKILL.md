---
name: verify
description: Docker コンテナ rerobot_env 内でビルド・スタック起動・動作検証を行う。実機 can0 があれば実機モード、無ければ vcan0 + canopen-fake-slaves のハード無しモードで検証する。
---

<!-- claude: verify スキル定義。Claude 作成。手順は 2026-07-12 にコンテナ内で実地確認済み。-->

# Verify

コード変更後に「実際に動くか」をコンテナ内で確認する。ビルド → スタック起動 →
トピック/CAN フレーム観測まで。実機が無くても vcan モードでコマンド経路
(Twist → IK → RPDO) をエンドツーエンドで検証できる。

## いつ使うか

- epos4_controller / epos4_odometry / bringup / bus config を変更した後のコミット前
- ユーザが「動作確認して」「検証して」等と要求したとき
- ハードが手元に無いが制御経路の変更を確かめたいとき (vcan モード)

## 共通手順

1. **コンテナ確認**: `docker ps --filter name=rerobot_env`。停止中なら
   `docker compose up -d main` (リポジトリルートで)。
   ※ 2026-07-26 再編後: main コンテナの ws は `ros2_ws_main/` (mount 先は従来どおり
   `/workspace/src`)。slam_toolbox は別コンテナ `slamtoolbox_env` (profile: slamtoolbox)。

2. **ビルド**:
   ```sh
   docker exec rerobot_env bash -c "source /opt/ros/jazzy/setup.bash && cd /workspace && colcon build --symlink-install --executor sequential"
   ```
   並列ビルドは canopen で壊れるため `--executor sequential` 必須。失敗したら
   `docs/report/2026-07-11_colcon_stale_volume_symlink_conflict.md` (stale volume) を
   先に疑う。

   ※ 非対話 bash は `.bashrc` を読まないため、以降すべてのコマンドで
   `source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash &&
   export ROS_DOMAIN_ID=150` を明示する。

3. **モード判定**: `docker exec rerobot_env ip link show can0` — UP なら実機モード、
   「does not exist」なら vcan モードへ。

## 実機モード (can0 + EPOS4)

4. `ros2 launch rerobot_bringup rerobot_bringup_2d.launch.py` をバックグラウンド起動し、
   5 s TimerAction + init 完了 (~15 s) を待つ。`init` の「Homing failed」は CSV では
   期待どおり (CLAUDE.md 参照)。

5. 確認項目: `ros2 node list` / `/motor{1,2}/cia402_device_{1,2}/joint_states` と
   `/odom` の publish / `candump -n 20 can0` で PDO 疎通 / statusword は
   controller のログ (`statusword=0x0027 mode=9` が正常) か SDO read で確認。

6. **モータを回す検証はユーザ立ち会いのみ**。Claude が無人で `/robot_speed_cmd` に
   Twist を流すのは禁止 (実機が走り出す)。回す場合は「浮かせて確認 → 接地」の順
   (PROJECT_STATE §9)。

## vcan モード (ハード無し・実地確認済み)

fake slave (`CIA402MockSlave`) は homing / profile position / profile velocity /
interpolated position / cyclic **position** のみ実装で、**CSV (cyclic velocity) が無い**。
そのため合格基準が実機と異なる (手順 11 参照)。

4. **vcan0 作成** (コンテナ内、privileged + host network なので可):
   ```sh
   docker exec rerobot_env bash -c "ip link add dev vcan0 type vcan; ip link set up vcan0; ip link show vcan0"
   ```
   `ip` が無ければ `apt-get install -y iproute2` (Dockerfile には追加済み)。

5. **fake slave 用 bus config を生成**。EPOS4 用 master.dcf は Vendor-ID チェック
   (0x1F85) を含むため fake slave では boot が
   「Value of object 1018:01 ... different to ... (Vendor-ID)」で失敗する。
   fake slave の EDS から作り直す:
   ```sh
   docker exec rerobot_env bash -c 'source /opt/ros/jazzy/setup.bash && mkdir -p /tmp/vcan_test && cp /opt/ros/jazzy/share/canopen_fake_slaves/config/cia402_slave.eds /tmp/vcan_test/ && sed -e "s|dcf_path:.*|dcf_path: \"/tmp/vcan_test\"|" -e "s|maxon_epos4_0x1018.eds|cia402_slave.eds|g" /workspace/install/maxon_epos4_ros2/share/maxon_epos4_ros2/config/bus_config_cia402_epos4_vel/bus.yml > /tmp/vcan_test/bus.yml && cd /tmp/vcan_test && dcfgen -r -d /tmp/vcan_test bus.yml'
   ```
   (install 済み bus.yml は `@BUS_CONFIG_PATH@` が展開済みなので `dcf_path:` 行ごと
   置換する。`dcfgen` は `/opt/ros/jazzy/bin` — source 後に使える)

6. **fake slave ×2 を起動** (それぞれバックグラウンド):
   ```sh
   docker exec rerobot_env bash -c "source /opt/ros/jazzy/setup.bash && export ROS_DOMAIN_ID=150 && export LD_LIBRARY_PATH=/opt/ros/jazzy/lib/canopen_fake_slaves:\$LD_LIBRARY_PATH && ros2 launch canopen_fake_slaves cia402_slave.launch.py node_id:=1 node_name:=fake_cia402_slave_1 can_interface_name:=vcan0"
   ```
   node_id:=2 / node_name:=fake_cia402_slave_2 でもう 1 台。
   - `LD_LIBRARY_PATH` 追加は必須 (`libmotion_generator.so` が非標準ディレクトリに
     ありローダが見つけられない Jazzy バイナリのパッケージング不備)。
   - 起動時の「Failed to make transition 'TRANSITION_CONFIGURE'」エラーは launch 内の
     二重 configure による**無害なもの**。成功判定は「Now sending Boot-Up frame...」。

7. **device container を起動** (バックグラウンド):
   ```sh
   docker exec rerobot_env bash -c "source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash && export ROS_DOMAIN_ID=150 && ros2 launch canopen_core canopen.launch.py master_config:=/tmp/vcan_test/master.dcf bus_config:=/tmp/vcan_test/bus.yml can_interface_name:=vcan0"
   ```
   `master_bin:=""` を渡すと「malformed launch argument」— **引数ごと省略**する。
   成功判定: 「Driver booted and ready.」、
   `/motor{1,2}/cia402_device_{1,2}/joint_states` が publish される。

8. **アプリ層を起動** (それぞれバックグラウンド):
   ```sh
   ros2 run epos4_controller epos4_controller --ros-args --params-file /workspace/src/bringup/rerobot_bringup/config/params_2d.yaml
   ros2 run epos4_controller epos4_odometry   --ros-args --params-file /workspace/src/bringup/rerobot_bringup/config/params_2d.yaml
   ```

9. **コマンド経路を検証**: Twist を流して CAN フレームを観測:
   ```sh
   (timeout 8 ros2 topic pub -r 10 /robot_speed_cmd geometry_msgs/msg/Twist '{linear: {x: 0.1}}' &) ; sleep 3; candump -n 12 -T 3000 vcan0
   ```
   合格 = cob-id `201`/`202` (RPDO1) の 7 byte 中、byte 2-5 の int32 (target velocity)
   が非ゼロ。実測例: linear.x=0.1 → `F1 FF FF FF` = **-15** (≒ 車輪 12.7 rpm ×
   gear 1.25、`invert: true` で負)。値はモータ軸 rpm スケールで載る。

10. **/odom の publish を確認**: `ros2 topic echo /odom --once`。
    fake slave は CSV 非対応で動かないため**値は 0 のままで正常**。

11. **合格基準 (vcan モード)**:
    | 項目 | 合格 |
    |------|------|
    | colcon build | 成功 |
    | 両 slave boot | 「Driver booted and ready.」/ joint_states publish |
    | controller 初期化 | ログに `statusword=0x0027` (op_enabled=1) |
    | CSV 遷移 | **「FAILED to reach CSV」が出るのが期待どおり** (mock に CSV が無い) |
    | コマンド経路 | RPDO1 に非ゼロ target velocity |
    | /odom | publish される (値 0 で正常) |

12. **結果を表で報告**する: `| 項目 | 期待 | 実測 | 判定 |`。

## 後始末 (必ず実行)

```sh
docker exec rerobot_env bash -c "pkill -9 -f 'epos4_[c]ontroller'; pkill -9 -f 'epos4_[o]dometry'; pkill -9 -f 'cia402_slave_[n]ode'; pkill -9 -f 'cia402_slave.launch.[p]y'; pkill -9 -f 'device_container_[n]ode'; ip link del vcan0 2>/dev/null; true"
```

- パターン中の `[x]` は必須: `pkill -f` は **docker exec 自身のコマンドライン
  (引数の文字列) にもマッチして自殺する** (実際に踏んだ)。
- `device_container_node` は SIGTERM を無視することがあるため `-9`。

## 守るべきこと

- **実機モードで無人の Twist publish は禁止**。ロボットが走る。
- CiA402 遷移サービスを手動で並行発行しない (PROJECT_STATE §7-1)。
- joint_states.velocity は常に 0 (bus.yml で 0x606C 未マップ)。回転判定は position 差分。
- ホストのサスペンド明けは DDS participant が壊れ、プロセスは生きているのに
  トピックが見えなくなることがある (実際に観測)。`ros2 daemon stop` で直らなければ
  チェーン全体を再起動する。
- 検証結果は実測のみ報告。**観測していない値・判定を捏造しない**。未実施項目は
  「未実施」と書く。
