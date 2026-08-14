# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

`reRoBot` is an autonomous navigation robot built for the Tsukuba Challenge. The codebase is a ROS 2 (Jazzy) workspace whose primary purpose is to drive a differential-drive base via two maxon EPOS4 motor controllers connected over CAN bus, using `ros2_canopen` (CiA 402 profile).

## Environment & Build

The workflow runs across **function-specific Docker containers** (2026-07-26 再編)。分割基準は「依存の壁」— apt で衝突しないものは main に同居させる (Nav2 は main 統合)。Running on the host directly is not the supported path.

| コンテナ | イメージ | workspace | 役割 |
|----------|---------|-----------|------|
| `rerobot_env` (main) | `docker/Dockerfile_main` | `ros2_ws_main/` | CAN モータ制御 + LiDAR/RealSense ドライバ + odometry + **Nav2** + teleop。常用 (profile なし) |
| `slamtoolbox_env` | `docker/Dockerfile_slamtoolbox` | `ros2_ws_slamtoolbox/` | slam_toolbox 2D mapping (profile: `slamtoolbox`) |
| `glim_env` | `docker/Dockerfile_glim` (公式 `koide3/glim_ros2:jazzy` ベースの薄い層、GLIM 本体はビルドしない) | `ros2_ws_glim/` (config JSON のみ) | 3D SLAM GLIM。GTSAM 4.3a0 同梱 (profile: `glim`) |
| `liosam_env` | `docker/Dockerfile_liosam` | `ros2_ws_liosam/` | LIO-SAM。GTSAM 4.2.0。**IMU 再入手まで凍結** (profile: `liosam`) |

全コンテナ共通: `network_mode: host` + `ipc: host` + `ROS_DOMAIN_ID=150` でコンテナ間は DDS 疎通 (`ipc: host` が無いと FastDDS の共有メモリ転送が繋がらず「topic list に見えるのに echo できない」症状になる)。コンテナは bash 常駐で、起動は `scripts/*.sh` が `docker exec` で launch を投入する方式。

```bash
# Host: clone with submodules
git clone --recursive https://github.com/YamamotoSoya/reRoBot.git

# Host: コンテナ起動 (main のみ。他 profile は用途時に)
xhost +local:docker
docker compose up -d main

# ビルド (main は --executor sequential 必須 — canopen の並列ビルドが壊れる既知問題)
./scripts/build.sh main
./scripts/build.sh slamtoolbox   # slam_toolbox 用 ws (軽量)
./scripts/build.sh images        # Docker イメージ再ビルド (1 本ずつ直列)
```

⚠️ **ビルドの鉄則** (2026-07-26 ユーザ指示):
- `colcon build` は**必ずコンテナ内** (= `scripts/build.sh` 経由)。ホストで実行しない (ホストは Humble で API 不一致、成果物 build/install/log が git を汚す)。
- **並列度は控えめに** — このマシンは重いビルドで落ちる。make は `BUILD_JOBS` (既定 2)、colcon は 1 パッケージずつ、イメージビルドは 1 サービスずつ。`docker compose build` を引数なしで直接叩かない (3 イメージ並列になる)。

git submodule は**各 workspace の src/ 直下に直接配置** (旧 symlink 方式は 2026-07-26 に撤廃):
`ros2_ws_main/src/drivers/{epos4compact50-5can, surestar_rfans_ros2, StarROS2, realsense-ros}`, `ros2_ws_liosam/src/LIO-SAM`。
`ros2_ws_main/src/` は `app/` (自作 C++) / `bringup/` (launch 資産) / `drivers/` (submodule) の 3 グループ構成 (colcon は src を再帰探索するので階層はビルドに無影響)。
旧モノリシック構成は `archive/monolithic` ブランチ + タグ `v1-monolithic` に恒久保存されている (参照専用 — 触るなら `git worktree` で別ツリーへ)。

## Running the Stack

推奨は `scripts/` の用途別スクリプト (ホストで実行)。コンテナ確保 → `docker exec` での launch 投入までを一括で行う:

```bash
./scripts/can_up.sh      # can0 状態確認 + 復旧。通常は udev + canusb-up.service で挿すだけ自動 up (README 参照)
./scripts/bringup2d.sh   # 2D bringup (main, バックグラウンド。ログ: /workspace/log/bringup2d.log)
                         #   IMU=true EKF=true を前置すると BNO086 + EKF 融合込みで起動 (2026-08-11)
./scripts/nav2d.sh       # 自律走行一発 = bringup2d (IMU+EKF 標準) + Nav2 + RViz (main 内)
./scripts/slam2d.sh      # 地図作成 = bringup2d + slam_toolbox + RViz (slamtoolbox コンテナ)
./scripts/bringup3d.sh   # 3D bringup (R-Fans。/scan は出ない)
./scripts/glim3d.sh      # 3D SLAM 評価 = bringup3d + GLIM (glim コンテナ)
./scripts/teleop.sh      # キーボード teleop (対話)
./scripts/stop.sh        # 全コンテナの ROS プロセスに SIGINT (コンテナは残る)
```

中身は「CANopen device container が先、アプリノードは後」という従来の順序をそのまま自動化したもの。手動で 1 コンテナ内を触る場合:

```bash
# main コンテナ内 (docker exec -it rerobot_env bash):
#   統合 bringup (実体)。lidar_2d / lidar_3d / imu を boolean 引数で選択:
ros2 launch rerobot_bringup rerobot_bringup.launch.py lidar_2d:=true lidar_3d:=true imu:=true
#   構成別ラッパ (推奨。実体 launch に boolean を固定して渡すだけ):
ros2 launch rerobot_bringup rerobot_bringup_2d.launch.py        # 2D のみ (IMU なし, scripts 互換)
ros2 launch rerobot_bringup rerobot_bringup_3d.launch.py        # 3D のみ (IMU なし, scripts 互換)
ros2 launch rerobot_bringup rerobot_bringup_2d_imu.launch.py    # 2D + IMU
ros2 launch rerobot_bringup rerobot_bringup_3d_imu.launch.py    # 3D + IMU
ros2 launch rerobot_bringup rerobot_bringup_2d3d_imu.launch.py  # 2D + 3D + IMU 全部載せ
```

The 5-second `TimerAction` in `rerobot_bringup.launch.py` exists because the cia402 `init/enable/cyclic_velocity_mode` services are not advertised until `ros2_canopen`'s device_manager has finished booting both drivers (~3-4 s in practice). Without the delay, `epos4_controller`'s constructor-time `wait_for_service(1s)` calls race the bus_config and silently fail, leaving the EPOS4s disabled.

If you prefer to bring the stack up piece by piece (useful for debugging, main コンテナ内):
```bash
ros2 launch maxon_epos4_ros2 bus_config_cia402_epos4_vel.launch.py
# ...wait until "Slave 0x1: Switched NMT state to START" appears...
ros2 run epos4_controller epos4_controller  --ros-args --params-file src/bringup/rerobot_bringup/config/params.yaml
ros2 run epos4_controller epos4_odometry    --ros-args --params-file src/bringup/rerobot_bringup/config/params.yaml
```

Keyboard teleop (publishes Twist on `/robot_speed_cmd`, prints per-wheel traveled distance from `/motor{1,2}/.../joint_states`):
```bash
ros2 run epos4_teleop teleop_keyboard --ros-args --params-file src/app/epos4_teleop/config/params.yaml
```

個別 launch (どのコンテナで動くかに注意):
```bash
ros2 launch rerobot_slamtoolbox slam.launch.py       # slamtoolbox コンテナ: slam_toolbox mapping + slam.rviz
ros2 launch rerobot_bringup nav2.launch.py           # main コンテナ: map_server + amcl + Nav2 (keepout 込み) + nav2.rviz
ros2 launch rerobot_bringup joy_teleop.launch.py     # main コンテナ: Xbox pad (LB=deadman, RB=turbo)
ros2 launch rerobot_bringup realsense_imu.launch.py  # main コンテナ: RealSense IMU → madgwick → /imu/data (LIO/GLIM 系入力用)
```

(旧 `epos4_vel_ros2` の単体テストは 2026-07-26 に削除 — 必要なら `v1-monolithic` から復元)

## Architecture

The robot's control plane is a layered pipeline; each layer is a separate ROS 2 node so failures and tuning stay isolated.

```
[teleop / nav stack]
        │ geometry_msgs/Twist on /robot_speed_cmd
        ▼
[epos4_controller]   ── inverse kinematics, mode/state mgmt for both motors
        │ canopen_interfaces/COData on /motor{1,2}/cia402_device_{1,2}/tpdo (target velocity)
        ▼
[ros2_canopen Cia402Driver]  ── from drivers/epos4compact50-5can (maxon_epos4_ros2)
        │ CAN frames (SDO/PDO)
        ▼
[EPOS4 #1 + EPOS4 #2 over can0]
        │ sensor_msgs/JointState on /motor{1,2}/cia402_device_{1,2}/joint_states
        ▼
[epos4_odometry] / [epos4_teleop]  ── consume per-motor joint_states directly
```

Note: consumers (`epos4_odometry`, `epos4_teleop`) subscribe to the two
per-motor `joint_states` topics directly. `epos4_odometry` pairs the two
streams with a `message_filters` ApproximateTime sync and republishes
wheel-side angles as `/joint_states` for `robot_state_publisher` — no remap
is involved (the old `/robot_encoder_states` fan-in design is gone).

### Packages

- **`ros2_ws_main/src/app/epos4_controller`** — application layer (executables only). Both nodes consume parameters from `bringup/rerobot_bringup/config/params.yaml` (`tread_width`, `tire_diam`, `gear_ratio`, `invert_left/right`):
  - `epos4_controller` — owns the EPOS4 lifecycle (auto-calls init → enable → cyclic_velocity_mode in its constructor), converts `/robot_speed_cmd` into per-wheel target velocities (rpm), and fans them out to both motors via the canopen TPDO topic at 100 Hz. The `init` service reliably emits `Homing failed` because CSV mode doesn't require homing — this is expected and the subsequent `enable` / `cyclic_velocity_mode` service calls succeed and leave the motors ready.
  - `epos4_odometry` — subscribes to both per-motor `joint_states` topics via a `message_filters` ApproximateTime sync. Computes 2D pose with mid-step heading integration; publishes `/odom`, broadcasts TF, and republishes wheel-side `/joint_states` for `robot_state_publisher`. Parameters for frame names, TF on/off, gear ratio, and per-wheel inversion (`invert_left/right`). ⚠️ `joint_states.velocity` from the driver is **always 0** (0x606C is not PDO-mapped in bus.yml — 2026-08-11 の実走 bag で実証: pose が 20 m 動く間 twist は常に 0)。このため `epos4_odometry` は 2026-08-11 から twist を位置差分 (d_s/dt, d_theta/dt) で計算する。rotation checks must use position deltas.
- **`ros2_ws_main/src/bringup/rerobot_bringup`** — system bringup assets (no C++ code). Owns:
  - `launch/rerobot_bringup.launch.py` — **統合 composite bringup (実体)** (2026-08-10 に 2d/3d launch を統合)。bus_config + 5 s TimerAction + controller + odometry + robot_state_publisher に加え、boolean 引数でセンサドライバを選択: `lidar_2d` → `urg_node` (frame_id `laser`, `/scan`)、`lidar_3d` → `surestar_rfans_ros2` の 2 ノード (R-Fans, `drivers/surestar_rfans_ros2`, frame_id `rfans`, PointCloud2 `/rfans_driver/rfans_points`。2026-08-14 刷新 — LD_PRELOAD と旧 typo topic `/sdk_could` は廃止)、`imu` → `bno086_imu_driver` (`/imu/data`)。`ekf` → `robot_localization` の EKF (車輪 odom + IMU 融合, `config/ekf.yaml`, 2026-08-11 追加) — true で `/odometry/filtered` + TF odom→base_link を EKF が出し、`epos4_odometry` の publish_tf は launch 側で自動 false (TF 二重配信防止。既定 false)。接続系引数 `serial_port` / `device_ip` / `rps` / `model` / `imu_port` で上書き可。Does **not** launch RViz; visualization is owned by `nav2.launch.py` / `slam.launch.py` so the two don't open duplicate windows。`/scan` は 2D LiDAR のみ (3D 点群の LaserScan 変換は含まない)。
  - `launch/rerobot_bringup_{2d,3d}.launch.py` — 構成別ラッパ (IMU なし、`scripts/bringup{2d,3d}.sh` 互換)。`launch/rerobot_bringup_{2d_imu,3d_imu,2d3d_imu}.launch.py` — IMU 込みの構成別ラッパ。全て実体 launch に boolean を固定して渡すだけ。
  - `launch/nav2.launch.py` — map_server + amcl + Nav2 (RPP controller, keepout フィルタ込み) + `nav2.rviz`。config は `config/nav2_params.yaml`。**odom 入力は `/odometry/filtered` (EKF 出力) が標準** (2026-08-11) — `nav2d.sh` が bringup を `imu:=true ekf:=true` で起動する前提。EKF なし運用に戻すときは bt_navigator / controller_server の `odom_topic` を `/odom` に戻す。⚠️ `bt_navigator` の `plugin_lib_names` を列挙すると Jazzy では二重登録 segfault — デフォルトに任せる。
  - `launch/joy_teleop.launch.py` + `config/joy_teleop.yaml` — Xbox ゲームパッド teleop (joy + teleop_twist_joy, LB=deadman, RB=turbo)。
  - `launch/realsense_imu.launch.py` — RealSense を 6 軸 IMU として起動し `imu_filter_madgwick` (use_mag=false) で orientation を合成して `/imu/data` に出す (LIO-SAM の imuTopic 既定と一致)。
  - `config/params.yaml` — 統合パラメータ (2026-08-10 に params_2d/3d を 1 本化)。chassis parameters (`epos4_controller` / `epos4_odometry`) + `rfans_driver` / `rfans_calculation` セクション (新ドライバの機種/接続/frame_id `rfans`/`vangle_override`。2026-08-14 刷新で旧 theta remap は廃止)。
  - `urdf/rerobot.urdf` — 統合 robot description (2026-08-10 に rerobot_2d/3d.urdf を 1 本化)。`laser` / `rfans` / `imu_link` を常に含む。センサ位置は実測反映済み (2026-08-10): laser (0.07, 0, 0.215)・rfans (-0.075, 0, 0.725)・imu (0, 0, 0.64)。IMU の向きは **rpy=(0, 0, π/2) (正立+90°) で最終確定** (2026-08-11, 実走 bag の 3 証拠: 静止 accel z=+9.815 / gyro_z⇔車輪 yaw rate 同符号 +1.015 / 裏返し URDF だと EKF yaw が鏡像)。⚠️ imu_joint を変えたら `ros2_ws_glim/config/config_sensors.json` の `T_lidar_imu` も要再計算。
  - `rviz/nav2.rviz` — Nav2 view (Fixed Frame: `map`, Navigation 2 panel, `/map` + keepout mask + costmaps + global/local plan + amcl particles + footprint). Launched by `nav2.launch.py`.
- **`ros2_ws_slamtoolbox/src/rerobot_slamtoolbox`** — slam_toolbox 系資産 (2026-07-26 に rerobot_bringup から分離、slamtoolbox コンテナ専用):
  - `launch/slam.launch.py` — slam_toolbox (mapping) + `slam.rviz`。config は `config/slam_toolbox.yaml` (scan_queue_size 等は意図的な設定)。lifecycle の CONFIGURE→ACTIVATE を launch 側から発火する (Jazzy の autostart 不具合対応)。
  - `rviz/slam.rviz` — SLAM view (Fixed Frame: `map`, RobotModel/TF/Odometry + the in-progress `/map` + `/scan`; no Nav2-specific displays)。
- **`ros2_ws_main/src/app/epos4_teleop`** — keyboard teleop. Publishes `geometry_msgs/Twist` on `/robot_speed_cmd` and subscribes directly to `/motor{1,2}/cia402_device_{1,2}/joint_states` to print cumulative left/right wheel distance. Keys: `w/s` (linear ±), `a/d` (angular ±), `space`/`x` (stop), `+/-` (scale step), `r` (reset distance), `f` (toggle 脱力/free mode via `/robot_free_mode` — see `docs/features/2026-06-05_motor_free_mode.md`), `q` (quit with zero Twist).
- **`ros2_ws_main/src/drivers/epos4compact50-5can`** — git submodule; vendors the `maxon_epos4_ros2` package. Its launch file (`bus_config_cia402_epos4_vel.launch.py`) is what wires `cia402_device_1` (node_id 1, namespace `/motor1`) and `cia402_device_2` (node_id 2, namespace `/motor2`) onto `can0`. ⚠️ `bus.yml` はこの submodule 内 — 変更は submodule 側にコミットし、親リポジトリで gitlink を更新する。
- **`ros2_ws_main/src/drivers/surestar_rfans_ros2`** — git submodule; R-Fans/C-Fans の**現行ドライバ** (2026-08-14 刷新)。付属 USB の公式オープンソース ROSDriver v2.3.18 を ROS2 移植したもの (クローズド libstar.so 不要、V6K-16G=0x5C 対応、1 回転 1 メッセージ、per-point `time` 相対秒)。経緯・新旧比較は `docs/issue/2026-08-14_rfans_driver_renewal.md`。実測縦角の受け口 `vangle_override` あり (params.yaml 参照)。
- **`ros2_ws_main/src/drivers/StarROS2`** — git submodule; **旧**ドライバ (SDK バイナリ依存)。2026-08-14 に surestar_rfans_ros2 へ置換され **COLCON_IGNORE 済み (参照専用)**。⚠️ 旧ドライバを一時的に使う場合は LiDAR の電源を入れ直すこと (新ドライバが書くレジスタ設定を旧 SDK は解読できない)。旧移植の経緯は `docs/features/2026-06-13_rfans_driver_ros2_port.md`。
- **`ros2_ws_main/src/drivers/realsense-ros`** — git submodule (Intel RealSense driver)。**`ros2_ws_main/src/bringup/realsense2_camera_launch`** (リポジトリ内の launch-only パッケージ) が IMU 専用デフォルト (color/depth 無効, gyro/accel 有効, unite_imu_method=2) の `rs_launch.py` を持つ。
- **`ros2_ws_glim/config/`** — GLIM の設定 JSON 一式 (colcon パッケージではない)。glim コンテナに `/glim_config` として mount され `glim_rosnode -p config_path:=/glim_config` で読まれる。2026-08-11 に **BNO086 IMU あり LIO 構成へ切替済み** (`config_odometry_cpu` + sub/global mapping `enable_imu: true` + `T_lidar_imu` は URDF から算出 — imu_joint 変更時は要再計算)。IMU レス (CT-ICP) に戻す場合は `odometry_estimation_ct` + `enable_imu: false` ×2 + config_ros.json の IMU レス用 TF 設定を復元。
- **`ros2_ws_liosam/src/LIO-SAM`** (ros2 branch) — git submodule; 3D LiDAR + IMU オドメトリ。liosam コンテナでビルド可能だが **IMU 入手不可のため凍結中** (`realsense_imu.launch.py` が入力側 `/imu/data` を用意する段階まで)。

### Key conventions

- The two motors are addressed via the namespaces `/motor1/cia402_device_1` and `/motor2/cia402_device_2` (defined in `drivers/epos4compact50-5can/.../bus.yml`). Any new node that talks to a motor must follow this namespace pattern. Physical wiring: **motor1 = right wheel, motor2 = left wheel** (see the `claude_swap` comments in `epos4_controller` / `epos4_odometry`; some older log strings still carry the reversed labels).
- `epos4_controller` drives the EPOS4 in **cyclic synchronous velocity mode** by writing target velocity (object 0x60FF, sub 0x00) into `COData` messages on the per-motor `tpdo` topic. The controller's own publish timer runs at 100 Hz; the PDO sync period is 50 ms (set in `bus.yml`).
- Joint position/velocity from the canopen driver are **SI-scaled by the driver itself** via `bus.yml`'s `scale_pos_from_dev = 0.0061359 (= 2π/1024; 実機エンコーダ 256 pulses × 4 逓倍, 2026-07-29 修正)` and `scale_vel_from_dev = 0.10472 (= 2π/60)`. So `joint_states.position` is **motor-shaft angle in radians** and `joint_states.velocity` is **rad/s** — not raw qc/rpm. Any consumer computing wheel distance should do `distance_m = (Δposition / gear_ratio) × (tire_diam / 2)`. Outgoing commands: controller publishes **rpm** on the raw `tpdo` topic, which the driver passes through **unscaled** to 0x60FF (candump で実証 2026-07-29 — `scale_vel_to_dev` はこの経路では使われない)。EPOS4 側の Velocity Unit が 1 rpm なので単位はそのまま一致する。
- ROS 2 params files **must** use the key `ros__parameters` (two underscores). A single-underscore typo (`ros_parameters`) will crash the node on startup with `RCLInvalidROSArgsError: Cannot have a value before ros__parameters`, and it is not obvious from the symptom (nodes exit before publishing anything).
- `tread_width`, `tire_diam`, `gear_ratio`, and `invert_left/right` are duplicated as ROS parameters in each consumer, sourced from `ros2_ws_main/src/bringup/rerobot_bringup/config/params.yaml` (and a parallel copy in `ros2_ws_main/src/app/epos4_teleop/config/params.yaml`). Keep them in sync if you change the chassis (`/params-sync` skill で検査できる)。(2026-08-10: params_2d/3d の 2 ファイル重複は params.yaml 統合で解消 — 残る重複は teleop 側のみ)
- `gear_ratio: 5.0` は物理ギヤ比 (motor_rev / wheel_rev)。2026-07-29 に根本修正済み — それ以前は bus.yml のエンコーダ分解能ズレ (4096 vs 実機 1024) を `gear_ratio: 1.25` で相殺しており、**実車が指令速度の 1/4 で走っていた**。修正は bus.yml (`2π/1024`) と gear_ratio (5.0) の 2 点同時変更が必須で、片方だけ戻すと距離か速度が 4 倍狂う。経緯は `docs/issue/2026-07-07_wheel_odometry_encoder_scaling_4x.md`。⚠️ 修正以前に較正した速度系チューニング (teleop スケール・Nav2 速度上限等) は「1/4 速の実車」基準の可能性があるので注意。

## Documentation & Claude Workflow

新しいセッションの読み順: **CLAUDE.md (規約・ビルド) → `docs/claude/PROJECT_STATE.md` (現在地・既知の問題・タイムライン) → `docs/claude/USER_LEVEL.md` (ユーザ知識レベル — 注釈・解説の較正) → `docs/issue/` (問題詳細)**。

| 場所 | 役割 | 編集ルール |
|------|------|-----------|
| `docs/claude/PROJECT_STATE.md` | Claude 用の状態メモ | 大きな状態変化 (機能追加・重要バグ解決・方針変更) があったら必ず更新 (`/project-state` skill) |
| `docs/claude/USER_LEVEL.md` | ユーザ知識レベルプロファイル (annotate/knowledge-check の較正元) | ⚠️ **個人情報のため git 管理外 (.gitignore 済み)・コミット禁止**。更新は `/user-level` skill |
| `docs/issue/` | 未解決問題の調査記録 | 1 issue = 1 ファイル、冒頭にステータス。解決したらステータスを更新 |
| `docs/report/` | 解決済みバグの事後報告 | 大きなデバッグ完了時に `/debug-report` skill で追加 |
| `docs/features/` | 追加機能の設計文書 | 機能追加時に `/feature-doc` skill で追加 |
| `docs/monthly/` | ユーザの月次ゼミ報告 | **Claude は編集禁止** (読み取り専用の入力) |
| `docs/text/` | テーマ別の体系的解説書 (書籍形式の教材) | 1 テーマ = 1 ディレクトリ。作成・更新は `/textbook` skill |

Project skills (`.claude/skills/`): `verify` (コンテナ `rerobot_env` 内でビルド+起動検証; 実機 can0 が無ければ vcan0 + fake-slaves), `stack-health` (稼働スタックの読み取り専用診断), `params-sync` (車体パラメータ 3 ファイルの整合検査), `debug-report` / `feature-doc` / `project-state` (docs 更新), `knowledge-check` (実装の理解確認 → `docs/claude/knowledge/`), `annotate` (返信中の用語注釈の基準・書式), `explain-edits` (ファイル変更前の説明の基準・書式), `user-level` (知識レベルプロファイル `docs/claude/USER_LEVEL.md` の再推定), `textbook` (docs/text/ にテーマ別解説書を書籍形式で作成 — 章構成の型・樹形図優先・事例解剖の型を持つ)。

MCP servers (`.mcp.json`): `context7` (ライブラリ最新ドキュメント), `playwright`, `memory` (`~/.claude/rerobot-mcp-memory.json`)。

作業慣例:
- Claude が書いた・変更したコードには `// claude` 系コメントタグを付ける (既存コード参照)。clang-format 準拠。
- **main 直コミットで運用する** (2026-07-26 方針変更、ユーザ指示)。壊れる可能性が高い実験的変更を隔離したいときだけ任意でブランチ (`fix/...`, `feat/...`) を切る。
- モータを動かす検証は「浮かせて確認 → 接地」の順。ただしハンチング等の挙動評価は接地状態でのみ有効。
- ドキュメント・ユーザへの応答は日本語。説明は「なぜそうなるか」まで書く。
- 返信中の発展的コマンド・専門用語には `※n` マーカー + 返信末尾の `📘 注釈` ブロックで注釈を付ける (1 返信最大 3 個)。レベル基準・書式・既知用語リストは `.claude/skills/annotate/SKILL.md` — 「それは知ってる」と言われた語は同ファイルの既知リストに追記し、以後注釈しない。
- **ファイルを変更する (Edit/Write) 前に、その直前の本文で「どこを・何を・なぜ」を 1〜4 行で説明する** (2026-08-11 ユーザ指示、常時適用)。書式・粒度の基準は `.claude/skills/explain-edits/SKILL.md`。
