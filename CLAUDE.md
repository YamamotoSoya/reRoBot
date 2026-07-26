# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

`reRoBot` is an autonomous navigation robot built for the Tsukuba Challenge. The codebase is a ROS 2 (Jazzy) workspace whose primary purpose is to drive a differential-drive base via two maxon EPOS4 motor controllers connected over CAN bus, using `ros2_canopen` (CiA 402 profile).

## Environment & Build

The expected workflow runs inside the provided Docker container (ROS 2 Jazzy + `ros2_canopen` + `can-utils`). Running on the host directly is not the supported path.

```bash
# Host: clone with submodules
git clone --recursive https://github.com/YamamotoSoya/reRoBot.git

# Host: bring up the container (binds /dev for CAN, X11 for RViz)
xhost +local:docker
docker compose up --build
docker exec -it rerobot_env bash

# Inside container (WORKDIR=/workspace, src is bind-mounted from host)
rosdep update
rosdep install --from-paths src --ignore-src --simulate
colcon build --symlink-install --executor sequential
source install/setup.bash
```

Note: `--executor sequential` is intentional — parallel builds have known issues with the canopen stack on this setup.

`src/external/` holds all git submodules and carries a `COLCON_IGNORE`; buildable packages are exposed to colcon via **symlinks in `src/`**:
`maxon_epos4_ros2 → external/epos4compact50-5can/maxon_epos4_ros2/`, `StarROS2 → external/StarROS2`, `LIO-SAM → external/LIO-SAM`, `realsense-ros → external/realsense-ros`. If a symlink breaks after clone, recreate it, e.g.:
```bash
cd src && ln -s external/epos4compact50-5can/maxon_epos4_ros2 .
```

## Running the Stack

The CANopen device container must be launched first; only then do the application nodes have something to talk to. The recommended path is the one-shot bringup launch, which wires bus_config + controller + odometry + robot_state_publisher together:

```bash
# 1. Bring up CAN interface (host or container, requires privileged)
sudo ip link set can0 up type can bitrate 1000000

# 2. One-shot: bus_config → (5s delay) → epos4_controller + epos4_odometry + robot_state_publisher
#    2D LiDAR (HOKUYO urg_node) 構成:
ros2 launch rerobot_bringup rerobot_bringup_2d.launch.py
#    3D LiDAR (R-Fans rfans_driver) 構成:
ros2 launch rerobot_bringup rerobot_bringup_3d.launch.py
```

The 5-second `TimerAction` in `rerobot_bringup_2d.launch.py` / `rerobot_bringup_3d.launch.py` exists because the cia402 `init/enable/cyclic_velocity_mode` services are not advertised until `ros2_canopen`'s device_manager has finished booting both drivers (~3-4 s in practice). Without the delay, `epos4_controller`'s constructor-time `wait_for_service(1s)` calls race the bus_config and silently fail, leaving the EPOS4s disabled.

If you prefer to bring the stack up piece by piece (useful for debugging):
```bash
ros2 launch maxon_epos4_ros2 bus_config_cia402_epos4_vel.launch.py
# ...wait until "Slave 0x1: Switched NMT state to START" appears...
ros2 run epos4_controller epos4_controller  --ros-args --params-file src/rerobot_bringup/config/params_2d.yaml
ros2 run epos4_controller epos4_odometry    --ros-args --params-file src/rerobot_bringup/config/params_2d.yaml
```

Keyboard teleop (publishes Twist on `/robot_speed_cmd`, prints per-wheel traveled distance from `/motor{1,2}/.../joint_states`):
```bash
ros2 run epos4_teleop teleop_keyboard --ros-args --params-file src/epos4_teleop/config/params.yaml
```

SLAM / Nav2 / gamepad / IMU (each in its own terminal, on top of a running bringup):
```bash
ros2 launch rerobot_bringup slam.launch.py           # slam_toolbox mapping + slam.rviz
ros2 launch rerobot_bringup nav2.launch.py           # map_server + amcl + Nav2 (keepout 込み) + nav2.rviz
ros2 launch rerobot_bringup joy_teleop.launch.py     # Xbox pad (LB=deadman, RB=turbo)
ros2 launch rerobot_bringup realsense_imu.launch.py  # RealSense IMU → madgwick → /imu/data (LIO-SAM 入力用)
```

Single-motor sanity tests live in `epos4_vel_ros2`:
```bash
ros2 run epos4_vel_ros2 epos4_vel_test    # one EPOS4 (motor1)
ros2 run epos4_vel_ros2 2chanel_test      # both EPOS4s
```

## Architecture

The robot's control plane is a layered pipeline; each layer is a separate ROS 2 node so failures and tuning stay isolated.

```
[teleop / nav stack]
        │ geometry_msgs/Twist on /robot_speed_cmd
        ▼
[epos4_controller]   ── inverse kinematics, mode/state mgmt for both motors
        │ canopen_interfaces/COData on /motor{1,2}/cia402_device_{1,2}/tpdo (target velocity)
        ▼
[ros2_canopen Cia402Driver]  ── from external/maxon_epos4_ros2
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

- **`src/epos4_controller`** — application layer (executables only). Both nodes consume parameters from `src/rerobot_bringup/config/params_2d.yaml` (3D 構成では `params_3d.yaml`; epos4 セクションは同値) (`tread_width`, `tire_diam`, `gear_ratio`, `invert_left/right`):
  - `epos4_controller` — owns the EPOS4 lifecycle (auto-calls init → enable → cyclic_velocity_mode in its constructor), converts `/robot_speed_cmd` into per-wheel target velocities (rpm), and fans them out to both motors via the canopen TPDO topic at 100 Hz. The `init` service reliably emits `Homing failed` because CSV mode doesn't require homing — this is expected and the subsequent `enable` / `cyclic_velocity_mode` service calls succeed and leave the motors ready.
  - `epos4_odometry` — subscribes to both per-motor `joint_states` topics via a `message_filters` ApproximateTime sync. Computes 2D pose with mid-step heading integration; publishes `/odom`, broadcasts TF, and republishes wheel-side `/joint_states` for `robot_state_publisher`. Parameters for frame names, TF on/off, gear ratio, and per-wheel inversion (`invert_left/right`). ⚠️ `joint_states.velocity` from the driver is suspected always-0 (0x606C is not PDO-mapped in bus.yml) — rotation checks must use position deltas.
- **`src/rerobot_bringup`** — system bringup assets (no C++ code). Owns:
  - `launch/rerobot_bringup_2d.launch.py` — **2D LiDAR** composite bringup (bus_config + 5 s TimerAction + controller + odometry + robot_state_publisher + `urg_node`, frame_id `laser`). Does **not** launch RViz; visualization is owned by `nav2.launch.py` / `slam.launch.py` so the two don't open duplicate windows.
  - `launch/rerobot_bringup_3d.launch.py` — **3D LiDAR** composite bringup. `rerobot_bringup_2d.launch.py` と同構成で `urg_node` を `rfans_driver` (R-Fans, `src/external/StarROS2`) に差し替えたもの。frame_id `rfans`、出力 PointCloud2 `/sdk_could`。`device_ip` / `rps` / `model` を launch 引数で上書き可。`/scan` は出さない (LaserScan 変換は含まない)。
  - `launch/slam.launch.py` — slam_toolbox (mapping) + `slam.rviz`。config は `config/slam_toolbox.yaml` (scan_queue_size 等は意図的な設定)。
  - `launch/nav2.launch.py` — map_server + amcl + Nav2 (RPP controller, keepout フィルタ込み) + `nav2.rviz`。config は `config/nav2_params.yaml`。⚠️ `bt_navigator` の `plugin_lib_names` を列挙すると Jazzy では二重登録 segfault — デフォルトに任せる。
  - `launch/joy_teleop.launch.py` + `config/joy_teleop.yaml` — Xbox ゲームパッド teleop (joy + teleop_twist_joy, LB=deadman, RB=turbo)。
  - `launch/realsense_imu.launch.py` — RealSense を 6 軸 IMU として起動し `imu_filter_madgwick` (use_mag=false) で orientation を合成して `/imu/data` に出す (LIO-SAM の imuTopic 既定と一致)。
  - `config/params_2d.yaml` — chassis parameters consumed by `epos4_controller` / `epos4_odometry` (2D 構成)。
  - `config/params_3d.yaml` — 同 chassis parameters (2D と同値) + `rfans_driver` セクション (機種/接続/frame_id `rfans`/theta remap)。
  - `urdf/rerobot_2d.urdf` — robot description (2D, LiDAR link `laser` @ xyz `0 0 0.714`).
  - `urdf/rerobot_3d.urdf` — robot description (3D, LiDAR link `rfans` を 2D と同位置 xyz `0 0 0.714` に配置)。
  - `rviz/nav2.rviz` — Nav2 view (Fixed Frame: `map`, Navigation 2 panel, `/map` + keepout mask + costmaps + global/local plan + amcl particles + footprint). Launched by `nav2.launch.py`.
  - `rviz/slam.rviz` — SLAM view (Fixed Frame: `map`, RobotModel/TF/Odometry + the in-progress `/map` + `/scan`; no Nav2-specific displays). Launched by `slam.launch.py`.
- **`src/epos4_teleop`** — keyboard teleop. Publishes `geometry_msgs/Twist` on `/robot_speed_cmd` and subscribes directly to `/motor{1,2}/cia402_device_{1,2}/joint_states` to print cumulative left/right wheel distance. Keys: `w/s` (linear ±), `a/d` (angular ±), `space`/`x` (stop), `+/-` (scale step), `r` (reset distance), `f` (toggle 脱力/free mode via `/robot_free_mode` — see `docs/features/2026-06-05_motor_free_mode.md`), `q` (quit with zero Twist).
- **`src/epos4_vel_ros2`** — standalone single-motor (and 2-motor) test programs. Useful for bench-bringing-up an EPOS4 without the full control stack.
- **`src/external/epos4compact50-5can`** — git submodule; vendors the `maxon_epos4_ros2` package. Its launch file (`bus_config_cia402_epos4_vel.launch.py`) is what wires `cia402_device_1` (node_id 1, namespace `/motor1`) and `cia402_device_2` (node_id 2, namespace `/motor2`) onto `can0`. ⚠️ `bus.yml` はこの submodule 内 — 変更は submodule 側にコミットし、親リポジトリで gitlink を更新する。
- **`src/external/StarROS2`** — git submodule; Sure-Star R-Fans-16 の `rfans_driver` (ROS 2 移植版)。経緯は `docs/features/2026-06-13_rfans_driver_ros2_port.md`。
- **`src/external/LIO-SAM`** (ros2 branch) — git submodule; 3D LiDAR + IMU オドメトリ。ビルドは通る (Dockerfile に `ros-jazzy-gtsam` 追加済み) が、bringup への統合はまだ (`realsense_imu.launch.py` が入力側 `/imu/data` を用意する段階まで)。
- **`src/external/realsense-ros`** — git submodule (Intel RealSense driver)。**`src/realsense2_camera_launch`** (リポジトリ内の launch-only パッケージ) が IMU 専用デフォルト (color/depth 無効, gyro/accel 有効, unite_imu_method=2) の `rs_launch.py` を持つ。

### Key conventions

- The two motors are addressed via the namespaces `/motor1/cia402_device_1` and `/motor2/cia402_device_2` (defined in `external/.../bus.yml`). Any new node that talks to a motor must follow this namespace pattern. Physical wiring: **motor1 = right wheel, motor2 = left wheel** (see the `claude_swap` comments in `epos4_controller` / `epos4_odometry`; some older log strings still carry the reversed labels).
- `epos4_controller` drives the EPOS4 in **cyclic synchronous velocity mode** by writing target velocity (object 0x60FF, sub 0x00) into `COData` messages on the per-motor `tpdo` topic. The controller's own publish timer runs at 100 Hz; the PDO sync period is 50 ms (set in `bus.yml`).
- Joint position/velocity from the canopen driver are **SI-scaled by the driver itself** via `bus.yml`'s `scale_pos_from_dev = 0.0015339 (≈ 2π/4096)` and `scale_vel_from_dev = 0.10472 (= 2π/60)`. So `joint_states.position` is **motor-shaft angle in radians** and `joint_states.velocity` is **rad/s** — not raw qc/rpm. Any consumer computing wheel distance should do `distance_m = (Δposition / gear_ratio) × (tire_diam / 2)`. Outgoing commands go the other way: controller publishes **rpm** on the TPDO, which the driver scales to device units via `scale_vel_to_dev = 9.5493`.
- ROS 2 params files **must** use the key `ros__parameters` (two underscores). A single-underscore typo (`ros_parameters`) will crash the node on startup with `RCLInvalidROSArgsError: Cannot have a value before ros__parameters`, and it is not obvious from the symptom (nodes exit before publishing anything).
- `tread_width`, `tire_diam`, `gear_ratio`, and `invert_left/right` are duplicated as ROS parameters in each consumer, sourced from `src/rerobot_bringup/config/params_2d.yaml` and `params_3d.yaml` (whose epos4 sections must match) (and a parallel copy in `src/epos4_teleop/config/params.yaml`). Keep them in sync if you change the chassis (`/params-sync` skill で検査できる)。
- ⚠️ `gear_ratio: 1.25` is deliberate: the physical reduction is 5:1, but the encoder resolution is off by exactly 4× (quadrature double-interpretation), and 1.25 compensates. Do **not** "fix" it to 5.0 in isolation — the proper fix changes three things at once (EPOS4 object 0x3010:01 / bus.yml scale / gear_ratio). See `docs/issue/2026-07-07_wheel_odometry_encoder_scaling_4x.md`.

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

Project skills (`.claude/skills/`): `verify` (コンテナ `rerobot_env` 内でビルド+起動検証; 実機 can0 が無ければ vcan0 + fake-slaves), `stack-health` (稼働スタックの読み取り専用診断), `params-sync` (車体パラメータ 3 ファイルの整合検査), `debug-report` / `feature-doc` / `project-state` (docs 更新), `knowledge-check` (実装の理解確認 → `docs/claude/knowledge/`), `annotate` (返信中の用語注釈の基準・書式), `user-level` (知識レベルプロファイル `docs/claude/USER_LEVEL.md` の再推定)。

MCP servers (`.mcp.json`): `context7` (ライブラリ最新ドキュメント), `playwright`, `memory` (`~/.claude/rerobot-mcp-memory.json`)。

作業慣例:
- Claude が書いた・変更したコードには `// claude` 系コメントタグを付ける (既存コード参照)。clang-format 準拠。
- **main 直コミットで運用する** (2026-07-26 方針変更、ユーザ指示)。壊れる可能性が高い実験的変更を隔離したいときだけ任意でブランチ (`fix/...`, `feat/...`) を切る。
- モータを動かす検証は「浮かせて確認 → 接地」の順。ただしハンチング等の挙動評価は接地状態でのみ有効。
- ドキュメント・ユーザへの応答は日本語。説明は「なぜそうなるか」まで書く。
- 返信中の発展的コマンド・専門用語には `※n` マーカー + 返信末尾の `📘 注釈` ブロックで注釈を付ける (1 返信最大 3 個)。レベル基準・書式・既知用語リストは `.claude/skills/annotate/SKILL.md` — 「それは知ってる」と言われた語は同ファイルの既知リストに追記し、以後注釈しない。
