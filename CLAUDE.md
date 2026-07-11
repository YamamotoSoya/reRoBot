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

The `src/maxon_epos4_ros2` entry is a **symlink** into `src/external/epos4compact50-5can/maxon_epos4_ros2/` (the git submodule). If the symlink breaks after clone, recreate it:
```bash
cd src && ln -s external/epos4compact50-5can/maxon_epos4_ros2 .
```

## Running the Stack

The CANopen device container must be launched first; only then do the application nodes have something to talk to. The recommended path is the one-shot bringup launch, which wires bus_config + controller + odometry + robot_state_publisher together:

```bash
# 1. Bring up CAN interface (host or container, requires privileged)
sudo ip link set can0 up type can bitrate 1000000

# 2. One-shot: bus_config + epos4_controller + epos4_odometry + robot_state_publisher + LiDAR
#    2D LiDAR (HOKUYO urg_node) 構成:
ros2 launch rerobot_bringup rerobot_bringup_2d.launch.py
#    3D LiDAR (R-Fans rfans_driver) 構成:
ros2 launch rerobot_bringup rerobot_bringup_3d.launch.py
```

Startup ordering against `ros2_canopen`'s device_manager (which advertises the cia402 `init/enable/cyclic_velocity_mode` services only after booting both drivers, ~3-4 s) is handled **inside `epos4_controller`**, not in launch: its background init thread waits up to 20 s for the `init` service to appear, then drives init → enable → cyclic_velocity_mode sequentially and verifies the result over SDO (statusword 0x6041 / mode 0x6061), retrying with `recover` on failure. The 5-second `TimerAction` that older launch revisions used for this is gone.

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

Note: `epos4_odometry` pairs the two per-motor `joint_states` topics by
header stamp (message_filters `ApproximateTime`; PDO sync is 50 ms on both
motors, so each pair collapses into one callback) and republishes wheel-side
angles as `/joint_states` (`m1_wheel`, `m2_wheel`), which
`robot_state_publisher` consumes for the dynamic wheel TFs. The old
`/robot_encoder_states` fan-in concept is dead and its remnants have been
removed from `epos4_controller`.

`epos4_controller` also subscribes to `/robot_free_mode` (`std_msgs/Bool`):
`true` disables both drives (脱力/free-wheel, velocity commands ignored),
`false` re-enables them (enable → CSV, sequenced and SDO-verified). Keyboard
teleop toggles this with the `f` key.

### Packages

- **`src/epos4_controller`** — application layer (executables only). Both nodes consume parameters from `src/rerobot_bringup/config/params_2d.yaml` (3D 構成では `params_3d.yaml`; epos4 セクションは同値) (`tread_width`, `tire_diam`, `gear_ratio`, `invert_left/right`):
  - `epos4_controller` — owns the EPOS4 lifecycle. A background init thread waits (≤20 s) for the cia402 services, then runs init → enable → cyclic_velocity_mode per motor sequentially, verifies over SDO, and retries with `recover` (the `init` step reliably logs `Homing failed` because CSV doesn't require homing — expected). Converts `/robot_speed_cmd` into per-wheel target velocities (rpm) and fans them out via the canopen TPDO topics at 100 Hz. Handles `/robot_free_mode` (disable / re-enable both drives) and sends `disable` to both drives from a `pre_shutdown` callback on exit. Per-motor plumbing lives in the `MotorInterface` struct — extend that (not copy-pasted members) for new per-motor I/O.
  - `epos4_odometry` — syncs the two per-motor `joint_states` topics (message_filters `ApproximateTime`), computes 2D pose with mid-step heading integration, publishes `/odom` + TF, and republishes wheel-side `/joint_states` for `robot_state_publisher`. Parameters for frame names, TF on/off, gear ratio, and per-wheel inversion (`invert_left/right`).
- **`src/rerobot_bringup`** — system bringup assets (no C++ code). Owns:
  - `launch/rerobot_bringup_common.launch.py` — 2D/3D 共通部 (bus_config + controller + odometry + robot_state_publisher)。`params_file` / `urdf_file` を launch 引数で受け取り、URDF は `cat` の Command substitution で読み込む。単体起動は想定せず、下の 2 ラッパーから include される。
  - `launch/rerobot_bringup_2d.launch.py` — **2D LiDAR** bringup wrapper: common + `urg_node` (frame_id `laser`, `serial_port` 引数)。Does **not** launch RViz; visualization is owned by `nav2.launch.py` / `slam.launch.py` so the two don't open duplicate windows.
  - `launch/rerobot_bringup_3d.launch.py` — **3D LiDAR** bringup wrapper: common + `rfans_driver` (R-Fans, `src/external/StarROS2`)。frame_id `rfans`、出力 PointCloud2 `/sdk_could`。`device_ip` / `rps` / `model` を launch 引数で上書き可。`/scan` は出さない (LaserScan 変換は含まない)。
  - `config/params_2d.yaml` — chassis parameters (2D 構成)。車体パラメータは `/**:` セクション (このファイルを読む全ノード共通) に一本化されている。rcl の yaml パーサは YAML アンカーを解釈しないため、共通化には必ず `/**` を使うこと。
  - `config/params_3d.yaml` — 同 chassis parameters (`/**`、2D と同値) + `rfans_driver` セクション (機種/接続/frame_id `rfans`/theta remap)。
  - `urdf/rerobot_2d.urdf` — robot description (2D, LiDAR link `laser` @ xyz `0 0 0.714`).
  - `urdf/rerobot_3d.urdf` — robot description (3D, LiDAR link `rfans` を 2D と同位置 xyz `0 0 0.714` に配置)。
  - `rviz/nav2.rviz` — Nav2 view (Fixed Frame: `map`, Navigation 2 panel, `/map` + keepout mask + costmaps + global/local plan + amcl particles + footprint). Launched by `nav2.launch.py`.
  - `rviz/slam.rviz` — SLAM view (Fixed Frame: `map`, RobotModel/TF/Odometry + the in-progress `/map` + `/scan`; no Nav2-specific displays). Launched by `slam.launch.py`.
- **`src/epos4_teleop`** — keyboard teleop. Publishes `geometry_msgs/Twist` on `/robot_speed_cmd` and subscribes directly to `/motor{1,2}/cia402_device_{1,2}/joint_states` to print cumulative left/right wheel distance. Keys: `w/s` (linear ±), `a/d` (angular ±), `space`/`x` (stop), `+/-` (scale step), `r` (reset distance), `f` (toggle 脱力/free mode via `/robot_free_mode`), `q` (quit with zero Twist).
- **`src/epos4_vel_ros2`** — standalone single-motor (and 2-motor) test programs. Useful for bench-bringing-up an EPOS4 without the full control stack.
- **`src/external/epos4compact50-5can`** — git submodule; vendors the `maxon_epos4_ros2` package. Its launch file (`bus_config_cia402_epos4_vel.launch.py`) is what wires `cia402_device_1` (node_id 1, namespace `/motor1`) and `cia402_device_2` (node_id 2, namespace `/motor2`) onto `can0`.

### Key conventions

- The two motors are addressed via the namespaces `/motor1/cia402_device_1` and `/motor2/cia402_device_2` (defined in `external/.../bus.yml`). Any new node that talks to a motor must follow this namespace pattern. **Physical wiring is `motor1 = RIGHT wheel`, `motor2 = LEFT wheel`** (`claude_swap` markers in the code) — `epos4_controller`, `epos4_odometry`, and `epos4_teleop` all follow this mapping; do not assume motor index = left/right order.
- `epos4_controller` drives the EPOS4 in **cyclic synchronous velocity mode** by writing target velocity (object 0x60FF, sub 0x00) into `COData` messages on the per-motor `tpdo` topic. The controller's own publish timer runs at 100 Hz; the PDO sync period is 50 ms (set in `bus.yml`).
- Joint position/velocity from the canopen driver are **SI-scaled by the driver itself** via `bus.yml`'s `scale_pos_from_dev = 0.0015339 (≈ 2π/4096)` and `scale_vel_from_dev = 0.10472 (= 2π/60)`. So `joint_states.position` is **motor-shaft angle in radians** and `joint_states.velocity` is **rad/s** — not raw qc/rpm. Any consumer computing wheel distance should do `distance_m = (Δposition / gear_ratio) × (tire_diam / 2)`. Outgoing commands go the other way: controller publishes **rpm** on the TPDO, which the driver scales to device units via `scale_vel_to_dev = 9.5493`.
- ROS 2 params files **must** use the key `ros__parameters` (two underscores). A single-underscore typo (`ros_parameters`) will crash the node on startup with `RCLInvalidROSArgsError: Cannot have a value before ros__parameters`, and it is not obvious from the symptom (nodes exit before publishing anything).
- `tread_width`, `tire_diam`, `gear_ratio`, and `invert_left/right` live in the `/**:` section of `src/rerobot_bringup/config/params_2d.yaml` and `params_3d.yaml` (one copy per file; the two files must match) plus a parallel copy in `src/epos4_teleop/config/params.yaml` (whose `invert_*` intentionally differ — teleop counts raw travel). Keep 2D/3D/teleop in sync if you change the chassis; the `/params-sync` skill checks this.
