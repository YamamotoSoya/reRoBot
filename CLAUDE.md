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

# 2. One-shot: bus_config → (5s delay) → epos4_controller + epos4_odometry + robot_state_publisher
ros2 launch rerobot_bringup rerobot_bringup.launch.py
```

The 5-second `TimerAction` in `rerobot_bringup.launch.py` exists because the cia402 `init/enable/cyclic_velocity_mode` services are not advertised until `ros2_canopen`'s device_manager has finished booting both drivers (~3-4 s in practice). Without the delay, `epos4_controller`'s constructor-time `wait_for_service(1s)` calls race the bus_config and silently fail, leaving the EPOS4s disabled.

If you prefer to bring the stack up piece by piece (useful for debugging):
```bash
ros2 launch maxon_epos4_ros2 bus_config_cia402_epos4_vel.launch.py
# ...wait until "Slave 0x1: Switched NMT state to START" appears...
ros2 run epos4_controller epos4_controller  --ros-args --params-file src/rerobot_bringup/config/params.yaml
ros2 run epos4_controller epos4_odometry    --ros-args --params-file src/rerobot_bringup/config/params.yaml
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

Note: `epos4_controller`'s in-code fan-in of both motors into a single
`/robot_encoder_states` topic is currently commented out. Consumers
(`epos4_odometry`, `epos4_teleop`) subscribe to the two per-motor
`joint_states` topics directly. `rerobot_bringup.launch.py` remaps
`robot_state_publisher`'s `/joint_states` input onto `/robot_encoder_states`,
so publishing that aggregate topic again is the intended future fix.

### Packages

- **`src/epos4_controller`** — application layer (executables only). Both nodes consume parameters from `src/rerobot_bringup/config/params.yaml` (`tread_width`, `tire_diam`, `gear_ratio`, `invert_left/right`):
  - `epos4_controller` — owns the EPOS4 lifecycle (auto-calls init → enable → cyclic_velocity_mode in its constructor), converts `/robot_speed_cmd` into per-wheel target velocities (rpm), and fans them out to both motors via the canopen TPDO topic at 100 Hz. The `init` service reliably emits `Homing failed` because CSV mode doesn't require homing — this is expected and the subsequent `enable` / `cyclic_velocity_mode` service calls succeed and leave the motors ready.
  - `epos4_odometry` — subscribes to `/robot_encoder_states` (currently unpopulated; see architecture note). Computes 2D pose with mid-step heading integration; publishes `/odom` and broadcasts TF. Parameters for frame names, TF on/off, gear ratio, and per-wheel inversion (`invert_left/right`).
- **`src/rerobot_bringup`** — system bringup assets (no C++ code). Owns:
  - `launch/rerobot_bringup.launch.py` — composite bringup (bus_config + 5 s TimerAction + controller + odometry + RViz).
  - `config/params.yaml` — chassis parameters consumed by `epos4_controller` / `epos4_odometry`.
  - `urdf/rerobot.urdf` — robot description (used by `robot_state_publisher` when re-enabled).
  - `rviz/rerobot.rviz` — preset RViz layout (odom fixed frame, RobotModel/TF/Odometry displays).
- **`src/epos4_teleop`** — keyboard teleop. Publishes `geometry_msgs/Twist` on `/robot_speed_cmd` and subscribes directly to `/motor{1,2}/cia402_device_{1,2}/joint_states` to print cumulative left/right wheel distance. Keys: `w/s` (linear ±), `a/d` (angular ±), `space`/`x` (stop), `+/-` (scale step), `r` (reset distance), `q` (quit with zero Twist).
- **`src/epos4_vel_ros2`** — standalone single-motor (and 2-motor) test programs. Useful for bench-bringing-up an EPOS4 without the full control stack.
- **`src/external/epos4compact50-5can`** — git submodule; vendors the `maxon_epos4_ros2` package. Its launch file (`bus_config_cia402_epos4_vel.launch.py`) is what wires `cia402_device_1` (node_id 1, namespace `/motor1`) and `cia402_device_2` (node_id 2, namespace `/motor2`) onto `can0`.

### Key conventions

- The two motors are addressed via the namespaces `/motor1/cia402_device_1` and `/motor2/cia402_device_2` (defined in `external/.../bus.yml`). Any new node that talks to a motor must follow this namespace pattern. In `epos4_controller`'s kinematics, `motor1 = left wheel` and `motor2 = right wheel`.
- `epos4_controller` drives the EPOS4 in **cyclic synchronous velocity mode** by writing target velocity (object 0x60FF, sub 0x00) into `COData` messages on the per-motor `tpdo` topic. The controller's own publish timer runs at 100 Hz; the PDO sync period is 50 ms (set in `bus.yml`).
- Joint position/velocity from the canopen driver are **SI-scaled by the driver itself** via `bus.yml`'s `scale_pos_from_dev = 0.0015339 (≈ 2π/4096)` and `scale_vel_from_dev = 0.10472 (= 2π/60)`. So `joint_states.position` is **motor-shaft angle in radians** and `joint_states.velocity` is **rad/s** — not raw qc/rpm. Any consumer computing wheel distance should do `distance_m = (Δposition / gear_ratio) × (tire_diam / 2)`. Outgoing commands go the other way: controller publishes **rpm** on the TPDO, which the driver scales to device units via `scale_vel_to_dev = 9.5493`.
- ROS 2 params files **must** use the key `ros__parameters` (two underscores). A single-underscore typo (`ros_parameters`) will crash the node on startup with `RCLInvalidROSArgsError: Cannot have a value before ros__parameters`, and it is not obvious from the symptom (nodes exit before publishing anything).
- `tread_width`, `tire_diam`, `gear_ratio`, and `invert_left/right` are duplicated as ROS parameters in each consumer, sourced from `src/rerobot_bringup/config/params.yaml` (and a parallel copy in `src/epos4_teleop/config/params.yaml`). Keep them in sync if you change the chassis.
