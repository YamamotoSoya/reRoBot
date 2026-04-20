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

The CANopen device container must be launched first; only then do the application nodes have something to talk to.

```bash
# 1. Bring up CAN interface (host or container, requires privileged)
sudo ip link set can0 up type can bitrate 1000000

# 2. Start the ros2_canopen master + Cia402 drivers for both motors
ros2 launch maxon_epos4_ros2 bus_config_cia402_epos4_vel.launch.py

# 3. Run the application controller (publishes /robot_encoder_states, subscribes /robot_speed_cmd)
ros2 run epos4_controller epos4_controller --ros-args --params-file src/epos4_controller/config/params.yaml

# 4. Run the odometry node (publishes /odom and odom→base_link TF)
ros2 run epos4_controller epos4_odmetry --ros-args --params-file src/epos4_controller/config/params.yaml
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
        │ subscribes /motor{1,2}/cia402_device_{1,2}/joint_states
        ▼
[ros2_canopen Cia402Driver]  ── from external/maxon_epos4_ros2, launched separately
        │ CAN frames (SDO/PDO)
        ▼
[EPOS4 #1 + EPOS4 #2 over can0]

epos4_controller also re-publishes the joint_states from both motors as
sensor_msgs/JointState on /robot_encoder_states (name=[m1_wheel,m2_wheel]).

[epos4_odmetry] subscribes /robot_encoder_states, runs differential-drive
forward kinematics, and publishes nav_msgs/Odometry on /odom plus the
odom→base_link TF.
```

### Packages

- **`src/epos4_controller`** — application layer. Two executables share `config/params.yaml` (`tread_width`, `tire_diam`):
  - `epos4_controller` — owns the EPOS4 lifecycle (init → enable → cyclic_velocity_mode), converts `/robot_speed_cmd` into per-wheel target velocities, fans them out to both motors via the canopen TPDO topic, and aggregates encoder feedback into `/robot_encoder_states`.
  - `epos4_odmetry` — pure consumer of `/robot_encoder_states`. Computes 2D pose with mid-step heading integration; publishes `/odom` and broadcasts TF. Has parameters for frame names, TF on/off, and per-wheel direction inversion (`invert_left/right`).
- **`src/epos4_vel_ros2`** — standalone single-motor (and 2-motor) test programs. Useful for bench-bringing-up an EPOS4 without the full control stack.
- **`src/external/epos4compact50-5can`** — git submodule; vendors the `maxon_epos4_ros2` package. Its launch file (`bus_config_cia402_epos4_vel.launch.py`) is what wires `cia402_device_1` (node_id 1, namespace `/motor1`) and `cia402_device_2` (node_id 2, namespace `/motor2`) onto `can0`.

### Key conventions

- The two motors are addressed via the namespaces `/motor1/cia402_device_1` and `/motor2/cia402_device_2` (defined in `external/.../bus.yml`). Any new node that talks to a motor must follow this namespace pattern.
- `epos4_controller` drives the EPOS4 in **cyclic synchronous velocity mode** by writing target velocity (object 0x60FF, sub 0x00) into `COData` messages on the per-motor `tpdo` topic. The PDO cycle is 50 ms (set in `bus.yml`).
- Joint position/velocity from the canopen driver are scaled 1:1 with EPOS4 device units (`scale_*: 1.0` in `bus.yml`) — i.e. position is **encoder counts (qc)** and velocity is **rpm**, *not* SI units. Code that interprets `/joint_states` or `/robot_encoder_states` must account for this. The current `epos4_odmetry` assumes radians/rad-per-second; if used against the unmodified bus config, scaling factors must be added.
- `tread_width` and `tire_diam` are duplicated as ROS parameters in each consumer, sourced from `src/epos4_controller/config/params.yaml`. Keep these in sync if you change the chassis.
