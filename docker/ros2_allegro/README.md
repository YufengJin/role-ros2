# ros2_allegro — ROS 2 Humble + Allegro Hand V4 Stack

A self-contained Docker stack for the Wonik Robotics Allegro Hand V4. It
bundles the official `allegro_hand_ros2` driver (a `ros2_control` hardware
plugin + posture/grasp controllers) and a thin role-ros2 wrapper node so
downstream code talks to the hand the same way it talks to the Franka
gripper or xArm: namespaced topics, custom messages, blocking services.

## Architecture

```
+-----------------------------------------------------------------------+
| Container ros2_allegro_container  (ROS 2 Humble, ros2_control)        |
|                                                                       |
|  Official Wonik driver (C++)             role-ros2 wrapper (Python)   |
|  ┌──────────────────────────┐            ┌──────────────────────────┐ |
|  │ controller_manager       │            │ allegro_hand_interface_  │ |
|  │   joint_state_broadcaster│  ──────►   │   node                   │ |
|  │     /joint_states        │  filter    │     /{ns}/joint_states   │ |
|  │                          │  + relay   │     /{ns}/hand_state     │ |
|  │   allegro_hand_position_ │  ◄──────   │     /{ns}/joint_position_│ |
|  │     controller (FCC)     │  Float64   │       controller/command │ |
|  │   allegro_hand_posture_  │  ◄ action  │     /{ns}/reset          │ |
|  │     controller           │            │     /{ns}/move_to_joint_ │ |
|  │   allegro_hand_grasp_    │  ◄ action  │       positions          │ |
|  │     controller           │            │     /{ns}/grasp_primitive│ |
|  │                          │            │                          │ |
|  │   AllegroHandV4HardwareInterface (ros2_control SystemInterface)  │ |
|  └─────────────┬────────────┘            └──────────────────────────┘ |
|                │ SocketCAN                                            |
+----------------┼──────────────────────────────────────────────────────+
                 │ network_mode: host  (shares host's can0)
                 ▼
       Linux SocketCAN  ──►  USB-CAN dongle (PCAN-USB / gs_usb / kvaser_usb)
                                     │
                                     ▼
                                  Allegro Hand V4
```

Three things to remember:

1. **The official driver does the hard part.** It owns the CAN protocol,
   the 16-DoF state, posture/grasp primitives, and ros2_control lifecycle.
   The wrapper does **no** hardware I/O.
2. **The wrapper exists only to translate to role-ros2 conventions.** It
   filters / republishes `/joint_states` into `/{ns}/...`, converts
   `JointPositionCommand` → `Float64MultiArray`, and exposes the
   `GraspCommand` action as a blocking service.
3. **Mock mode is free** — set `use_mock:=true` and the launch file passes
   `ros2_control_hardware_type:=mock_components` to the official driver.
   The wrapper code path is unchanged; no Python mock is needed.

---

## Files in this directory

| File | Purpose |
|---|---|
| `Dockerfile` | Ubuntu 22.04 + ROS 2 Humble + full ros2_control stack + grasp_library deps + can-utils |
| `docker-compose.yaml` | Build context = repo root; `network_mode: host` (no `privileged`, no `/dev` mounts) |
| `entrypoint.sh` | Sources ROS, runs `colcon build` (skips Gazebo plugin), starts daemon, checks CAN |
| `ros2_allegro.env` | Env vars + status banner showing CAN state |
| `smoke_test.sh` | Hardware-free 5-step check: ros2_control_node, official packages, role_ros2 messages, wrapper import |

External, mutually referenced files (in the repo root):

| Path | Role |
|---|---|
| `allegro_hand_ros2/` | Official Wonik driver (vendored as a sub-tree; built by colcon) |
| `nodes/allegro_hand_interface_node.py` | role-ros2 wrapper executable |
| `launch/allegro_hand_robot.launch.py` | Launch — includes official bringup + wrapper + aggregator |
| `config/allegro_hand_config.yaml` | 16 joint names, namespace, prefix, home pose |
| `msg/HandState.msg`, `srv/GraspPrimitive.srv` | role-ros2 ROS interfaces specific to the hand |

---

## Hardware prerequisites

- **Allegro Hand V4** (the Plexus variant uses a different hardware plugin
  and is not enabled in this image).
- **USB-CAN dongle** on the host. Tested with **PEAK PCAN-USB** (`peak_usb`
  kernel module). Generic `gs_usb` and Kvaser dongles also work because the
  driver uses the kernel SocketCAN API, not a vendor userspace library.
- **Linux kernel with CAN support**: `CONFIG_CAN`, `CONFIG_CAN_RAW`,
  `CONFIG_CAN_DEV`, plus the dongle's driver module. Standard Ubuntu kernels
  ship these as modules.
- **No PEAK userspace driver, no realtime kernel needed.**

CAN setup happens **on the host, before `docker compose up`**. The container
shares `can0` via `network_mode: host`, so it does **not** need
`privileged: true` or any `/dev` passthrough.

```bash
sudo modprobe peak_usb            # or gs_usb / kvaser_usb
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
ip -details link show can0        # should print state ERROR-ACTIVE
candump can0 &                    # optional: confirm hand heartbeats
```

If you have **two** hands, plug in two dongles and bring both up:

```bash
sudo ip link set can0 up type can bitrate 1000000
sudo ip link set can1 up type can bitrate 1000000
```

(The current launch is single-hand; bimanual support is a planned extension
that will pass `prefix:=lh_/rh_` plus distinct CAN interfaces.)

---

## Build & run

All commands are run from the **repo root** (`role-ros2/`).

### 1. Build the image (one-time, ≈ 5–10 min)

```bash
docker compose -f docker/ros2_allegro/docker-compose.yaml build
```

The bulk of the build time is `apt install ros-humble-ros2-control` and
friends — there are no source builds for the driver itself; that happens at
container start via `colcon build`.

### 2. Start the container

```bash
docker compose -f docker/ros2_allegro/docker-compose.yaml up -d
```

First start runs `colcon build --symlink-install` over the bind-mounted
workspace. This builds:
- `allegro_hand_ros2/` (official driver, all its sub-packages except
  `allegro_hand_gazebo_plugin` which is skipped — Gazebo Harmonic is not
  installed in this image).
- `role_ros2/` (messages + wrapper node).

Total cold colcon build ≈ 3–4 min. Incremental rebuilds are seconds.

### 3. Enter the container

```bash
docker exec -it ros2_allegro_container bash
```

You will see a status banner including:

```
Hand device:          v4 (right)
Hand namespace:       allegro_hand
CAN interface:        can0 (state=ERROR-ACTIVE)
```

If `state=ABSENT` or `state=DOWN`, you forgot the host-side `ip link` step
(or you're going to launch in mock mode — that's also fine).

### 4. Smoke test (no real hand needed)

```bash
/smoke_test.sh
```

Validates that ros2_control_node is on PATH, all four official packages are
built, role_ros2's `HandState` and `GraspPrimitive` are registered, and the
wrapper module imports cleanly. Five checks should print `ok`.

---

## Configuration

`config/allegro_hand_config.yaml` is the single source of truth. The launch
file reads it and forwards values into the wrapper node as parameters.

| Key | Default | Notes |
|---|---|---|
| `device` | `v4` | Hardware version. Currently only `v4` is wired up. |
| `hand_side` | `right` | `right` or `left`; passed to the official launch's `hand:=` arg. |
| `hand_id` | `0` | CAN bus hand id. |
| `can_interface` | `can0` | Documentation only — actual CAN setup is on the host. |
| `namespace` | `allegro_hand` | Wrapper publishes / subscribes under `/{this}`. |
| `prefix` | `ah_` | URDF / joint name prefix used by the official driver. |
| `hand_joint_names` | 16 names | Must match `allegro_hand_ros2/.../v4/single_hand/ros2_controllers.yaml`. |
| `home_joints` | 16 zeros | Used by `move_to_joint_positions` and `auto_home_on_startup`. |
| `publish_rate` | `50.0` | Hz; wrapper republish rate for `hand_state`. |
| `use_mock` | `false` | `true` → launch sets `ros2_control_hardware_type:=mock_components`. |
| `auto_home_on_startup` | `false` | `true` → wrapper calls posture `"home"` after `auto_home_delay` s. |
| `auto_home_delay` | `5.0` | Seconds; lets the official controllers warm up first. |
| `move_tolerance` | `0.02` | Per-joint convergence tolerance (rad) for `move_to_joint_positions`. |
| `move_timeout` | `5.0` | Seconds; max wait in `move_to_joint_positions` and grasp services. |

Override any of these via launch args:

```bash
ros2 launch role_ros2 allegro_hand_robot.launch.py \
    namespace:=right_hand hand_side:=right
```

Or by editing the YAML and rebuilding (`colcon build --packages-select role_ros2`).

---

## Usage

### Launch — mock mode (no hardware needed)

```bash
ros2 launch role_ros2 allegro_hand_robot.launch.py use_mock:=true
```

The official driver instantiates `mock_components/GenericSystem`, which
echoes commanded positions back to `/joint_states`. The wrapper relays them
to `/{ns}/joint_states` and `/{ns}/hand_state`. Posture / grasp action
goals also succeed in mock mode (the official controllers don't actually
need hardware to compute targets).

### Launch — real hardware

```bash
# Host: bring CAN up first (one-time per boot)
sudo ip link set can0 up type can bitrate 1000000

# Container: launch
ros2 launch role_ros2 allegro_hand_robot.launch.py
```

This brings up:
1. The official `allegro_hand.launch.py` (controller_manager + URDF +
   `joint_state_broadcaster` + IMU broadcaster + position/posture/grasp
   controllers + diagnostic_aggregator + RViz).
2. `allegro_hand_interface_node` in `/{namespace}` — the role-ros2 wrapper.
3. `robot_state_aggregator_node` — merges `/{ns}/joint_states` into root
   `/joint_states` for downstream `RobotEnv` consumers.

### Topics (in `/{namespace}`)

| Topic | Direction | Type |
|---|---|---|
| `joint_states` | published, 50 Hz | `sensor_msgs/JointState` (16 hand joints, ordered per `hand_joint_names`) |
| `hand_state` | published, 50 Hz | `role_ros2/HandState` (positions/velocities/efforts/temperatures + last grasp action result) |
| `controller_status` | published, 50 Hz | `role_ros2/ControllerStatus` (always reports `joint_position` mode) |
| `joint_position_controller/command` | subscribed | `role_ros2/JointPositionCommand` (16-DoF positions; relayed to the official FCC) |

### Services (in `/{namespace}`)

| Service | Type | Behaviour |
|---|---|---|
| `reset` | `role_ros2/Reset` | Calls posture action with `command="home"`, blocks until reached. |
| `move_to_joint_positions` | `role_ros2/MoveToJointPositions` | Sends 16-DoF target via FCC, polls `/joint_states` until \|err\|≤`move_tolerance` (or `move_timeout`). |
| `grasp_primitive` | `role_ros2/GraspPrimitive` | Routes `command` to posture (`home`/`ready`/`off`) or grasp (`grasp_3`/`grasp_4`/`pinch_it`/`pinch_mt`/`envelop`) action. Returns `stalled`/`reached_goal` from the action result. |

### Example commands

```bash
# State stream
ros2 topic echo /allegro_hand/hand_state

# 16-DoF position (open hand)
ros2 topic pub --once /allegro_hand/joint_position_controller/command \
    role_ros2/msg/JointPositionCommand \
    "{positions: [0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0], blocking: false}"

# Blocking joint move with custom timeout
ros2 service call /allegro_hand/move_to_joint_positions \
    role_ros2/srv/MoveToJointPositions \
    "{joint_positions: [0.0,0.5,0.5,0.5, 0.0,0.5,0.5,0.5, 0.0,0.5,0.5,0.5, 1.0,0.5,0.5,0.5], time_to_go: 3.0}"

# Posture primitive — open
ros2 service call /allegro_hand/reset role_ros2/srv/Reset "{}"

# Grasp primitive — envelop
ros2 service call /allegro_hand/grasp_primitive role_ros2/srv/GraspPrimitive \
    "{command: 'envelop', effort: 0.5, blocking: true}"

# Direct call into the official action (bypasses the wrapper)
ros2 action send_goal /allegro_hand_grasp_controller/grasp_cmd \
    allegro_hand_control_msgs/action/GraspCommand "{command: 'grasp_4', effort: 0.3}"
```

### RViz

The official launch starts RViz with a hand visualization preset. Set
`DISPLAY` and run `xhost +local:root` on the host once per session.

---

## Environment variables

Read by `docker-compose.yaml` and `ros2_allegro.env`:

| Variable | Default | Purpose |
|---|---|---|
| `CAN_INTERFACE` | `can0` | Used by the entrypoint health check + status banner. |
| `HAND_SIDE` | `right` | Default for launch arg `hand_side`. |
| `HAND_DEVICE` | `v4` | Default for launch arg `device`. |
| `HAND_NAMESPACE` | `allegro_hand` | Default for launch arg `namespace`. |
| `ROS_DOMAIN_ID` | `0` | Must match across all containers/peers. |
| `RMW_IMPLEMENTATION` | `rmw_fastrtps_cpp` | Pinned for stability. |
| `FASTDDS_BUILTIN_TRANSPORTS` | `UDPv4` | Avoids known SHM crashes inside Docker. |
| `ROS_LOCALHOST_ONLY` | `0` | `1` to confine to the host. |
| `DISPLAY` | (host) | Required for RViz. |
| `SKIP_AUTO_ENV` | `false` | `true` to disable automatic env sourcing. |

Example:

```bash
HAND_SIDE=left HAND_NAMESPACE=left_hand CAN_INTERFACE=can1 \
    docker compose -f docker/ros2_allegro/docker-compose.yaml up -d
```

---

## Co-running with other stacks

Because every stack uses `network_mode: host`, the Allegro container can
publish on the same DDS bus as a Franka, xArm, or ZED container as long as
they share `ROS_DOMAIN_ID` and `RMW_IMPLEMENTATION`. A typical bimanual
hand+arm setup spawns one Franka container and one Allegro container per
arm with distinct namespaces.

---

## Troubleshooting

### Entrypoint warns: `CAN interface 'can0' is DOWN`

Bring it up on the host (not inside the container):

```bash
sudo ip link set can0 up type can bitrate 1000000
ip -details link show can0
```

If the interface itself doesn't exist (`ABSENT`), the kernel module isn't
loaded — `sudo modprobe peak_usb` (or your dongle's driver).

### `controller_manager: Could not load class 'allegro_hand_v4_hardware/AllegroHandV4HardwareInterface'`

The colcon build silently failed. Rebuild with verbose output:

```bash
cd /app/ros2_ws
colcon build --packages-up-to allegro_hand_v4_hardware --event-handlers console_direct+
```

Common causes:
- Missing system deps (`libspdlog-dev`, `libyaml-cpp-dev`, `libboost-all-dev`,
  `ros-humble-pinocchio`). All are installed in the image; this only matters
  if you customised the Dockerfile.
- Stale `build/` / `install/` from a previous failed attempt — `rm -rf
  build install log` and rebuild.

### `/joint_states` is empty after launch

Likely the controller_manager couldn't load the hardware interface or
couldn't open the CAN socket. Check:

```bash
ros2 control list_hardware_components
ros2 control list_controllers
ros2 node list
```

If `joint_state_broadcaster` is `inactive` or absent, the hardware load
failed — see the previous section. If it's `active` but `/joint_states` is
silent, run `candump can0` on the host to verify the hand is actually
talking on the bus.

### `grasp_primitive` returns `success: false, message: 'goal accept timed out'`

The corresponding action server isn't up:

```bash
ros2 action list | grep grasp
```

Both `/allegro_hand_posture_controller/grasp_cmd` and
`/allegro_hand_grasp_controller/grasp_cmd` should be present. If they
aren't, the relevant controllers failed to spawn — check
`ros2 control list_controllers` and the controller_manager logs.

### `move_to_joint_positions` never converges

Two common causes:

1. **The hand is stuck against an obstacle** — the joint will never reach
   the target. Loosen `move_tolerance` or adjust the target.
2. **The target is outside the joint limits** — the FCC will accept any
   position, but the hardware will saturate at its limit. Watch
   `/joint_states` to see where the joint actually ends up.

### `ros2 topic list` is empty after launch

The ROS 2 daemon is flaky inside containers. Restart it:

```bash
ros2 daemon stop && ros2 daemon start
```

If still empty, verify `ROS_DOMAIN_ID` and `RMW_IMPLEMENTATION` match your
other peers.

### Wrapper ignores commands silently

Likely a length mismatch — check the log for `Ignoring
joint_position_controller/command: got N positions, expected 16`. Allegro V4
is strictly 16-DoF; partial commands are rejected.

### `colcon build` fails on `allegro_hand_gazebo_plugin`

The entrypoint already passes `--packages-skip allegro_hand_gazebo_plugin`
because Gazebo Harmonic isn't installed in the image. If you ran colcon
manually, add the same flag:

```bash
colcon build --symlink-install --packages-skip allegro_hand_gazebo_plugin
```

### Two Allegro hands on the same host?

This launch is single-hand only. Bimanual support is a planned extension
(switch to `bimanual_franka`-style topology with two namespaces, each with
its own `prefix`, `can_interface`, and joint-name set; URDF already
supports per-hand `prefix`).
