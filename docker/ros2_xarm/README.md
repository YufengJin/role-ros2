# ros2_xarm — ROS 2 Humble + xArm Stack

A standalone Docker stack that runs UFACTORY's `xarm-python-sdk` together with
ROS 2 Humble and the role-ros2 wrapper node `xarm_robot_interface_node`. The
stack exposes the same role-ros2 conventions (namespaced topics, custom
messages, blocking services) as the Franka stack, so downstream policy /
teleop / data-collection code works uniformly across robots.

```
              role-ros2 conventions
+---------------------------------------+
|  xarm_robot_interface_node            |
|    /xarm6_arm/joint_states            |
|    /xarm6_arm/arm_state               |
|    /xarm6_arm/ee_pose                 |
|    /xarm6_arm/joint_position_controller/command
|    /xarm6_arm/cartesian_position_controller/command
|    /xarm6_arm/move_to_joint_positions  (srv)
|    /xarm6_arm/move_to_ee_pose          (srv)
|    /xarm6_arm/reset                    (srv)
|    /xarm6_arm/start_joint_impedance    (srv)
|    /xarm6_arm/start_cartesian_impedance(srv)
|    ...                                |
+-------------------+-------------------+
                    |  xarm-python-sdk
                    v
              xArm controller box (TCP @ 192.168.1.x)
                    |
                    v
                xArm6 robot
```

The image is intentionally lightweight: no CUDA, no ZED, no source builds.
`xarm-python-sdk` is installed via pip on the same Python interpreter that
ROS 2 Humble uses (`/usr/bin/python3` → 3.10), so `import rclpy` and
`from xarm.wrapper import XArmAPI` coexist in one process.

---

## Files in this directory

| File | Purpose |
|---|---|
| `Dockerfile` | Ubuntu 22.04 + ROS 2 Humble + `xarm-python-sdk` (pip) |
| `docker-compose.yaml` | Build context = repo root; `network_mode: host` for DDS + xArm TCP |
| `entrypoint.sh` | Sources ROS, runs `colcon build`, starts ROS 2 daemon |
| `ros2_xarm.env` | Env vars sourced on shell entry; prints a status banner |
| `smoke_test.sh` | 5-step smoke test (no real robot needed) |
| `tests/test_ros2.py`, `tests/test_xarm_sdk.py`, `tests/test_combined.py` | Used by `smoke_test.sh` |

External, mutually referenced files (in the repo root):

| Path | Role |
|---|---|
| `nodes/xarm_robot_interface_node.py` | Wrapper node executable |
| `launch/xarm_robot.launch.py` | Launch file — wrapper + aggregator + RSP |
| `config/xarm_robot_config.yaml` | Joint names, IP, namespace, home pose |
| `role_ros2/robot_ik/xarm/xarm6.urdf` | URDF used by `robot_state_publisher` |
| `msg/`, `srv/` | role-ros2 ROS interfaces (built by colcon) |

---

## Hardware prerequisites

- xArm6 (or compatible UFACTORY arm) connected to the host's LAN.
- The xArm controller box reachable on `${ROBOT_IP}` (default `192.168.1.185`).
  Verify with `ping ${ROBOT_IP}` from the host.
- No USB/CAN dongles needed — xArm communicates over TCP.

For the official robot setup (power, e-stop, network), follow UFACTORY's
documentation. Once you can `ping` the controller box from the host, the
container will be able to talk to it.

---

## Build & run

All commands are run from the **repo root** (`role-ros2/`).

### 1. Build the image (one-time, ≈ 3 min)

```bash
docker compose -f docker/ros2_xarm/docker-compose.yaml build
```

### 2. Start the container

```bash
docker compose -f docker/ros2_xarm/docker-compose.yaml up -d
```

The first start runs `colcon build --symlink-install` over the bind-mounted
`role-ros2/` workspace. Subsequent starts incrementally rebuild whatever
changed.

### 3. Enter the container

```bash
docker exec -it ros2_xarm_container bash
```

You will see a status banner from `ros2_xarm.env` listing
`ROS2 Distribution`, `xarm-python-sdk` version, `ROBOT_IP`, etc.

### 4. Smoke test (no real robot needed)

```bash
/smoke_test.sh
```

This validates that ROS 2 + `xarm-python-sdk` + role-ros2 messages all build
and import cleanly. Five checks should print `ok`.

---

## Configuration

`config/xarm_robot_config.yaml` is the single source of truth. The launch
file reads it and forwards values into the wrapper node as parameters.

| Key | Default | Notes |
|---|---|---|
| `robot_ip` | `192.168.1.185` | xArm controller box IP |
| `arm_namespace` | `xarm6_arm` | Wrapper publishes / subscribes under `/{this}` |
| `arm_joints` | 6 names | Must match the URDF |
| `ee_frame_id` | `xarm6_link_base` | Frame published in `arm_state` / `ee_pose` |
| `urdf_file` | `xarm6.urdf` | Loaded by `robot_state_publisher` |
| `use_mock` | `false` | `true` → `MockXArmInterface` (no hardware) |
| `publish_rate` | `50.0` | Hz |
| `home_joints` | `[0]*6` | Drive target on `auto_reset_on_startup` |
| `auto_reset_on_startup` | `false` | Move to `home_joints` after `auto_reset_delay` s |
| `max_joint_velocity` | `1.0` | rad/s — used to derive SDK `speed` |

Override any of these via launch args or by editing the YAML and running
`colcon build --packages-select role_ros2 --symlink-install` again
(symlink install means YAML edits don't require a rebuild for nodes that
already loaded it).

---

## Usage

### Launch (real hardware)

```bash
ros2 launch role_ros2 xarm_robot.launch.py
```

This brings up three nodes (mirroring the Franka launch):

1. `xarm_robot_interface_node` (in `/{arm_namespace}`) — wraps `XArmAPI`.
2. `robot_state_aggregator_node` — merges `/{ns}/joint_states` into root
   `/joint_states`.
3. `robot_state_publisher` — publishes `/tf` from URDF + `/joint_states`.

### Launch (mock — no robot needed)

```bash
ros2 launch role_ros2 xarm_robot.launch.py use_mock:=true
```

`MockXArmInterface` simulates an integrator-based 6-DoF state machine.
Useful for CI, RViz layout work, and downstream policy debugging.

### Multi-robot (different namespace + IP)

```bash
ros2 launch role_ros2 xarm_robot.launch.py \
    arm_namespace:=robot1_arm \
    robot_ip:=192.168.1.211
```

Run once per robot in separate terminals (or separate compose services with
distinct `container_name` and an explicit `ROS_DOMAIN_ID`).

### Topics (in `/{arm_namespace}`)

| Topic | Direction | Type |
|---|---|---|
| `joint_states` | published, 50 Hz | `sensor_msgs/JointState` |
| `arm_state` | published, 50 Hz | `role_ros2/ArmState` |
| `ee_pose` | published, 50 Hz | `geometry_msgs/PoseStamped` |
| `controller_status` | published, 50 Hz | `role_ros2/ControllerStatus` |
| `joint_position_controller/command` | subscribed | `role_ros2/JointPositionCommand` |
| `joint_velocity_controller/command` | subscribed | `role_ros2/JointVelocityCommand` |
| `cartesian_position_controller/command` | subscribed | `role_ros2/CartesianPositionCommand` |
| `cartesian_velocity_controller/command` | subscribed | `role_ros2/CartesianVelocityCommand` |

### Services (in `/{arm_namespace}`)

| Service | Type | Purpose |
|---|---|---|
| `reset` | `role_ros2/Reset` | Move to `home_joints` |
| `move_to_joint_positions` | `role_ros2/MoveToJointPositions` | Blocking joint-space move |
| `move_to_ee_pose` | `role_ros2/MoveToEEPose` | Blocking Cartesian move |
| `start_joint_impedance` | `role_ros2/StartJointImpedance` | Switch controller mode |
| `start_cartesian_impedance` | `role_ros2/StartCartesianImpedance` | Switch controller mode |
| `start_joint_velocity` | `role_ros2/StartJointVelocity` | Switch controller mode |
| `terminate_policy` | `role_ros2/TerminatePolicy` | Stop the active policy |
| `solve_ik` | `role_ros2/SolveIK` | IK utility |
| `compute_fk` | `role_ros2/ComputeFK` | FK utility |
| `compute_time_to_go` | `role_ros2/ComputeTimeToGo` | Trajectory timing |

### Example commands

```bash
# Stream a joint-position setpoint (non-blocking)
ros2 topic pub --once /xarm6_arm/joint_position_controller/command \
    role_ros2/msg/JointPositionCommand \
    "{positions: [0.1, 0.0, 0.0, 0.0, 0.0, 0.0], blocking: false}"

# Blocking joint-space move with explicit time-to-go
ros2 service call /xarm6_arm/move_to_joint_positions \
    role_ros2/srv/MoveToJointPositions \
    "{joint_positions: [0,0,0,0,0,0], time_to_go: 2.0}"

# Reset to home pose
ros2 service call /xarm6_arm/reset role_ros2/srv/Reset "{}"

# Watch state stream
ros2 topic echo /xarm6_arm/arm_state
```

---

## Environment variables

Read by `docker-compose.yaml` and `ros2_xarm.env`:

| Variable | Default | Purpose |
|---|---|---|
| `ROBOT_IP` | `192.168.1.222` | xArm controller IP (env override of YAML default) |
| `ROS_DOMAIN_ID` | `0` | Must match across all containers/peers |
| `RMW_IMPLEMENTATION` | `rmw_fastrtps_cpp` | Pinned for stability |
| `FASTDDS_BUILTIN_TRANSPORTS` | `UDPv4` | Avoids known SHM crashes inside Docker |
| `ROS_LOCALHOST_ONLY` | `0` | `1` to confine to the host (no LAN peers) |
| `DISPLAY` | (host) | Required for RViz / rqt |
| `SKIP_AUTO_ENV` | `false` | `true` to disable automatic env sourcing |

Set these on the `docker compose` invocation, e.g.:

```bash
ROBOT_IP=192.168.1.211 ROS_DOMAIN_ID=7 \
    docker compose -f docker/ros2_xarm/docker-compose.yaml up -d
```

---

## Co-running with other stacks

Because every stack uses `network_mode: host`, multiple containers (xArm,
Franka, Allegro, ZED) can publish to the same DDS bus as long as they share
`ROS_DOMAIN_ID` and `RMW_IMPLEMENTATION`. Use distinct `arm_namespace` /
`gripper_namespace` / `namespace` per robot to avoid topic collisions.

---

## Troubleshooting

### `ping ${ROBOT_IP}` fails from the container

`network_mode: host` shares the host's networking, so this is identical to
pinging from the host. Check cabling, the xArm controller's IP, and any
firewalls.

### `XArmAPI(...)` hangs at startup

Usually one of:
- Wrong IP in `xarm_robot_config.yaml`.
- xArm controller box not powered or in error state — check the teach pendant
  / web UI.
- A second client is already connected — xArm controllers serialise client
  connections.

### `ros2 topic list` is empty after launch

The ROS 2 daemon is flaky inside containers. Restart it:

```bash
ros2 daemon stop && ros2 daemon start
```

If still empty, verify `ROS_DOMAIN_ID` and `RMW_IMPLEMENTATION` match your
other peers (`echo $ROS_DOMAIN_ID $RMW_IMPLEMENTATION`).

### `colcon build` fails after pulling new role-ros2 changes

Clean and rebuild:

```bash
cd /app/ros2_ws
rm -rf build install log
colcon build --symlink-install
source install/setup.bash
```

### Mock mode behaves unrealistically

`MockXArmInterface` is a simple integrator — it does not enforce joint limits
or singularities. For policy testing in mock mode, clamp commanded positions
to `home ± reasonable_delta` yourself.
