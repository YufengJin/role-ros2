# Scripts Documentation

This directory contains Python scripts for robot trajectory collection and replay.

## Overview

- **`collect_trajectory.py`**: Collect robot trajectories using VR controller (Oculus Quest)
- **`replay_trajectory.py`**: Replay saved robot trajectories from HDF5 files

Both scripts are pure Python scripts (not ROS2 nodes) that use `RobotEnv` internally for robot control.

---

## 📝 collect_trajectory.py

### Description

Collect robot trajectories using an Oculus Quest VR controller. The script provides two modes:

1. **Teleoperation Mode**: Control the robot without saving data (default when no `--task` is provided)
2. **Recording Mode**: Automatically enabled when `--task` is provided, saves trajectories to HDF5 files

### Features

- Real-time robot control via VR controller
- Automatic trajectory saving when task name is provided
- Long-press button controls for SUCCESS/FAILURE marking
- Continuous loop mode (no single trajectory limit)
- Automatic robot reset after trajectory completion
- Support for multiple action spaces (cartesian_velocity, joint_velocity, etc.)

### Prerequisites

- ROS2 environment initialized
- Oculus Quest controller connected and configured
- Robot hardware (Franka Panda) connected via Polymetis
- `polymetis_bridge_node` running
- `oculus_reader_node` running (if using VR controller)

### Usage

#### Basic Usage

```bash
# Teleoperation only (no saving)
python3 collect_trajectory.py

# Save trajectory with task name (auto-enables saving)
python3 collect_trajectory.py --task pick_and_place
```

#### Full Options

```bash
python3 collect_trajectory.py \
    --task pick_and_place \
    --save-folder /path/to/save \
    --save-images \
    --save-depths \
    --action-space cartesian_velocity \
    --control-hz 15.0 \
    --right-controller \
    --reset-robot \
    --randomize-reset
```

### Arguments

| Argument | Type | Default | Description |
|----------|------|---------|-------------|
| `--task` | str | `''` | Task name. If provided, automatically enables saving to `save_folder/task_name/` |
| `--save-folder` | str | `/app/ros2_ws/src/role-ros2/data` | Base folder to save trajectory files |
| `--save-images` | flag | `False` | Save RGB images in trajectory files (MP4 video) |
| `--save-depths` | flag | `False` | Save depth images (PNG-in-HDF5, lossless). Requires `--save-images`. |
| `--action-space` | str | `cartesian_velocity` | Action space: `cartesian_velocity`, `cartesian_position`, `joint_velocity`, `joint_position` |
| `--control-hz` | float | `15.0` | Control frequency in Hz |
| `--wait-for-controller` | flag | `True` | Wait for controller movement before executing actions |
| `--no-wait-for-controller` | flag | - | Disable waiting for controller movement |
| `--right-controller` | flag | `True` | Use right controller (default) |
| `--left-controller` | flag | - | Use left controller |
| `--reset-robot` | flag | `True` | Reset robot to home position on startup |
| `--no-reset-robot` | flag | - | Skip robot reset on startup |
| `--randomize-reset` | flag | `False` | Add random offset to home position on reset |
| `--horizon` | int | `-1` | Maximum steps per trajectory (-1 for unlimited) |

### Control Instructions

| Action | Description |
|--------|-------------|
| **Hold GRIP button** | Enable robot movement |
| **Long press A/X** (0.5s) | **SUCCESS**: Save trajectory, reset robot, start new trajectory |
| **Long press B/Y** (0.5s) | **FAILURE**: Discard trajectory, reset robot, start new trajectory |
| **Ctrl+C** | Exit program (saves current trajectory if interrupted) |

### Workflow

1. **Initialization**:
   - Initialize `RobotEnv` and `VRPolicy` (controller)
   - Reset robot to home position (if `--reset-robot` is set)
   - Start new trajectory recording (if `--task` is provided)

2. **Control Loop** (runs at `--control-hz` frequency):
   - Get observation from `RobotEnv`
   - Get action from `VRPolicy` (controller)
   - Execute action on robot
   - Save timestep data (if recording mode)

3. **Trajectory Completion**:
   - **SUCCESS** (long press A/X): Save trajectory, reset robot, start new trajectory
   - **FAILURE** (long press B/Y): Discard trajectory, reset robot, start new trajectory
   - **Horizon reached**: Save trajectory, reset robot, start new trajectory
   - **Ctrl+C**: Save interrupted trajectory, shutdown

### Output

Trajectories are saved as HDF5 files with the following structure:

```
save_folder/
└── task_name/
    ├── trajectory_20240105_143022.h5
    ├── trajectory_20240105_143155.h5
    └── ...
```

Each trajectory file contains:
- `observation/`: Robot state, camera images, timestamps
- `action/`: Actions in multiple action spaces
- `metadata/`: Task name, trajectory ID, success/failure status

### Examples

#### Example 1: Basic Teleoperation

```bash
# Just control the robot, no saving
python3 collect_trajectory.py
```

#### Example 2: Collect Pick and Place Trajectories

```bash
python3 collect_trajectory.py \
    --task pick_and_place \
    --save-images \
    --control-hz 20.0
```

#### Example 3: Custom Save Location

```bash
python3 collect_trajectory.py \
    --task assembly \
    --save-folder /data/robot_trajectories \
    --action-space joint_velocity \
    --control-hz 10.0
```

---

## ▶️ replay_trajectory.py

### Description

Replay saved robot trajectories from HDF5 files. The script:

1. Loads trajectory data from an HDF5 file
2. Moves the robot to the initial position
3. Replays recorded actions at the specified control frequency
4. Supports speed adjustment via `--speed-factor`

### Features

- Automatic initial position alignment
- Configurable replay speed
- Support for multiple action spaces
- Progress logging
- Graceful error handling

### Prerequisites

- ROS2 environment initialized
- Robot hardware (Franka Panda) connected via Polymetis
- `polymetis_bridge_node` running
- Valid trajectory HDF5 file

### Usage

#### Basic Usage

```bash
python3 replay_trajectory.py --filepath /path/to/trajectory.h5
```

#### With Options

```bash
python3 replay_trajectory.py \
    --filepath /path/to/trajectory.h5 \
    --action-space cartesian_velocity \
    --control-hz 15.0 \
    --speed-factor 1.5
```

### Arguments

| Argument | Type | Default | Description |
|----------|------|---------|-------------|
| `--filepath` | str | **Required** | Path to trajectory HDF5 file |
| `--action-space` | str | `cartesian_velocity` | Action space: `cartesian_velocity`, `cartesian_position`, `joint_velocity`, `joint_position` |
| `--control-hz` | float | `15.0` | Control frequency in Hz |
| `--speed-factor` | float | `1.0` | Speed multiplier (>1.0 for faster, <1.0 for slower) |

### Workflow

1. **Initialization**:
   - Load trajectory from HDF5 file
   - Initialize `RobotEnv`
   - Move robot to initial position from trajectory

2. **Replay Loop** (runs at `--control-hz` / `--speed-factor` frequency):
   - Read next timestep from trajectory
   - Extract action in desired action space
   - Execute action on robot
   - Log progress every 50 steps

3. **Completion**:
   - Replay completes when all steps are executed
   - Robot remains at final position

### Examples

#### Example 1: Basic Replay

```bash
python3 replay_trajectory.py \
    --filepath /app/ros2_ws/src/role-ros2/data/pick_and_place/trajectory_20240105_143022.h5
```

#### Example 2: Fast Replay (1.5x speed)

```bash
python3 replay_trajectory.py \
    --filepath /path/to/trajectory.h5 \
    --speed-factor 1.5 \
    --control-hz 20.0
```

#### Example 3: Slow Replay (0.5x speed) with Joint Position

```bash
python3 replay_trajectory.py \
    --filepath /path/to/trajectory.h5 \
    --action-space joint_position \
    --speed-factor 0.5 \
    --control-hz 10.0
```

---

## 🔧 Common Issues

### collect_trajectory.py

#### Issue: Controller not detected

**Solution**: Ensure `oculus_reader_node` is running:
```bash
ros2 run role_ros2 oculus_reader_node
```

#### Issue: Robot not moving

**Solution**: 
- Check that `polymetis_bridge_node` is running
- Ensure GRIP button is held down
- Check `--wait-for-controller` setting

#### Issue: Trajectory not saving

**Solution**: 
- Ensure `--task` argument is provided
- Check that `--save-folder` is writable
- Verify disk space

### replay_trajectory.py

#### Issue: File not found

**Solution**: Use absolute path or ensure working directory is correct:
```bash
python3 replay_trajectory.py --filepath $(pwd)/trajectory.h5
```

#### Issue: Action space mismatch

**Solution**: Ensure `--action-space` matches the action space used during collection, or the trajectory file contains the desired action space.

#### Issue: Robot doesn't reach initial position

**Solution**: 
- Check robot workspace limits
- Verify trajectory file is valid
- Check robot connection status

---

## 📊 Trajectory File Structure

Trajectory HDF5 files contain the following structure:

```
trajectory.h5
├── observation/
│   ├── robot_state/
│   │   ├── joint_positions (N, 7)
│   │   ├── joint_velocities (N, 7)
│   │   ├── ee_position (N, 3)
│   │   ├── ee_orientation (N, 4)
│   │   └── gripper_position (N,)
│   ├── images/
│   │   ├── camera_id_1/
│   │   │   ├── rgb (N, H, W, 3)
│   │   │   └── depth (N, H, W)
│   │   └── camera_id_2/...
│   ├── camera_intrinsics/
│   ├── camera_extrinsics/
│   └── timestamp/
├── action/
│   ├── cartesian_velocity (N, 6)
│   ├── cartesian_position (N, 6)
│   ├── joint_velocity (N, 7)
│   ├── joint_position (N, 7)
│   ├── gripper_velocity (N,)
│   └── gripper_position (N,)
└── metadata/ (file-level attributes)
    ├── task_name
    ├── trajectory_id
    ├── success
    └── failure
```

Where `N` is the number of timesteps in the trajectory.

---

## 🔗 Related Documentation

- `role_ros2/robot_env.py`: Robot environment interface
- `role_ros2/controllers/oculus_controller.py`: VR controller implementation
- `role_ros2/trajectory_utils/trajectory_reader.py`: Trajectory reading utilities
- `role_ros2/trajectory_utils/trajectory_writer.py`: Trajectory writing utilities

---

## 📝 Notes

- Both scripts use `RobotEnv`'s internal `MultiThreadedExecutor` for ROS2 message processing
- The main thread only handles control loop timing and user interaction
- All timestamps use ROS nanoseconds for consistency
- Trajectory files are compatible with the DROID dataset format

---

## 🐛 Troubleshooting

### Check ROS2 Nodes

```bash
# List running nodes
ros2 node list

# Check topics
ros2 topic list

# Check services
ros2 service list
```

### Verify Trajectory File

Use `misc/hdf5_reader.py` to inspect trajectory files:

```bash
python3 misc/hdf5_reader.py /path/to/trajectory.h5
```

### Enable Debug Logging

Set ROS2 log level:
```bash
export RCUTILS_LOGGING_SEVERITY=DEBUG
```

---

## 📧 Support

For issues or questions, please refer to the main project documentation or contact the Role-ROS2 team.
