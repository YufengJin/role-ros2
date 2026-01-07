# Scripts Documentation

This directory contains Python scripts for robot trajectory collection, replay, and camera calibration.

## Overview

- **`collect_trajectory.py`**: Collect robot trajectories using VR controller (Oculus Quest)
- **`replay_trajectory.py`**: Replay saved robot trajectories from HDF5 files
- **`calibrate_camera.py`**: Perform hand-eye calibration for cameras using Charuco board

All scripts are pure Python scripts (not ROS2 nodes) that use `RobotEnv` internally for robot control.

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

## 📷 calibrate_camera.py

### Description

Perform hand-eye calibration for cameras using a Charuco board. The script supports two calibration modes:

1. **Hand Camera Calibration** (`--mode hand`): Calibrate hand-mounted camera (camera → gripper link)
2. **Third-Person Camera Calibration** (`--mode third`): Calibrate static camera (camera → base_link)

The calibration process:
1. Resets robot to home position
2. Waits for user to position robot facing Charuco board using VR controller
3. Automatically executes calibration trajectory while collecting images
4. Evaluates calibration accuracy
5. Publishes static TF transform and saves results to YAML file

### Features

- Automatic calibration trajectory execution
- Real-time Charuco board detection visualization
- Calibration accuracy evaluation (linear and rotation error)
- Static TF transform publishing
- Results saved to YAML configuration file
- Support for recalibration if accuracy is insufficient

### Prerequisites

- ROS2 environment initialized
- Oculus Quest controller connected and configured
- Robot hardware (Franka Panda) connected via Polymetis
- Camera driver running (ZED cameras)
- `polymetis_bridge_node` running
- `oculus_reader_node` running
- Charuco board (9x14, checker size: 0.0285m, marker size: 0.0215m)
- Camera configuration in `config/multi_camera_reader_config.yaml` with:
  - `camera_frame`: Camera optical frame name
  - `camera_base_frame`: Camera base frame name
  - `camera_base_parent_frame`: Parent frame for calibration (e.g., `base_link` or `fr3_panda_link8`)

### Usage

#### Basic Usage

```bash
# Calibrate static camera
python3 calibrate_camera.py --camera_id 24285872 --mode third

# Calibrate hand camera
python3 calibrate_camera.py --camera_id 11022812 --mode hand
```

#### With Options

```bash
python3 calibrate_camera.py \
    --camera_id 24285872 \
    --mode third \
    --output ../config/calibration_results.yaml \
    --step_size 0.01 \
    --pause_time 0.5 \
    --image_freq 10 \
    --right-controller
```

### Arguments

| Argument | Type | Default | Description |
|----------|------|---------|-------------|
| `--camera_id` | str | **Required** | Camera identifier (serial number) |
| `--mode` | str | **Required** | Calibration mode: `hand` or `third` |
| `--output` | str | `../config/calibration_results.yaml` | Output file path for calibration results |
| `--step_size` | float | `0.01` | Trajectory step size (smaller = more samples) |
| `--pause_time` | float | `0.5` | Pause time before capturing image (seconds) |
| `--image_freq` | int | `10` | Take image every N steps |
| `--right-controller` | flag | `True` | Use right VR controller (default) |
| `--left-controller` | flag | - | Use left VR controller |

### Control Instructions

| Action | Description |
|--------|-------------|
| **Hold GRIP button** | Enable robot movement (during positioning phase) |
| **Long press A/X** (0.5s) | **START**: Begin calibration trajectory |
| **Long press A/X** (0.5s) | **ACCEPT**: Accept calibration, publish TF, save results |
| **Long press B/Y** (0.5s) | **REJECT**: Reject calibration, reset robot, recalibrate |
| **Ctrl+C** | Exit program |

### Workflow

1. **Initialization**:
   - Load camera configuration from `multi_camera_reader_config.yaml`
   - Initialize `RobotEnv` and `VRPolicy` (controller)
   - Get camera intrinsics
   - Lookup fixed TF transform (camera_base_frame → camera_frame)

2. **Positioning Phase**:
   - Reset robot to home position
   - User moves robot with VR controller to face Charuco board
   - Press A/X (long press) to start calibration

3. **Calibration Trajectory**:
   - Robot automatically moves through predefined trajectory
   - Images captured at regular intervals
   - Charuco board detection visualized in real-time
   - Progress displayed (percentage and sample count)

4. **Evaluation**:
   - Calibration accuracy checked (linear and rotation error)
   - Transformation computed (camera_base_frame → camera_base_parent_frame)
   - Results displayed

5. **User Decision**:
   - **ACCEPT** (A/X): Publish static TF, save to YAML file
   - **REJECT** (B/Y): Reset robot, return to positioning phase

### Output

Calibration results are saved to YAML file with the following structure:

```yaml
cameras:
- camera_id: "24285872"
  rgb_topic: "/static_camera/zed_node/rgb/image_rect_color"
  child_frame: "static_camera_left_camera_frame"
  parent_frame: "base_link"
  transform:
    x: 0.1234
    y: -0.5678
    z: 0.9012
    rx: 0.0123
    ry: -0.0456
    rz: 0.0789
- camera_id: "11022812"
  rgb_topic: "/hand_camera/zed_node/rgb/image_rect_color"
  child_frame: "hand_camera_left_camera_frame"
  parent_frame: "fr3_panda_link8"
  transform:
    x: 0.0
    y: 0.0
    z: 0.0
    rx: 0.0
    ry: 0.0
    rz: 0.0
```

The script also publishes a static TF transform using `ros2 run tf2_ros static_transform_publisher`.

### Transform Computation

The calibration process computes the transform chain:

1. **Calibrator output**: `T_optical_to_parent` (camera_frame → camera_base_parent_frame)
2. **TF lookup**: `T_base_to_optical` (camera_base_frame → camera_frame) [fixed, queried once]
3. **Final result**: `T_base_to_parent = T_optical_to_parent @ T_base_to_optical`

The final transform (`T_base_to_parent`) is published as static TF and saved to the YAML file.

### Examples

#### Example 1: Calibrate Static Camera

```bash
python3 calibrate_camera.py \
    --camera_id 24285872 \
    --mode third
```

#### Example 2: Calibrate Hand Camera with Custom Parameters

```bash
python3 calibrate_camera.py \
    --camera_id 11022812 \
    --mode hand \
    --step_size 0.005 \
    --image_freq 5 \
    --pause_time 0.3
```

#### Example 3: Custom Output Path

```bash
python3 calibrate_camera.py \
    --camera_id 24285872 \
    --mode third \
    --output /path/to/my_calibration_results.yaml
```

### Tips for Best Results

1. **Lighting**: Ensure good, even lighting on the Charuco board
2. **Distance**: Position camera about 1 foot away from the board
3. **Visibility**: Make sure the entire board is visible in the camera frame
4. **Stability**: Keep the board flat and stable during calibration
5. **Multiple Attempts**: If accuracy check fails, try recalibrating with better positioning

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

### calibrate_camera.py

#### Issue: Camera not found in config

**Solution**: Ensure camera is configured in `config/multi_camera_reader_config.yaml` with:
- `camera_id`: Matching the `--camera_id` argument
- `camera_frame`: Camera optical frame name
- `camera_base_frame`: Camera base frame name
- `camera_base_parent_frame`: Parent frame for calibration

#### Issue: TF lookup failed

**Solution**: 
- Ensure camera driver is running and publishing TF transforms
- Check that `camera_base_frame` and `camera_frame` are correct in config
- Verify TF tree: `ros2 run tf2_tools view_frames`

#### Issue: No Charuco board detected

**Solution**:
- Ensure Charuco board is fully visible in camera frame
- Check lighting conditions (improve if too dark or too bright)
- Verify board is flat and not warped
- Move camera closer to board (about 1 foot away)

#### Issue: Calibration accuracy check failed

**Solution**:
- Recalibrate with better lighting
- Ensure board remains stable during trajectory
- Try adjusting `--step_size` (smaller = more samples)
- Increase `--pause_time` to allow camera to stabilize
- Make sure board is well-positioned at start

#### Issue: Camera intrinsics not available

**Solution**:
- Ensure camera driver is running
- Check that `camera_info_topic` is publishing
- Verify camera is publishing on expected topics
- Wait a few seconds after starting camera driver

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
- `role_ros2/calibration/calibration_utils.py`: Camera calibration utilities
- `role_ros2/calibration/config.py`: Calibration configuration parameters
- `config/multi_camera_reader_config.yaml`: Camera configuration file

---

## 📝 Notes

- All scripts use `RobotEnv`'s internal `MultiThreadedExecutor` for ROS2 message processing
- The main thread only handles control loop timing and user interaction
- All timestamps use ROS nanoseconds for consistency
- Trajectory files are compatible with the DROID dataset format
- Camera calibration uses Charuco board (9x14 grid, DICT_5X5_100)
- Static TF transforms published by `calibrate_camera.py` persist until the process exits

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
