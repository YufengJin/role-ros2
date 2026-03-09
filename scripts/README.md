# Scripts Documentation

This document describes scripts in `scripts/`, including `scripts/postprocess/` and `scripts/misc/`.

## Overview

**scripts/**

- **`collect_trajectory_franka.py`**: Collect single-arm Franka trajectories using VR controller (Oculus Quest)
- **`collect_trajectory_bimanual_franka.py`**: Collect bimanual Franka trajectories using VR controller
- **`replay_trajectory.py`**: Replay saved robot trajectories from HDF5 files
- **`calibrate_camera.py`**: Perform hand-eye calibration for cameras using Charuco board
- **`bringup.py`**: Docker-ROS control center (PyQt5 GUI). Reads `scripts/conf/*.json` to start/stop commands in Docker containers and view logs.

**scripts/postprocess/**

- **`scripts/postprocess/to_tfrecord.py`**: Convert `.h5` trajectories to TFRecord (success-only; uses `task_labels.json` when present).
- **`scripts/postprocess/label_data.py`**: Tk GUI to label success/failure/task_name for `**/trajectory.h5`, maintains `task_labels.json` and `_last_index`.

**scripts/misc/**

- **`scripts/misc/hdf5_reader.py`**: Inspect HDF5 trajectory structure, shapes, and attributes.
- **`scripts/misc/trajectory_visualizer.py`**: Matplotlib GUI to visualize trajectories (slider, camera images, robot/action plots, Gantt).
- **`scripts/misc/visualize_tfds.py`**: Matplotlib GUI 可视化 TFDS/RLDS 数据（to_tfrecord、DROID 等）：三路图像、关节/夹爪/动作时序，Episode/Step 滑条。
- **`scripts/misc/mujoco_to_urdf.py`**: Convert MuJoCo XML to URDF for ROS2.

**scripts/tests/**

- **`scripts/tests/test_tf_load.py`**: Inspect TFDS datasets (to_tfrecord output, DROID, etc.) and provide `get_tfds_dataloader()` for training.

Robot-control scripts (collect, replay, calibrate) are pure Python (not ROS2 nodes) and use `RobotEnv` internally. `bringup.py` is a standalone PyQt5 GUI.

---

## 📝 collect_trajectory_franka.py / collect_trajectory_bimanual_franka.py

### Description

Collect robot trajectories using an Oculus Quest VR controller. The scripts provide two modes:

1. **Teleoperation Mode**: Control the robot without saving data (default when no `--task` is provided)
2. **Recording Mode**: Automatically enabled when `--task` is provided, saves trajectories to HDF5 files

- **`collect_trajectory_franka.py`**: Single-arm Franka (VRPolicy)
- **`collect_trajectory_bimanual_franka.py`**: Bimanual Franka (VRBimanPolicy)

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
# Single-arm Franka
python3 collect_trajectory_franka.py --task pick_and_place --viz

# Bimanual Franka
python3 collect_trajectory_bimanual_franka.py --task pick_and_place --viz
```

#### Full Options

```bash
python3 collect_trajectory_franka.py \
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
python3 collect_trajectory_franka.py --viz
```

#### Example 2: Collect Pick and Place Trajectories

```bash
python3 collect_trajectory_franka.py \
    --task pick_and_place \
    --viz \
    --save-images \
    --control-hz 20.0
```

#### Example 3: Custom Save Location

```bash
python3 collect_trajectory_franka.py \
    --task assembly \
    --viz \
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

## 📦 scripts/postprocess/to_tfrecord.py

### Description

Convert role-ros2 `.h5` trajectories under a data directory to a standard TFDS dataset for training (e.g. with droid_dataset_builder-style loaders).

- **Input**: `--data-dir` (required). `task_labels.json` from `<data-dir>/task_labels.json`.
- **No `task_labels.json`**: Uses `success/**/trajectory.h5` only; prompts to continue.
- **With `task_labels.json`**: Uses HDF5 with `success: true`; `task_name` from `labels[rel]["task_name"]` or HDF5 `attrs["task_name"]` or `"unknown"` → used as **language_instruction** per step.
- **Output**: Standard TFDS dataset in `<output-dir>/<dataset-name>/<version>/`:
  - `dataset_info.json`, `features.json`
  - `{DATASET}-{SPLIT}.tfrecord-{X}-of-{Y}` (e.g. `role_ros2-train.tfrecord-00000-of-00005`)
  - Usable with `tfds.builder_from_directory()` and `tfds.load()`.

Each TFRecord entry = one episode: `steps` (observation: `exterior_image_1_left`, `exterior_image_2_left`, `wrist_image_left`, `cartesian_position`, `gripper_position`, `joint_position`; `action_dict`; `action` (7); `discount`; `reward`; `is_first`; `is_last`; `is_terminal`; `language_instruction`), `episode_metadata` (`file_path`, `recording_folderpath`). Camera mapping: `CAMERA_ID_TO_IMAGE_KEYS` (cam_id → RLDS/DROID observation key) in the script.

### Dependencies

- `tensorflow`, `tensorflow_datasets` (required; e.g. `pip install -e ".[tfrecord]"` or use the Docker image)
- `h5py`, `numpy` (from role_ros2), `tqdm` (optional)

### Usage

```bash
# Required: --data-dir. Output: <data-dir>/tfrecord/role_ros2/1.0.0
python3 scripts/postprocess/to_tfrecord.py --data-dir /path/to/data

# Custom output, dataset name, version, and overwrite
python3 scripts/postprocess/to_tfrecord.py --data-dir /path/to/data --output-dir /path/to/tfrecords --dataset-name my_robot --version 2.0.0 --overwrite

# Tune sharding and workers
python3 scripts/postprocess/to_tfrecord.py --data-dir /path/to/data --shard-size 200 --num-workers 4
```

### Arguments

| Argument | Type | Default | Description |
|----------|------|---------|-------------|
| `--data-dir` | Path | **Required** | Data root (success/, failure/, task_labels.json) |
| `--output-dir` | Path | `<data-dir>/tfrecord` | Parent of `<dataset-name>/<version>/`; versioned dir is `<output-dir>/<dataset-name>/<version>/` |
| `--dataset-name` | str | `role_ros2` | TFDS dataset name for subdir and filenames |
| `--version` | str | `1.0.0` | TFDS version subdir |
| `--shard-size` | int | 200 | Max trajectories per TFRecord shard |
| `--train-fraction` | float | 0.8 | Fraction for train split; rest for test |
| `--overwrite` | flag | False | Overwrite existing versioned output directory |
| `--image-height` | int | 180 | Resize height for video frames |
| `--image-width` | int | 320 | Resize width for video frames |
| `--num-workers` | int | 4 | Parallel workers for shard conversion (use 1 for sequential) |

### Inspecting and loading TFDS output

The versioned dir (e.g. `<output-dir>/role_ros2/1.0.0`) contains `dataset_info.json` and `features.json`. Load with:

```python
import tensorflow_datasets as tfds
builder = tfds.builder_from_directory("/path/to/role_ros2/1.0.0")
ds = builder.as_dataset(split="train")
```

To inspect structure or get a batched dataloader, use **`scripts/tests/test_tf_load.py`** (see below).

---

## 🔍 scripts/tests/test_tf_load.py

### Description

Inspect TFDS datasets (to_tfrecord.py output, DROID, etc.) and provide `get_tfds_dataloader()` for training. Expects a TFDS versioned dir: `dataset_info.json`, `features.json`, and `{DATASET}-{SPLIT}.tfrecord-{X}-of-{Y}`. Uses `tfds.builder_from_directory()` and `builder.as_dataset()`.

### Usage

```bash
# role_ros2 (to_tfrecord output)
python3 scripts/tests/test_tf_load.py --tfrecord-dir /path/to/tfrecord/role_ros2/1.0.0

# DROID or other TFDS
python3 scripts/tests/test_tf_load.py --tfrecord-dir /app/datasets/droid/droid_100/1.0.0 --max-examples 1 --max-steps-per-episode 3
```

### Arguments

| Argument | Type | Default | Description |
|----------|------|---------|-------------|
| `--tfrecord-dir` | Path | `data/tfrecord` | TFDS versioned dir (dataset_info.json, *-train.tfrecord-*) |
| `--split` | str | `train` | `train` or `test` |
| `--max-examples` | int | 2 | Max episodes to describe |
| `--max-steps-per-episode` | int | 10 | Max steps to show per episode |

### get_tfds_dataloader()

```python
from scripts.tests.test_tf_load import get_tfds_dataloader

ds = get_tfds_dataloader("/path/to/role_ros2/1.0.0", split="train", batch_size=4)
for batch in ds.take(10):
    # batch: { steps: Dataset, episode_metadata: {...} }
    ...
```

**Args**: `builder_dir`, `split`, `shuffle_files`, `batch_size`, `**as_dataset_kwargs`. **Returns**: `tf.data.Dataset` of RLDS episodes.

---

## 📊 scripts/misc/visualize_tfds.py

### Description

用 Matplotlib 交互可视化 TFDS/RLDS 数据集（to_tfrecord 输出、DROID 等）。布局与 `trajectory_visualizer` 类似，数据来源为 TFDS：`tfds.builder_from_directory` + `as_dataset`。

- **第一行**：三路相机图 `exterior_image_1_left`、`exterior_image_2_left`、`wrist_image_left` + 信息框（instruction、file）
- **第二行**：`observation.joint_position`(7)、`observation.gripper_position`、`action`(7) 的时序到当前 step
- **滑条**：Episode、Step；可用 `--max-episodes` 限制预加载的 episode 数

目录需为含 `dataset_info.json` 的 TFDS 版本目录（如 `.../role_ros2/1.0.0` 或 `.../droid_100/1.0.0`）。

### Usage

```bash
# role_ros2（to_tfrecord 输出）
python3 scripts/misc/visualize_tfds.py --tfrecord-dir data/tfrecord/role_ros2/1.0.0

# DROID，限制缓存 5 个 episode
python3 scripts/misc/visualize_tfds.py --tfrecord-dir /app/datasets/droid/droid_100/1.0.0 --max-episodes 5

# 无头保存为 PNG（无 GUI 时先 export MPLBACKEND=Agg）
python3 scripts/misc/visualize_tfds.py --tfrecord-dir data/tfrecord/role_ros2/1.0.0 --save /tmp/out.png
```

### Arguments

| Argument | Type | Default | Description |
|----------|------|---------|-------------|
| `--tfrecord-dir` | Path | `data/tfrecord` | TFDS 版本目录（含 dataset_info.json、*-train.tfrecord-*） |
| `--split` | str | `train` | `train` 或 `test` |
| `--max-episodes` | int | 10 | 预加载并缓存的 episode 数量上限 |
| `--save` | Path | - | 将当前 figure 保存到文件后退出（不弹 GUI）；无头环境可配合 `MPLBACKEND=Agg` |

### Examples

参见 **`scripts/misc/visualize_tfds_examples.sh`**：可复制其中命令，或在 role-ros2 根目录运行 `./scripts/misc/visualize_tfds_examples.sh 1` 等。

#### Example 1: role_ros2（to_tfrecord 输出）

```bash
python3 scripts/misc/visualize_tfds.py --tfrecord-dir data/tfrecord/role_ros2/1.0.0
```

#### Example 2: DROID，限制缓存的 episode

```bash
python3 scripts/misc/visualize_tfds.py --tfrecord-dir /app/datasets/droid/droid_100/1.0.0 --max-episodes 5
```

#### Example 3: 无头保存（CI / 无显示器）

```bash
export MPLBACKEND=Agg
python3 scripts/misc/visualize_tfds.py --tfrecord-dir data/tfrecord/role_ros2/1.0.0 --save /tmp/visualize_tfds_out.png
```

#### Example 4: 只缓存 3 个 episode、train 分片

```bash
python3 scripts/misc/visualize_tfds.py --tfrecord-dir data/tfrecord/role_ros2/1.0.0 --max-episodes 3 --split train
```

---

## 🏷️ scripts/postprocess/label_data.py

### Description

Tk-based GUI to label trajectories under a data folder: set **success** / **failure** and **task_name** for each `**/trajectory.h5`. Reads and writes `<data_folder>/task_labels.json` and keeps `_last_index` so you can resume from the last labeled file.

### Usage

```bash
python3 scripts/postprocess/label_data.py <data_folder>
```

### Arguments

| Argument | Type | Description |
|----------|------|-------------|
| `data_folder` | str | **Required** (positional). Root folder containing `**/trajectory.h5`; `task_labels.json` will be at `<data_folder>/task_labels.json`. |

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

## 🖥️ bringup.py

### Description

Docker-ROS control center: a PyQt5 GUI that reads `scripts/conf/*.json` and provides start/stop buttons and a log view for each entry. Each entry specifies a Docker `container`, a `command` to run inside it (e.g. `ros2 launch ...` or `python3 scripts/collect_trajectory_franka.py`), and a `kill_keyword` to stop the process. Useful to launch robot, cameras, Rviz2, and collection scripts from one window.

### Prerequisites

- Docker with the target containers running (e.g. `ros2_polymetis_container`, `ros2_cu118_container`)
- PyQt5, `psutil`
- `scripts/conf/franka.json` or `scripts/conf/biman_franka.json`

### Usage

```bash
python3 scripts/bringup.py
# Or specify config: python3 scripts/bringup.py --config conf/biman_franka.json
```

Default config: `scripts/conf/franka.json`. Use `--config` to load a different file.

---

## 🛠️ scripts/misc/ Tools

### scripts/misc/hdf5_reader.py

Inspect HDF5 trajectory files: hierarchy, dataset shapes/dtypes, and attributes.

```bash
python3 scripts/misc/hdf5_reader.py /path/to/trajectory.h5 [--show-data 0 | --show-all-data] [--max-depth N] [--show-attrs] [--timestamps-only] [--summary-only]
```

### scripts/misc/trajectory_visualizer.py

Matplotlib GUI: slider, camera images, robot/action time series, Gantt chart. Depends on `role_ros2.trajectory_utils.trajectory_reader`.

```bash
python3 scripts/misc/trajectory_visualizer.py /path/to/trajectory.h5 [--debug]
```

### scripts/misc/visualize_tfds.py

Matplotlib GUI 可视化 TFDS/RLDS：三路图像、joint/gripper/action 时序，Episode/Step 滑条。支持 to_tfrecord 输出与 DROID。示例：`scripts/misc/visualize_tfds_examples.sh`。

```bash
python3 scripts/misc/visualize_tfds.py --tfrecord-dir data/tfrecord/role_ros2/1.0.0 [--max-episodes 10] [--save out.png]
```

### scripts/misc/mujoco_to_urdf.py

Convert MuJoCo XML model to URDF for use in ROS2. Can optionally generate `joint_names.yaml`.

---

## 🔧 Common Issues

### collect_trajectory_franka.py / collect_trajectory_bimanual_franka.py

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

Use `scripts/misc/hdf5_reader.py` to inspect trajectory structure, shapes, and attributes. For interactive visualization, see **scripts/misc/ Tools** (`scripts/misc/trajectory_visualizer.py`).

```bash
python3 scripts/misc/hdf5_reader.py /path/to/trajectory.h5
```

### Enable Debug Logging

Set ROS2 log level:
```bash
export RCUTILS_LOGGING_SEVERITY=DEBUG
```

---

## 📧 Support

For issues or questions, please refer to the main project documentation or contact the Role-ROS2 team.
