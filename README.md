# role-ros2: Unified Robot Learning Platform

`role-ros2` (Robot Learning ROS2) is a unified ROS2 platform for robot learning tasks, providing complete ROS2 interfaces and tools.

## Overview

`role-ros2` is an independent ROS2 package specifically designed for robot learning tasks, providing:

- 🤖 **Robot Control Interface**: Franka robot arm control based on `franka_ros2`
- 🎮 **Gym-compatible Environment**: OpenAI Gym-compatible robot environment interface
- 📷 **Camera Data Synchronization**: Multi-camera data subscription with ROS2 time synchronization
- 🎯 **Oculus Quest Support**: VR controller data publishing and subscription
- 🔧 **Robot Learning Tools**: Inverse kinematics solver, calibration tools, etc.

## Version Information

- **ROS2 Distribution**: Humble Hawksbill
- **role_ros2 Version**: 1.0.0
- **Dependencies**:
  - `franka_ros2`: b79ce40
  - `franka_description`: 0.5.1
  - `zed_wrapper`: humble-v5.1.0

## Features

### 1. Robot Control (`role_ros2.robot`)

Provides `FrankaRobot` class that encapsulates Franka robot arm control interface:

- Joint space control (position, velocity, torque)
- Cartesian space control (position, velocity)
- Gripper control
- Robot state subscription (joint states, end-effector pose, robot state)

```python
from role_ros2.robot.franka.robot import FrankaRobot
import rclpy

rclpy.init()
robot = FrankaRobot()

# Move to joint position
robot.update_joints([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785], velocity=False)

# Move to Cartesian position
robot.update_pose([0.5, 0.0, 0.5, 0.0, 0.0, 0.0], velocity=False)

# Control gripper
robot.update_gripper(0.04)  # Open to 4cm
```

### 2. Robot Environment (`role_ros2.robot_env`)

Provides Gym-compatible robot environment supporting reinforcement learning and imitation learning:

- Observation space: Robot state + camera images
- Action space: Joint position/velocity or Cartesian position
- Time synchronization: Multi-sensor data synchronization using ROS2 `ApproximateTimeSynchronizer`

```python
from role_ros2.robot_env import RobotEnv
import rclpy

rclpy.init()
env = RobotEnv()

# Get observation
obs = env.get_observation()
# obs contains:
# - 'robot_state': Robot state (joint positions, velocities, end-effector pose, etc.)
# - 'images': Camera image dictionary

# Execute action
action = env.action_space.sample()
obs, reward, done, info = env.step(action)
```

### 3. Camera Data Subscription

Supports time-synchronized multi-camera data subscription:

- RGB images
- Depth images
- Camera intrinsics (CameraInfo)
- Automatic time synchronization (using `ApproximateTimeSynchronizer`)

### 4. Oculus Quest Controller

Publishes Oculus Quest controller pose and button states:

- Left/right hand controller poses (`geometry_msgs/PoseStamped`)
- Button and joystick states (custom message `role_ros2/OculusButtons`)
- TF transform publishing (optional)
- RViz visualization markers (optional)

### 5. Utility Modules

- **Inverse Kinematics Solver** (`role_ros2.robot_ik`): Inverse kinematics computation based on `RobotIKSolver`
- **Calibration Tools** (`role_ros2.calibration`): Camera and robot calibration tools
- **Transformation Tools** (`role_ros2.misc.transformations`): Pose transformation and coordinate frame conversion

## Installation

### Method 1: Docker Installation (Recommended)

Docker installation provides a complete isolated environment with all dependencies, making it the simplest and most reliable installation method.

#### Prerequisites

- **Docker** and **Docker Compose** installed
- **NVIDIA GPU** (optional, for ZED cameras): Requires `nvidia-container-toolkit`
- **X11 Forwarding** (optional, for GUI applications like rviz2)

#### Choose Docker Environment

Select the appropriate Docker environment based on your needs:

| Docker Environment | ROS2 Version | Purpose | Description |
|-------------------|--------------|---------|-------------|
| `docker/ros2_cu118/` | Humble | ZED cameras, CUDA acceleration | For scenarios requiring GPU and ZED cameras |
| `docker/ros2_franka_libfranka_0.18.x/` | Foxy | Franka robot control | For robot system 5.9.0+ |
| `docker/ros2_franka_libfranka_0.14.x/` | Foxy | Franka robot control | For robot system 5.7.1 |

#### Quick Start (ROS2 Humble + CUDA)

```bash
# 1. Navigate to Docker config directory
cd docker/ros2_cu118

# 2. Build image
docker compose build

# 3. Start container
docker compose up -d

# 4. Enter container
docker exec -it ros2_cu118_container bash

# 5. Build workspace inside container
cd /app/ros2_ws
colcon build --packages-select role_ros2 --symlink-install
source install/setup.bash
```

#### Quick Start (ROS2 Foxy + Polymetis)

```bash
# 1. Navigate to Docker config directory (select based on robot system version)
cd docker/ros2_franka_libfranka_0.18.x  # or 0.14.x

# 2. Build image
docker compose build

# 3. Start container
docker compose up -d

# 4. Enter container
docker exec -it ros2_polymetis_container bash

# 5. Build workspace inside container
cd /app/ros2_ws
colcon build --packages-select role_ros2 --symlink-install
source install/setup.bash
```

#### Docker Environment Configuration

**GPU Support (Required for ZED cameras)**:
```bash
cd docker/ros2_cu118
sudo ./install_nvidia_toolkit.sh
```

**X11 Forwarding (GUI applications)**:
```bash
cd docker/ros2_cu118
source setup_x11.sh
```

**Development Mode**:
- Docker Compose is configured with source code mounting, allowing code editing outside the container and compilation inside
- Changes are immediately reflected in the container, supporting hot reload

For detailed documentation, refer to:
- `docker/ros2_cu118/README.md` - ROS2 Humble + CUDA environment
- `docker/ros2_franka_libfranka_0.18.x/README.md` - ROS2 Foxy + Polymetis environment

### Method 2: Local Installation

#### Prerequisites

1. **ROS2 Humble**: Installed and configured
2. **Python 3.10**: ROS2 Humble requires Python 3.10
3. **Dependencies**:
   - `franka_ros2`
   - `franka_description`
   - `zed_wrapper` (if using cameras)
4. **Python Dependencies**:
   - `rclpy`
   - `numpy`
   - `gym`
   - `cv_bridge`
   - `message_filters`

#### Build

```bash
cd ~/repos/role-ros2
source /opt/ros/humble/setup.bash
colcon build --packages-select role_ros2 --symlink-install
source install/setup.bash
```

## Usage

### Start Environment

**Recommended: Use workspace `start_env.sh` script**

```bash
# From project root directory
source ros2_ws/start_env.sh
```

### Launch System via CLI

You can launch system components directly from the command line in Docker containers. All commands from `scripts/config.json` can be executed manually:

#### Launch Robot

**Real Robot** (in `ros2_polymetis_container`):
```bash
docker exec -it ros2_polymetis_container bash -c "source /opt/ros/foxy/setup.bash && source /app/ros2_ws/install/setup.bash && ros2 launch role_ros2 franka_robot.launch.py use_mock:=false"
```

**Mock Robot** (in `ros2_polymetis_container`):
```bash
docker exec -it ros2_polymetis_container bash -c "source /opt/ros/foxy/setup.bash && source /app/ros2_ws/install/setup.bash && ros2 launch role_ros2 franka_robot.launch.py use_mock:=true"
```

#### Launch Cameras

**ZED Hand Camera (Low Res)** (in `ros2_cu118_container`):
```bash
docker exec -it ros2_cu118_container bash -c "source /opt/ros/humble/setup.bash && source /app/ros2_ws/install/setup.bash && ros2 launch role_ros2 zed_camera.launch.py config_file:=hand_zed_low_res.yaml"
```

**ZED Static Camera (Low Res)** (in `ros2_cu118_container`):
```bash
docker exec -it ros2_cu118_container bash -c "source /opt/ros/humble/setup.bash && source /app/ros2_ws/install/setup.bash && ros2 launch role_ros2 zed_camera.launch.py config_file:=static_zed_low_res.yaml"
```

**ZED Hand Camera (High Res)** (in `ros2_cu118_container`):
```bash
docker exec -it ros2_cu118_container bash -c "source /opt/ros/humble/setup.bash && source /app/ros2_ws/install/setup.bash && ros2 launch role_ros2 zed_camera.launch.py config_file:=hand_zed_high_res.yaml"
```

**ZED Static Camera (High Res)** (in `ros2_cu118_container`):
```bash
docker exec -it ros2_cu118_container bash -c "source /opt/ros/humble/setup.bash && source /app/ros2_ws/install/setup.bash && ros2 launch role_ros2 zed_camera.launch.py config_file:=static_zed_high_res.yaml"
```

#### Launch Visualization

**Rviz2** (in `ros2_cu118_container`):
```bash
docker exec -it ros2_cu118_container bash -c "cd /app/ros2_ws/src/role-ros2 && source /opt/ros/humble/setup.bash && source /app/ros2_ws/install/setup.bash && ros2 run rviz2 rviz2 -d config/rviz/collect_traj.rviz"
```

#### Launch Data Collection

**Collect Trajectory** (in `ros2_cu118_container`):
```bash
docker exec -it ros2_cu118_container bash -c "cd /app/ros2_ws/src/role-ros2 && source /opt/ros/humble/setup.bash && source /app/ros2_ws/install/setup.bash && python3 scripts/collect_trajectory.py --viz"
```

**Note**: For easier management of multiple components, consider using the GUI tool `bringup.py` (see [Utility Scripts](#utility-scripts) section).

### Start Robot

Start Franka robot arm and gripper controller:

```bash
ros2 launch role_ros2 franka_robot.launch.py
```

### Start Cameras

Start two ZED cameras (hand_camera and static_camera):

```bash
ros2 launch role_ros2 zed_camera.launch.py config_file:=hand_zed_low_res.yaml
ros2 launch role_ros2 zed_camera.launch.py config_file:=static_zed_low_res.yaml
```

### Start Oculus Quest Controller Node

```bash
# USB connection (default)
ros2 launch role_ros2 oculus_controller.launch.py

# Network connection
ros2 launch role_ros2 oculus_controller.launch.py oculus_ip_address:=192.168.1.100
```

### Using Python API

#### Basic Robot Control

```python
import rclpy
from role_ros2.robot.franka.robot import FrankaRobot

rclpy.init()
robot = FrankaRobot()

# Get robot state
state, timestamps = robot.get_robot_state()
print(f"Joint positions: {state['joint_positions']}")
print(f"End-effector pose: {state['cartesian_position']}")

# Move to target position
target_joints = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
robot.update_joints(target_joints, velocity=False, blocking=True)

rclpy.shutdown()
```

#### Using Robot Environment

```python
import rclpy
from role_ros2.robot_env import RobotEnv

rclpy.init()
env = RobotEnv()

# Reset environment
obs = env.reset()

# Execute action loop
for _ in range(100):
    # Get observation
    obs = env.get_observation()
    
    # Select action (example: random action)
    action = env.action_space.sample()
    
    # Execute action
    obs, reward, done, info = env.step(action)
    
    if done:
        obs = env.reset()

rclpy.shutdown()
```

## Utility Scripts

### Scripts Directory

The `scripts/` directory contains Python script tools for robot learning and data collection:

#### 1. `bringup.py` - Docker-ROS Control Center

Graphical interface tool for managing ROS2 node startup in Docker containers.

**Features**:
- Reads configuration from `config.json`, dynamically generates control interface
- One-click start/stop for multiple ROS2 nodes
- Real-time log display
- Supports multiple Docker containers (ros2_cu118_container, ros2_polymetis_container)

**Usage**:
```bash
python3 scripts/bringup.py
```

**Configuration**: Edit `scripts/config.json` to add or modify task configurations.

#### 2. `collect_trajectory.py` - Trajectory Collection Tool

Collect robot trajectory data using VR controller (Oculus Quest).

**Features**:
- Real-time robot control via VR controller
- Automatic trajectory saving to HDF5 files
- Support for multiple action spaces (cartesian_velocity, joint_velocity, etc.)
- Long-press button to mark SUCCESS/FAILURE

**Usage**:
```bash
# Basic usage (teleoperation only, no saving)
python3 scripts/collect_trajectory.py

# Collect trajectory data
python3 scripts/collect_trajectory.py --task pick_and_place --save-images
```

Detailed documentation: `scripts/README.md`

#### 3. `replay_trajectory.py` - Trajectory Replay Tool

Replay saved robot trajectories.

**Features**:
- Load trajectories from HDF5 files
- Automatic initial position alignment
- Adjustable replay speed
- Support for multiple action spaces

**Usage**:
```bash
python3 scripts/replay_trajectory.py --filepath /path/to/trajectory.h5
```

#### 4. `calibrate_camera.py` - Camera Calibration Tool

Perform hand-eye calibration using Charuco board.

**Features**:
- Automatic calibration trajectory execution
- Real-time Charuco board detection visualization
- Calibration accuracy evaluation
- Automatic static TF transform publishing
- Save calibration results to fixed file

**Usage**:
```bash
# Calibrate camera (calibration_mode automatically read from config file)
python3 scripts/calibrate_camera.py --camera_id 24285872
```

Detailed documentation: `scripts/README.md`

### Misc Directory

The `misc/` directory contains auxiliary utility scripts:

#### 1. `hdf5_reader.py` - HDF5 File Viewer

View and analyze structure and data of HDF5 trajectory files.

**Features**:
- Display basic file information (size, creation time, etc.)
- Display data structure hierarchy
- Display array shapes and data types
- Optional data content display

**Usage**:
```bash
python3 scripts/misc/hdf5_reader.py /path/to/trajectory.h5
python3 scripts/misc/hdf5_reader.py /path/to/trajectory.h5 --show-data 0  # Show data at index 0
```

#### 2. `control_analysis.py` - Control System Performance Analysis

Analyze robot control system performance metrics.

**Features**:
- Calculate rise time (Rise Time)
- Calculate percentage overshoot (Percentage Overshoot)
- Calculate steady-state error (Steady-State Error)
- Control effort and smoothness metrics
- Generate professional visualization charts

**Usage**:
```bash
python3 scripts/misc/control_analysis.py /path/to/trajectory.h5
python3 scripts/misc/control_analysis.py /path/to/trajectory.h5 --dim 0  # Analyze specific dimension
```

#### 3. `trajectory_visualizer.py` - Trajectory Visualization Tool

Interactive GUI tool for visualizing robot learning trajectory data.

**Features**:
- Interactive time slider navigation
- 4x4 subplot layout:
  - Camera RGB/depth images + timestamp panel
  - Robot state time series + Gantt chart
  - Action time series + latency statistics box plot
- Support for multi-camera data visualization

**Usage**:
```bash
python3 scripts/misc/trajectory_visualizer.py /path/to/trajectory.h5
```

#### 4. `mujoco_to_urdf.py` - MuJoCo to URDF Converter

Convert MuJoCo XML models to URDF format for ROS2 visualization.

**Features**:
- Convert MuJoCo XML to URDF
- Handle joints, links, inertial parameters
- Generate ROS2-compatible URDF files

**Usage**:
```bash
python3 scripts/misc/mujoco_to_urdf.py input.xml output.urdf arm_id
```

## Package Structure

```
role-ros2/
├── scripts/                    # Python script tools
│   ├── bringup.py             # Docker-ROS control center (GUI)
│   ├── collect_trajectory.py  # Trajectory collection tool
│   ├── replay_trajectory.py   # Trajectory replay tool
│   ├── calibrate_camera.py    # Camera calibration tool
│   ├── config.json            # bringup.py configuration file
│   ├── misc/                  # Utility scripts
│   │   ├── hdf5_reader.py         # HDF5 trajectory file viewer
│   │   ├── control_analysis.py    # Control system performance analysis
│   │   ├── trajectory_visualizer.py # Trajectory visualization tool (GUI)
│   │   └── mujoco_to_urdf.py      # MuJoCo XML to URDF converter
│   └── postprocess/           # Data postprocessing
│       ├── to_tfrecord.py     # Convert .h5 to TFRecord
│       └── label_data.py      # GUI to label success/failure/task_name
├── launch/                     # Launch files
│   ├── franka_robot.launch.py  # Start robot controller
│   ├── zed_camera.launch.py    # Start ZED camera
│   ├── realsense_camera.launch.py # Start RealSense camera
│   └── oculus_controller.launch.py # Start Oculus node
├── config/                     # Configuration files
│   ├── franka_robot_config.yaml # Robot configuration
│   ├── multi_camera_reader_config.yaml # Multi-camera configuration
│   ├── calibration_results.yaml # Camera calibration results
│   └── cameras/               # Camera configuration files
├── docker/                     # Docker environment configuration
│   ├── ros2_cu118/            # ROS2 Humble + CUDA environment
│   ├── ros2_franka_libfranka_0.18.x/ # ROS2 Foxy + Polymetis (0.18.x)
│   └── ros2_franka_libfranka_0.14.x/ # ROS2 Foxy + Polymetis (0.14.x)
├── nodes/                      # ROS2 nodes
│   ├── franka_robot_interface_node.py
│   ├── franka_gripper_interface_node.py
│   ├── robot_state_aggregator_node.py
│   └── camera_calibration_tf_publisher_node.py
├── msg/                        # Custom messages
├── srv/                        # Custom services
└── role_ros2/                  # Python modules
    ├── robot/                  # Robot control
    │   ├── base_robot.py      # Robot base class
    │   └── franka/robot.py     # Franka robot implementation
    ├── robot_env.py            # Gym environment
    ├── robot_ik/               # Inverse kinematics
    ├── calibration/            # Calibration tools
    ├── camera/                 # Camera tools
    └── misc/                   # Utility functions
```

## Topics

### Robot State Topics

- `/joint_states` (sensor_msgs/JointState) - Joint states
- `/robot_state` (role_ros2/RobotState) - Robot state (aggregated)
- `/fr3_arm/arm_state` (role_ros2/ArmState) - Arm state
- `/fr3_gripper/gripper_state` (role_ros2/GripperState) - Gripper state

### Camera Topics

- `/hand_camera/zed_node/rgb/image_rect_color` (sensor_msgs/Image) - Hand camera RGB
- `/hand_camera/zed_node/depth/depth_registered` (sensor_msgs/Image) - Hand camera depth
- `/static_camera/zed_node/rgb/image_rect_color` (sensor_msgs/Image) - Static camera RGB
- `/static_camera/zed_node/depth/depth_registered` (sensor_msgs/Image) - Static camera depth

### Oculus Quest Controller Topics

- `/oculus/right_controller/pose` (geometry_msgs/PoseStamped) - Right hand controller pose
- `/oculus/left_controller/pose` (geometry_msgs/PoseStamped) - Left hand controller pose
- `/oculus/buttons` (role_ros2/OculusButtons) - Button and joystick states
- `/oculus/controllers/markers` (visualization_msgs/MarkerArray) - RViz visualization markers (optional)

## Configuration

### Robot Configuration

Robot configuration is controlled via launch file parameters:

- `arm_id` (default: "fr3") - Robot arm ID
- `arm_namespace` (default: "fr3_arm") - Arm namespace
- `gripper_namespace` (default: "fr3_gripper") - Gripper namespace
- `robot_ip` - Robot IP address (required)
- `use_mock` (default: false) - Use mock interfaces (no real robot)

### Camera Configuration

Camera parameters are set via YAML configuration files:

- `config/multi_camera_reader_config.yaml` - Multi-camera configuration
- `config/cameras/` - Individual camera configuration files

### Oculus Reader Node Parameters

- `publish_rate` (default: 50.0) - Publishing frequency (Hz)
- `publish_tf` (default: true) - Whether to publish TF transforms
- `publish_markers` (default: false) - Whether to publish RViz visualization markers
- `oculus_ip_address` (default: '') - Oculus Quest IP address (empty string means USB connection)
- `oculus_port` (default: 5555) - ADB network connection port
- `frame_id` (default: 'oculus_base') - Coordinate frame ID

## Development Guide

### Adding New Robot Interface

1. Add new robot class in `role_ros2/robot/`
2. Implement standard robot interface methods
3. Update `__init__.py` to export new class

### Extending Robot Environment

1. Inherit from `RobotEnv` class
2. Override `get_observation()` and `step()` methods
3. Define custom observation and action spaces

### Adding New Sensors

1. Create sensor subscription node
2. Integrate sensor data in `robot_env.py`
3. Use `ApproximateTimeSynchronizer` to synchronize multi-sensor data

## Troubleshooting

### Robot Connection Issues

**Issue: Cannot connect to robot**
- Check if robot IP address is correct
- Ensure robot and computer are on the same network
- Check firewall settings

### Camera Synchronization Issues

**Issue: Camera data not synchronized**
- Adjust `slop` parameter of `ApproximateTimeSynchronizer`
- Check if camera timestamps are synchronized
- Ensure cameras publish at consistent rates

### ZED Camera Startup Issues

**Issue: `CAMERA NOT DETECTED` or `MOTION SENSORS REQUIRED`**
- **Check USB connection**: Ensure camera is connected to USB 3.0 port (`lsusb -t` shows `5000M`)
- **Check permissions**: Ensure udev rules are configured and active
  ```bash
  # Check permissions
  ls -l /dev/bus/usb/$(lsusb -d 2b03: | awk '{print $2}')/$(lsusb -d 2b03: | awk '{print $4}' | tr -d ':')
  # Should show rw-rw-rw- (0666)
  ```
- **Reload udev rules**: `sudo udevadm control --reload-rules && sudo udevadm trigger`
- **Physical reconnection**: Unplug camera USB cable, wait 5 seconds, then reconnect

**Issue: `LOW USB BANDWIDTH` or `Unable to capture images`**
- **Reduce resolution or frame rate**: Set `grab_resolution` to `VGA` or reduce `grab_frame_rate` to 10Hz in config file
- **Check USB bus allocation**: Ensure two cameras are connected to different USB buses (`lsusb -t` shows different Bus)
- **Reduce depth mode**: Change `depth_mode` from `ULTRA` to `PERFORMANCE`
- **Increase startup delay**: Add camera startup interval time in launch file

**Issue: `libhid error: -6` or `can't claim interface`**
- **Check USB HID permissions**: Ensure udev rules include HID device permissions
- **Check device occupancy**: Ensure no other process is using the camera device
- **Restart container**: `docker restart <container_name>`

**Issue: Cannot access camera in Docker container**
- **Check Docker configuration**: Ensure `docker-compose.yaml` includes `privileged: true` and `devices: - "/dev:/dev"`
- **Check device mapping**: Execute `lsusb -d 2b03:` inside container to confirm camera devices are visible
- **Check NVIDIA runtime**: Ensure using `nvidia` runtime (GPU support)

### Python Import Errors

**Issue: `ModuleNotFoundError: No module named 'role_ros2'`**
- Ensure workspace is built and sourced: `source ros2_ws/install/setup.bash`
- Check Python path: `python -c "import sys; print(sys.path)"`
- Verify package is correctly installed: `ros2 pkg list | grep role_ros2`

## Contributing

Contributions and suggestions are welcome! Please ensure:

1. Code follows ROS2 and Python best practices
2. Appropriate documentation and comments are added
3. Related README and documentation are updated

## License

Apache-2.0

## Related Documentation

- [Code Deep Understanding](docs/CODE_DEEP_UNDERSTANDING.md) - Architecture, core modules, data flow, and design decisions
- [ROS2 Workspace README](../../README.md)
- [Franka ROS2 Documentation](https://github.com/souljaboy764/franka_ros2)
- [ZED ROS2 Wrapper Documentation](https://github.com/stereolabs/zed-ros2-wrapper)
