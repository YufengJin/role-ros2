# role-ros2: Robot Learning 统一平台

`role-ros2`（Robot Learning ROS2）是机器人学习（Robot Learning）的 ROS2 统一平台，为机器人学习任务提供完整的 ROS2 接口和工具。

## 概述

`role-ros2` 是一个独立的 ROS2 包，专门为机器人学习任务设计，提供：

- 🤖 **机器人控制接口**：基于 `franka_ros2` 的 Franka 机械臂控制
- 🎮 **Gym-compatible 环境**：兼容 OpenAI Gym 的机器人环境接口
- 📷 **相机数据同步**：使用 ROS2 时间同步的多相机数据订阅
- 🎯 **Oculus Quest 支持**：VR 控制器数据发布和订阅
- 🔧 **机器人学习工具**：逆运动学求解器、标定工具等

## 版本信息

- **ROS2 发行版**: Humble Hawksbill
- **role_ros2 版本**: 1.0.0
- **依赖包**:
  - `franka_ros2`: b79ce40
  - `franka_description`: 0.5.1
  - `zed_wrapper`: humble-v5.1.0

## 功能特性

### 1. 机器人控制 (`role_ros2.robot`)

提供 `FrankaRobot` 类，封装 Franka 机械臂的控制接口：

- 关节空间控制（位置、速度、力矩）
- 笛卡尔空间控制（位置、速度）
- 夹爪控制
- 机器人状态订阅（关节状态、末端位姿、机器人状态）

```python
from role_ros2.robot import FrankaRobot
import rclpy

rclpy.init()
robot = FrankaRobot(arm_id="fr3", controller_name="fr3_arm_controller")

# 移动到关节位置
robot.update_joint_positions([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785])

# 移动到笛卡尔位置
robot.update_ee_pose([0.5, 0.0, 0.5], [0.0, 0.0, 0.0, 1.0])

# 控制夹爪
robot.update_gripper(0.04)  # 打开到 4cm
```

### 2. 机器人环境 (`role_ros2.robot_env`)

提供 Gym-compatible 的机器人环境，支持强化学习和模仿学习：

- 观察空间：机器人状态 + 相机图像
- 动作空间：关节位置/速度或笛卡尔位置
- 时间同步：使用 ROS2 `ApproximateTimeSynchronizer` 同步多传感器数据

```python
from role_ros2.robot_env import RobotEnv
import rclpy

rclpy.init()
env = RobotEnv()

# 获取观察
obs = env.get_observation()
# obs 包含：
# - 'robot_state': 机器人状态（关节位置、速度、末端位姿等）
# - 'images': 相机图像字典

# 执行动作
action = env.action_space.sample()
obs, reward, done, info = env.step(action)
```

### 3. 相机数据订阅

支持多相机数据的时间同步订阅：

- RGB 图像
- 深度图像
- 相机内参（CameraInfo）
- 自动时间同步（使用 `ApproximateTimeSynchronizer`）

### 4. Oculus Quest 控制器

发布 Oculus Quest 控制器的姿态和按钮状态：

- 左右手控制器姿态（`geometry_msgs/PoseStamped`）
- 按钮和摇杆状态（自定义消息 `role_ros2/OculusButtons`）
- TF 变换发布（可选）
- RViz 可视化标记（可选）

### 5. 工具模块

- **逆运动学求解器** (`role_ros2.robot_ik`): 基于 `RobotIKSolver` 的逆运动学计算
- **标定工具** (`role_ros2.calibration`): 相机和机器人标定工具
- **变换工具** (`role_ros2.misc.transformations`): 位姿变换和坐标系转换

## 安装

### 前置要求

1. **ROS2 Humble**：已安装并配置
2. **Python 3.10**：ROS2 Humble 需要 Python 3.10（详见下面的"Python 环境设置"部分）
3. **依赖包**：
   - `franka_ros2`
   - `franka_description`
   - `zed_wrapper`（如果使用相机）
4. **Python 依赖**：
   - `rclpy`
   - `numpy`
   - `gym`
   - `cv_bridge`
   - `message_filters`

### Python 环境设置

#### 为什么需要 Python 3.10？

ROS2 Humble 需要 Python 3.10。如果使用 Python 3.8 或 3.9，会出现以下错误：

```
ModuleNotFoundError: No module named 'rclpy._rclpy_pybind11'
The C extension '/opt/ros/humble/lib/python3.10/site-packages/_rclpy_pybind11.cpython-38-x86_64-linux-gnu.so' isn't present on the system.
```

这是因为 ROS2 系统安装的 C 扩展模块是为 Python 3.10 编译的，与 Python 3.8 不兼容。

#### 迁移到 Python 3.10

**1. 备份当前环境（可选但推荐）**

```bash
# 导出当前环境的包列表
conda activate robot
conda env export -n robot > robot_env_py38_backup.yml
pip freeze > robot_pip_py38_backup.txt
```

**2. 创建新的 Python 3.10 环境**

```bash
# 创建新环境
conda create -n robot python=3.10

# 激活新环境
conda activate robot
```

**3. 重新安装项目依赖**

```bash
# 确保在项目根目录
cd ~/repos/droid

# 安装 Oculus Reader
pip install -e ./droid/oculus_reader

# 安装主项目
pip install -e .

# 安装其他依赖（如果需要）
pip install dm-robotics-moma==0.5.0 --no-deps
pip install dm-robotics-transformations==0.5.0 --no-deps
pip install dm-robotics-agentflow==0.5.0 --no-deps
pip install dm-robotics-geometry==0.5.0 --no-deps
pip install dm-robotics-manipulation==0.5.0 --no-deps
pip install dm-robotics-controllers==0.5.0 --no-deps
```

**4. 验证安装**

```bash
# 激活 ROS2 环境
source ros2_ws/start_env.sh

# 验证 Python 版本
python --version  # 应该显示 Python 3.10.x

# 验证 rclpy
python -c "import rclpy; print('rclpy version:', rclpy.__version__)"

# 验证其他关键包
python -c "import pyzed; print('pyzed available')"

# 测试 ROS2 节点
ros2 launch role_ros2 zed_cameras.launch.py
ros2 launch role_ros2 oculus_controller.launch.py
```

**常见问题**

- **Q: 如何同时保留两个环境？**
  - 可以重命名旧环境：`conda create -n robot_py38 --clone robot`，然后创建新的 Python 3.10 环境

- **Q: 某些包在 Python 3.10 下不兼容怎么办？**
  - 大多数现代 Python 包都支持 Python 3.10。如果遇到兼容性问题，检查包的最新版本是否支持 Python 3.10

- **Q: 如何回滚到旧环境？**
  - 如果备份了环境文件：`conda env create -n robot_py38 --file robot_env_py38_backup.yml`

### 构建

```bash
cd ~/repos/droid/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select role_ros2 --symlink-install
source install/setup.bash
```

## 使用方法

### 启动环境

**推荐方式：使用工作空间的 `start_env.sh` 脚本**

```bash
# 从项目根目录
source ros2_ws/start_env.sh
```

### 启动机器人

启动 Franka 机械臂和夹爪控制器：

```bash
ros2 launch role_ros2 franka_robot.launch.py
```

### 启动相机

启动两个 ZED 相机（hand_camera 和 static_camera）：

```bash
ros2 launch role_ros2 zed_cameras.launch.py
```

### 启动 Oculus Quest 控制器节点

```bash
# USB 连接（默认）
ros2 launch role_ros2 oculus_controller.launch.py

# 网络连接
ros2 launch role_ros2 oculus_controller.launch.py oculus_ip_address:=192.168.1.100
```

### 使用 Python API

#### 基本机器人控制

```python
import rclpy
from role_ros2.robot import FrankaRobot

rclpy.init()
robot = FrankaRobot(arm_id="fr3", controller_name="fr3_arm_controller")

# 获取机器人状态
state = robot.get_robot_state()
print(f"关节位置: {state['joint_positions']}")
print(f"末端位姿: {state['ee_pose']}")

# 移动到目标位置
target_joints = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
robot.update_joint_positions(target_joints)

rclpy.shutdown()
```

#### 使用机器人环境

```python
import rclpy
from role_ros2.robot_env import RobotEnv

rclpy.init()
env = RobotEnv()

# 重置环境
obs = env.reset()

# 执行动作循环
for _ in range(100):
    # 获取观察
    obs = env.get_observation()
    
    # 选择动作（示例：随机动作）
    action = env.action_space.sample()
    
    # 执行动作
    obs, reward, done, info = env.step(action)
    
    if done:
        obs = env.reset()

rclpy.shutdown()
```

## 包结构

```
role-ros2/
├── scripts/                    # ROS2 节点脚本
│   └── oculus_reader_node.py   # Oculus Quest 控制器节点
├── launch/                     # Launch 文件
│   ├── franka_robot.launch.py  # 启动机器人控制器
│   ├── zed_cameras.launch.py # 启动相机
│   └── oculus_controller.launch.py # 启动 Oculus 节点
├── config/                     # 配置文件
│   ├── hand_camera_params.yaml
│   └── static_camera_params.yaml
├── msg/                        # 自定义消息
│   └── OculusButtons.msg
└── role-ros2/                  # Python 模块
    ├── robot/                  # 机器人控制
    │   └── robot.py
    ├── robot_env.py            # Gym 环境
    ├── robot_ik/               # 逆运动学
    ├── calibration/            # 标定工具
    ├── camera_utils/           # 相机工具
    └── misc/                   # 工具函数
```

## 话题

### 机器人状态话题

- `/joint_states` (sensor_msgs/JointState) - 关节状态
- `/fr3/robot_state` (franka_msgs/FrankaRobotState) - 机器人状态
- `/fr3/current_pose` (geometry_msgs/PoseStamped) - 末端位姿

### 相机话题

- `/hand_camera/zed_node/rgb/image_rect_color` (sensor_msgs/Image) - Hand camera RGB
- `/hand_camera/zed_node/depth/depth_registered` (sensor_msgs/Image) - Hand camera 深度
- `/static_camera/zed_node/rgb/image_rect_color` (sensor_msgs/Image) - Static camera RGB
- `/static_camera/zed_node/depth/depth_registered` (sensor_msgs/Image) - Static camera 深度

### Oculus Quest 控制器话题

- `/oculus/right_controller/pose` (geometry_msgs/PoseStamped) - 右手控制器姿态
- `/oculus/left_controller/pose` (geometry_msgs/PoseStamped) - 左手控制器姿态
- `/oculus/buttons` (role_ros2/OculusButtons) - 按钮和摇杆状态
- `/oculus/controllers/markers` (visualization_msgs/MarkerArray) - RViz 可视化标记（可选）

## 配置

### 机器人配置

机器人配置通过 launch 文件参数控制：

- `arm_id` (默认: "fr3") - 机械臂 ID
- `controller_name` (默认: "fr3_arm_controller") - 控制器名称
- `robot_ip` - 机器人 IP 地址（必需）
- `use_fake_hardware` (默认: false) - 是否使用仿真硬件

### 相机配置

相机参数通过 YAML 配置文件设置：

- `config/hand_camera_params.yaml` - Hand camera 参数
- `config/static_camera_params.yaml` - Static camera 参数

### Oculus Reader 节点参数

- `publish_rate` (默认: 50.0) - 发布频率（Hz）
- `publish_tf` (默认: true) - 是否发布 TF 变换
- `publish_markers` (默认: false) - 是否发布 RViz 可视化标记
- `oculus_ip_address` (默认: '') - Oculus Quest IP 地址（空字符串表示 USB 连接）
- `oculus_port` (默认: 5555) - ADB 网络连接端口
- `frame_id` (默认: 'oculus_base') - 坐标系 ID

## 开发指南

### 添加新的机器人接口

1. 在 `role-ros2/robot/` 中添加新的机器人类
2. 实现标准的机器人接口方法
3. 更新 `__init__.py` 导出新类

### 扩展机器人环境

1. 继承 `RobotEnv` 类
2. 重写 `get_observation()` 和 `step()` 方法
3. 定义自定义的观察空间和动作空间

### 添加新的传感器

1. 创建传感器订阅节点
2. 在 `robot_env.py` 中集成传感器数据
3. 使用 `ApproximateTimeSynchronizer` 同步多传感器数据

## 故障排除

### 机器人连接问题

**问题：无法连接到机器人**
- 检查机器人 IP 地址是否正确
- 确保机器人和计算机在同一网络
- 检查防火墙设置

### 相机同步问题

**问题：相机数据不同步**
- 调整 `ApproximateTimeSynchronizer` 的 `slop` 参数
- 检查相机时间戳是否同步
- 确保相机发布频率一致

### Python 导入错误

**问题：`ModuleNotFoundError: No module named 'role_ros2'`**
- 确保已构建并 source 工作空间：`source ros2_ws/install/setup.bash`
- 检查 Python 路径：`python -c "import sys; print(sys.path)"`
- 验证包是否正确安装：`ros2 pkg list | grep role_ros2`

## 贡献

欢迎贡献代码和提出建议！请确保：

1. 代码遵循 ROS2 和 Python 最佳实践
2. 添加适当的文档和注释
3. 更新相关的 README 和文档

## 许可证

Apache-2.0

## 相关文档

- [ROS2 工作空间 README](../../README.md)
- [Franka ROS2 文档](https://github.com/souljaboy764/franka_ros2)
- [ZED ROS2 Wrapper 文档](https://github.com/stereolabs/zed-ros2-wrapper)
