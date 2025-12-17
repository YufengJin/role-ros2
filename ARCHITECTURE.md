# Role-ROS2 架构说明

## 系统架构

### 三层架构设计

```
┌─────────────────────────────────────┐
│   robot_env.py (Gym-like API)      │  ← 最外层：Gym接口
│   - step(action)                   │
│   - reset()                         │
│   - get_observation()              │
└──────────────┬──────────────────────┘
               │ 使用API
               ▼
┌─────────────────────────────────────┐
│   robot.py (Robot API)              │  ← 中间层：提供API
│   - 订阅 /joint_states              │
│   - 发布控制命令到 topics           │
│   - update_command()                │
│   - update_joints()                 │
│   - update_gripper()                │
└──────────────┬──────────────────────┘
               │ 通过ROS2 topics
               ▼
┌─────────────────────────────────────┐
│   polymetis_manager.py               │  ← 通信层：与硬件通信
│   - 发布 /joint_states              │
│   - 订阅控制命令 topics              │
│   - 启动 Polymetis controller       │
│   - 启动 Polymetis robot            │
│   - 实现不同控制模式                 │
└─────────────────────────────────────┘
```

## 数据流

### 状态流（State Flow）
```
Polymetis Hardware
    ↓
polymetis_manager.py
    ↓ (发布)
/joint_states (sensor_msgs/JointState)
    ↓ (订阅)
robot.py
    ↓ (提供API)
robot_env.py
```

### 控制流（Control Flow）
```
robot_env.py
    ↓ (调用API)
robot.py
    ↓ (发布)
polymetis_manager/robot_command (role_ros2/PolymetisRobotCommand)
    ↓ (订阅)
polymetis_manager.py
    ↓ (执行)
Polymetis Hardware
```

## 组件说明

### 1. polymetis_manager.py（通信层）

**职责**：
- 启动 Polymetis controller 和 robot
- 发布 `/joint_states`（arm + gripper）
- 订阅控制命令 topics
- 实现不同控制模式（cartesian/joint + position/velocity）

**关键功能**：
- `launch_controller()` - 启动 Polymetis 服务器
- `launch_robot()` - 连接 Polymetis 接口
- `_publish_joint_states()` - 发布关节状态
- `_command_callback()` - 处理控制命令
- `_handle_*_command()` - 实现不同控制模式

**关节名称**：
- 从 URDF 文件自动读取（使用 `urdf_utils.py`）
- 支持配置文件覆盖
- 确保与 URDF 中的关节名称一致

### 2. robot.py（API 层）

**职责**：
- 订阅 `/joint_states` 获取机器人状态
- 发布控制命令到 `polymetis_manager/robot_command` topic
- 提供高级 API（`update_command`, `update_joints`, `update_gripper`）
- 计算正向运动学（从关节位置计算末端执行器位姿）

**关键功能**：
- `_joint_state_callback()` - 更新关节状态
- `update_command()` - 发布控制命令
- `get_ee_pose()` - 计算末端执行器位姿（使用正向运动学）
- `create_action_dict()` - 创建动作字典

### 3. robot_env.py（Gym 接口层）

**职责**：
- 提供 Gym-like 接口（`step`, `reset`, `get_observation`）
- 管理相机订阅
- 协调机器人控制和观察

## 控制模式

支持四种控制模式：

1. **cartesian_position** - 笛卡尔空间位置控制
2. **cartesian_velocity** - 笛卡尔空间速度控制
3. **joint_position** - 关节空间位置控制
4. **joint_velocity** - 关节空间速度控制

所有模式都通过 `PolymetisRobotCommand` 消息发送到 `polymetis_manager`。

## 消息定义

### PolymetisRobotCommand.msg
```msg
string action_space  # 控制模式
string gripper_action_space  # 夹爪控制模式
float64[] command  # 控制命令数组
bool blocking  # 是否阻塞等待
bool velocity  # 是否为速度命令
float64[] cartesian_noise  # 可选的笛卡尔噪声
```

## 关节名称配置

关节名称从 URDF 文件自动提取，确保与 URDF 一致：

- **Arm joints**: `fr3_panda_joint1` 到 `fr3_panda_joint7`
- **Gripper joints**: `fr3_finger_joint1`, `fr3_finger_joint2`

配置文件：`config/joint_names.yaml`（可选，用于覆盖）

## 启动流程

1. `polymetis_manager` 启动：
   - 调用 `launch_controller()` 启动 Polymetis 服务器
   - 调用 `launch_robot()` 连接机器人
   - 启动笛卡尔阻抗控制器
   - 开始发布 `/joint_states`

2. `robot.py` 启动：
   - 订阅 `/joint_states`
   - 发布控制命令到 `polymetis_manager/robot_command`

3. `robot_env.py` 启动：
   - 初始化 `robot.py`
   - 初始化相机订阅
   - 准备 Gym 接口

## 使用示例

```python
import rclpy
from role_ros2.robot_env import RobotEnv

rclpy.init()
env = RobotEnv(action_space="cartesian_velocity")

# Reset
obs = env.reset()

# Step
action = [0.01, 0, 0, 0, 0, 0, 0]  # 7 DOF: 6 cartesian + 1 gripper
obs = env.step(action)
```

