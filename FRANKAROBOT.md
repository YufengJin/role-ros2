# Franka 机器人系统架构文档

本文档详细说明 `franka_robot.launch.py` 启动后的系统架构，包括节点、话题、动作和服务的功能。

## 系统启动状态分析

### ✅ 成功启动的组件

根据启动日志和系统检查，以下组件已成功启动：

1. **硬件连接**：成功连接到机器人 (172.17.0.2)
2. **控制器管理器**：正常运行，更新频率 1000 Hz
3. **所有控制器**：已加载并激活
4. **夹爪节点**：已连接并运行
5. **状态广播器**：正常发布机器人状态

### ⚠️ 已知问题

- `joint_state_publisher` 节点因 topic 名称格式问题失败（不影响系统运行，因为 `joint_state_broadcaster` 控制器已提供相同功能）

---

## 节点 (Nodes) 说明

### 核心系统节点

#### 1. `/controller_manager`
- **类型**：`controller_manager/ros2_control_node`
- **作用**：ROS2 控制框架的核心管理器
- **功能**：
  - 管理所有控制器的生命周期（加载、配置、激活、停用）
  - 协调硬件接口和控制器之间的通信
  - 提供控制器切换服务
  - 更新频率：1000 Hz（实时控制）
- **重要性**：⭐⭐⭐⭐⭐ **核心节点，必须运行**

#### 2. `/robot_state_publisher`
- **类型**：`robot_state_publisher`
- **作用**：发布机器人 TF 变换和机器人描述
- **功能**：
  - 发布 `/tf` 和 `/tf_static` 话题（机器人各关节的坐标变换）
  - 发布 `/robot_description` 话题（URDF 描述）
  - 为 RViz 和其他可视化工具提供机器人模型
- **重要性**：⭐⭐⭐⭐⭐ **可视化必需**

#### 3. `/ros2_control_node`
- **类型**：`controller_manager/ros2_control_node`（与 controller_manager 相同）
- **作用**：硬件抽象层，连接真实硬件和控制器
- **功能**：
  - 管理硬件接口（位置、速度、力矩）
  - 处理硬件状态和命令
  - 提供硬件参数设置服务
- **重要性**：⭐⭐⭐⭐⭐ **硬件控制必需**

### 控制器节点

#### 4. `/joint_state_broadcaster`
- **类型**：`joint_state_broadcaster/JointStateBroadcaster`
- **状态**：✅ active
- **作用**：发布所有关节的状态信息
- **功能**：
  - 订阅硬件关节状态
  - 发布 `/joint_states` 话题（包含位置、速度、力矩）
  - 为其他节点提供关节状态数据
- **重要性**：⭐⭐⭐⭐⭐ **状态监控必需**
- **输出话题**：`/joint_states`

#### 5. `/fr3_arm_controller`
- **类型**：`joint_trajectory_controller/JointTrajectoryController`
- **状态**：✅ active
- **作用**：机械臂轨迹控制器
- **功能**：
  - 接收关节轨迹动作目标
  - 执行平滑的关节空间轨迹
  - 支持位置、速度、加速度约束
  - 使用 PID 控制实现轨迹跟踪
- **重要性**：⭐⭐⭐⭐⭐ **机械臂控制核心**
- **动作接口**：`/fr3_arm_controller/follow_joint_trajectory`
- **话题**：
  - `/fr3_arm_controller/joint_trajectory` (命令)
  - `/fr3_arm_controller/state` (状态)

#### 6. `/franka_robot_state_broadcaster`
- **类型**：`franka_robot_state_broadcaster/FrankaRobotStateBroadcaster`
- **状态**：✅ active
- **作用**：发布 Franka 机器人特有的状态信息
- **功能**：
  - 发布机器人完整状态（关节、末端执行器、力/力矩等）
  - 提供笛卡尔空间信息（位置、速度、加速度）
  - 发布外部力/力矩信息
  - 提供碰撞检测相关信息
- **重要性**：⭐⭐⭐⭐ **高级状态监控**
- **输出话题**：多个（见下方 Topics 部分）

### 夹爪节点

#### 7. `/fr3_gripper`
- **类型**：`franka_gripper/franka_gripper_node`
- **状态**：✅ active
- **作用**：Franka 夹爪控制节点
- **功能**：
  - 控制夹爪开合
  - 执行抓取动作
  - 发布夹爪关节状态
  - 提供夹爪动作接口
- **重要性**：⭐⭐⭐⭐⭐ **夹爪控制必需**
- **动作接口**：多个（见下方 Actions 部分）
- **输出话题**：`/fr3_gripper/joint_states`

### 服务节点

#### 8. `/service_server`
- **类型**：Franka 硬件参数服务服务器
- **作用**：提供机器人参数设置服务
- **功能**：
  - 设置关节刚度
  - 设置笛卡尔刚度
  - 设置碰撞行为
  - 设置负载参数
  - 设置 TCP 和刚度框架
- **重要性**：⭐⭐⭐ **高级参数配置**
- **服务接口**：多个（见下方 Services 部分）

#### 9. `/action_server`
- **类型**：Franka 动作服务器
- **作用**：提供错误恢复等动作接口
- **功能**：
  - 错误恢复动作
  - 其他系统级动作
- **重要性**：⭐⭐⭐ **错误处理**

---

## 动作 (Actions) 说明

### 机械臂控制动作

#### `/fr3_arm_controller/follow_joint_trajectory`
- **类型**：`control_msgs/action/FollowJointTrajectory`
- **作用**：执行关节空间轨迹
- **使用场景**：
  - 移动到指定关节角度
  - 执行预定义的轨迹
  - 平滑的运动控制
- **消息结构**：
  - `trajectory`: 包含多个轨迹点（位置、速度、加速度、时间）
  - `joint_names`: 关节名称列表
- **重要性**：⭐⭐⭐⭐⭐ **主要控制接口**
- **使用示例**：
  ```python
  from control_msgs.action import FollowJointTrajectory
  from trajectory_msgs.msg import JointTrajectoryPoint
  
  goal = FollowJointTrajectory.Goal()
  goal.trajectory.joint_names = ['fr3_joint1', 'fr3_joint2', ...]
  point = JointTrajectoryPoint()
  point.positions = [0.0, -0.5, 0.0, -2.0, 0.0, 1.5, 0.0]
  point.time_from_start = rclpy.duration.Duration(seconds=2.0).to_msg()
  goal.trajectory.points = [point]
  ```

### 夹爪控制动作

#### `/fr3_gripper/move`
- **类型**：`franka_msgs/action/Move`
- **作用**：移动夹爪到指定宽度
- **参数**：
  - `width`: 目标宽度（米），范围 [0, 0.08]
  - `speed`: 移动速度（米/秒）
- **重要性**：⭐⭐⭐⭐⭐ **夹爪基本控制**
- **使用示例**：
  ```python
  from franka_msgs.action import Move
  
  goal = Move.Goal()
  goal.width = 0.04  # 4 cm
  goal.speed = 0.1   # 0.1 m/s
  ```

#### `/fr3_gripper/grasp`
- **类型**：`franka_msgs/action/Grasp`
- **作用**：执行抓取动作
- **参数**：
  - `width`: 目标抓取宽度
  - `speed`: 抓取速度
  - `force`: 抓取力（牛顿）
  - `epsilon.inner`: 内层容差
  - `epsilon.outer`: 外层容差
- **重要性**：⭐⭐⭐⭐ **精确抓取**
- **使用场景**：抓取物体时使用，可以检测抓取是否成功

#### `/fr3_gripper/homing`
- **类型**：`franka_msgs/action/Homing`
- **作用**：夹爪回零（归位）
- **重要性**：⭐⭐⭐ **初始化/校准**

#### `/fr3_gripper/gripper_action`
- **类型**：通用夹爪动作接口
- **作用**：兼容性接口
- **重要性**：⭐⭐ **兼容性**

### 系统动作

#### `/action_server/error_recovery`
- **类型**：错误恢复动作
- **作用**：从错误状态恢复
- **重要性**：⭐⭐⭐ **错误处理**

---

## 话题 (Topics) 说明

### 关节状态话题

#### `/joint_states`
- **类型**：`sensor_msgs/msg/JointState`
- **发布者**：`joint_state_broadcaster`
- **频率**：实时（跟随硬件更新）
- **内容**：
  - `name[]`: 关节名称列表
  - `position[]`: 关节位置（弧度）
  - `velocity[]`: 关节速度（弧度/秒）
  - `effort[]`: 关节力矩（牛·米）
- **重要性**：⭐⭐⭐⭐⭐ **核心状态信息**
- **使用场景**：
  - 获取当前机器人姿态
  - 监控关节运动
  - 计算逆运动学

#### `/franka/joint_states`
- **类型**：`sensor_msgs/msg/JointState`
- **发布者**：`ros2_control_node`（硬件接口）
- **作用**：机械臂原始关节状态（来自硬件）
- **重要性**：⭐⭐⭐⭐ **硬件原始数据**

#### `/fr3_gripper/joint_states`
- **类型**：`sensor_msgs/msg/JointState`
- **发布者**：`fr3_gripper` 节点
- **作用**：夹爪关节状态
- **内容**：夹爪两个手指的位置
- **重要性**：⭐⭐⭐⭐ **夹爪状态监控**

### 机器人状态话题

#### `/franka_robot_state_broadcaster/robot_state`
- **类型**：`franka_msgs/msg/FrankaRobotState`
- **发布者**：`franka_robot_state_broadcaster`
- **频率**：实时
- **内容**：完整的机器人状态，包括：
  - 关节状态（位置、速度、力矩）
  - 末端执行器状态（位置、速度、加速度）
  - 力/力矩信息
  - 碰撞检测状态
  - 控制器状态
- **重要性**：⭐⭐⭐⭐⭐ **最完整的状态信息**
- **使用场景**：
  - 高级控制算法
  - 力控制
  - 碰撞检测
  - 状态监控

#### `/franka_robot_state_broadcaster/current_pose`
- **类型**：`geometry_msgs/msg/PoseStamped`
- **作用**：当前末端执行器位姿
- **重要性**：⭐⭐⭐⭐ **笛卡尔空间信息**

#### `/franka_robot_state_broadcaster/measured_joint_states`
- **类型**：`sensor_msgs/msg/JointState`
- **作用**：测量的关节状态（带时间戳）
- **重要性**：⭐⭐⭐⭐ **带时间戳的状态**

#### `/franka_robot_state_broadcaster/desired_joint_states`
- **类型**：`sensor_msgs/msg/JointState`
- **作用**：期望的关节状态（控制器目标）
- **重要性**：⭐⭐⭐ **控制目标监控**

#### `/franka_robot_state_broadcaster/desired_end_effector_twist`
- **类型**：`geometry_msgs/msg/TwistStamped`
- **作用**：期望的末端执行器速度
- **重要性**：⭐⭐⭐ **速度控制监控**

#### `/franka_robot_state_broadcaster/external_wrench_in_base_frame`
- **类型**：`geometry_msgs/msg/WrenchStamped`
- **作用**：基坐标系下的外部力/力矩
- **重要性**：⭐⭐⭐⭐ **力控制必需**

#### `/franka_robot_state_broadcaster/external_wrench_in_stiffness_frame`
- **类型**：`geometry_msgs/msg/WrenchStamped`
- **作用**：刚度坐标系下的外部力/力矩
- **重要性**：⭐⭐⭐ **力控制（刚度框架）**

#### `/franka_robot_state_broadcaster/external_joint_torques`
- **类型**：`std_msgs/msg/Float64MultiArray`
- **作用**：外部关节力矩
- **重要性**：⭐⭐⭐ **力矩监控**

#### `/franka_robot_state_broadcaster/last_desired_pose`
- **类型**：`geometry_msgs/msg/PoseStamped`
- **作用**：上一个期望的末端执行器位姿
- **重要性**：⭐⭐ **历史信息**

### 控制器话题

#### `/fr3_arm_controller/joint_trajectory`
- **类型**：`trajectory_msgs/msg/JointTrajectory`
- **订阅者**：`fr3_arm_controller`
- **作用**：关节轨迹命令（通过话题发送，替代动作）
- **重要性**：⭐⭐⭐ **话题式控制**

#### `/fr3_arm_controller/state`
- **类型**：`control_msgs/msg/JointTrajectoryControllerState`
- **发布者**：`fr3_arm_controller`
- **作用**：控制器状态（当前轨迹执行状态）
- **重要性**：⭐⭐⭐ **控制器监控**

#### `/fr3_arm_controller/transition_event`
- **类型**：控制器状态转换事件
- **作用**：通知控制器状态变化
- **重要性**：⭐⭐ **事件监控**

### TF 话题

#### `/tf`
- **类型**：`tf2_msgs/msg/TFMessage`
- **发布者**：`robot_state_publisher`
- **作用**：动态坐标变换（实时更新）
- **重要性**：⭐⭐⭐⭐⭐ **坐标变换核心**

#### `/tf_static`
- **类型**：`tf2_msgs/msg/TFMessage`
- **发布者**：`robot_state_publisher`
- **作用**：静态坐标变换（不变的部分）
- **重要性**：⭐⭐⭐⭐⭐ **坐标变换核心**

#### `/robot_description`
- **类型**：`std_msgs/msg/String`
- **发布者**：`robot_state_publisher`
- **作用**：机器人 URDF 描述（XML 格式）
- **重要性**：⭐⭐⭐⭐ **模型信息**

### 系统话题

#### `/dynamic_joint_states`
- **类型**：`control_msgs/msg/DynamicJointState`
- **作用**：动态关节状态（包含接口信息）
- **重要性**：⭐⭐⭐ **ros2_control 接口信息**

#### `/parameter_events`
- **类型**：`rcl_interfaces/msg/ParameterEvent`
- **作用**：参数变化事件
- **重要性**：⭐⭐ **参数监控**

#### `/rosout`
- **类型**：`rcl_interfaces/msg/Log`
- **作用**：日志消息
- **重要性**：⭐⭐ **调试信息**

---

## 服务 (Services) 说明

### 控制器管理服务

#### `/controller_manager/list_controllers`
- **类型**：`controller_manager_msgs/srv/ListControllers`
- **作用**：列出所有控制器及其状态
- **重要性**：⭐⭐⭐⭐ **控制器监控**

#### `/controller_manager/switch_controllers`
- **类型**：`controller_manager_msgs/srv/SwitchControllers`
- **作用**：切换控制器（激活/停用）
- **参数**：
  - `activate_controllers[]`: 要激活的控制器列表
  - `deactivate_controllers[]`: 要停用的控制器列表
  - `strictness`: 严格模式（1=STRICT, 2=BEST_EFFORT）
- **重要性**：⭐⭐⭐⭐⭐ **控制器切换核心**
- **使用场景**：在不同控制模式间切换

#### `/controller_manager/load_controller`
- **类型**：`controller_manager_msgs/srv/LoadController`
- **作用**：加载控制器
- **重要性**：⭐⭐⭐ **动态加载**

#### `/controller_manager/unload_controller`
- **类型**：`controller_manager_msgs/srv/UnloadController`
- **作用**：卸载控制器
- **重要性**：⭐⭐⭐ **动态卸载**

#### `/controller_manager/list_controller_types`
- **类型**：列出可用的控制器类型
- **重要性**：⭐⭐ **信息查询**

#### `/controller_manager/list_hardware_components`
- **类型**：列出硬件组件
- **重要性**：⭐⭐ **硬件信息**

#### `/controller_manager/list_hardware_interfaces`
- **类型**：列出硬件接口
- **重要性**：⭐⭐ **接口信息**

### 机器人参数服务

#### `/service_server/set_joint_stiffness`
- **类型**：`franka_msgs/srv/SetJointStiffness`
- **作用**：设置关节刚度
- **参数**：7 个关节的刚度值
- **重要性**：⭐⭐⭐⭐ **阻抗控制**

#### `/service_server/set_cartesian_stiffness`
- **类型**：`franka_msgs/srv/SetCartesianStiffness`
- **作用**：设置笛卡尔空间刚度
- **参数**：6x6 刚度矩阵
- **重要性**：⭐⭐⭐⭐ **笛卡尔阻抗控制**

#### `/service_server/set_force_torque_collision_behavior`
- **类型**：`franka_msgs/srv/SetForceTorqueCollisionBehavior`
- **作用**：设置力/力矩碰撞行为
- **参数**：碰撞阈值和响应
- **重要性**：⭐⭐⭐⭐⭐ **安全设置**

#### `/service_server/set_full_collision_behavior`
- **类型**：`franka_msgs/srv/SetFullCollisionBehavior`
- **作用**：设置完整碰撞行为
- **重要性**：⭐⭐⭐⭐⭐ **安全设置**

#### `/service_server/set_load`
- **类型**：`franka_msgs/srv/SetLoad`
- **作用**：设置末端负载参数
- **参数**：质量、质心、惯性矩阵
- **重要性**：⭐⭐⭐⭐ **负载补偿**

#### `/service_server/set_tcp_frame`
- **类型**：`franka_msgs/srv/SetTCPFrame`
- **作用**：设置 TCP（工具中心点）框架
- **重要性**：⭐⭐⭐⭐ **工具校准**

#### `/service_server/set_stiffness_frame`
- **类型**：`franka_msgs/srv/SetStiffnessFrame`
- **作用**：设置刚度框架
- **重要性**：⭐⭐⭐ **阻抗控制框架**

### 夹爪服务

#### `/fr3_gripper/stop`
- **类型**：停止夹爪运动
- **作用**：紧急停止夹爪
- **重要性**：⭐⭐⭐⭐⭐ **安全功能**

---

## 必需的组件清单

### 🔴 必须启动的 Action Servers

以下 Action Servers **必须**可用，否则系统无法正常工作：

1. **`/fr3_arm_controller/follow_joint_trajectory`**
   - **类型**：`control_msgs/action/FollowJointTrajectory`
   - **必需性**：⭐⭐⭐⭐⭐ **核心控制接口**
   - **用途**：机械臂轨迹控制
   - **测试**：`ros2 action list | grep follow_joint_trajectory`

2. **`/fr3_gripper/move`**
   - **类型**：`franka_msgs/action/Move`
   - **必需性**：⭐⭐⭐⭐⭐ **夹爪基本控制**
   - **用途**：夹爪开合控制
   - **测试**：`ros2 action list | grep gripper/move`

3. **`/fr3_gripper/grasp`**
   - **类型**：`franka_msgs/action/Grasp`
   - **必需性**：⭐⭐⭐⭐ **精确抓取**
   - **用途**：执行抓取动作
   - **测试**：`ros2 action list | grep gripper/grasp`

### 🔴 必须启动的 Topics（必须有数据流）

以下 Topics **必须**存在且有数据发布：

1. **`/joint_states`**
   - **类型**：`sensor_msgs/msg/JointState`
   - **发布者**：`joint_state_broadcaster`
   - **必需性**：⭐⭐⭐⭐⭐ **核心状态信息**
   - **测试**：`ros2 topic hz /joint_states`

2. **`/franka/joint_states`**
   - **类型**：`sensor_msgs/msg/JointState`
   - **发布者**：`ros2_control_node`（硬件接口）
   - **必需性**：⭐⭐⭐⭐⭐ **硬件原始数据**
   - **测试**：`ros2 topic hz /franka/joint_states`

3. **`/fr3_gripper/joint_states`**
   - **类型**：`sensor_msgs/msg/JointState`
   - **发布者**：`fr3_gripper` 节点
   - **必需性**：⭐⭐⭐⭐⭐ **夹爪状态监控**
   - **测试**：`ros2 topic hz /fr3_gripper/joint_states`

4. **`/franka_robot_state_broadcaster/robot_state`**
   - **类型**：`franka_msgs/msg/FrankaRobotState`
   - **发布者**：`franka_robot_state_broadcaster`
   - **必需性**：⭐⭐⭐⭐⭐ **最完整的状态信息**
   - **测试**：`ros2 topic hz /franka_robot_state_broadcaster/robot_state`

5. **`/franka_robot_state_broadcaster/current_pose`**
   - **类型**：`geometry_msgs/msg/PoseStamped`
   - **发布者**：`franka_robot_state_broadcaster`
   - **必需性**：⭐⭐⭐⭐ **笛卡尔空间信息**
   - **测试**：`ros2 topic hz /franka_robot_state_broadcaster/current_pose`

6. **`/tf`**
   - **类型**：`tf2_msgs/msg/TFMessage`
   - **发布者**：`robot_state_publisher`
   - **必需性**：⭐⭐⭐⭐⭐ **坐标变换核心**
   - **测试**：`ros2 topic hz /tf`

7. **`/tf_static`**
   - **类型**：`tf2_msgs/msg/TFMessage`
   - **发布者**：`robot_state_publisher`
   - **必需性**：⭐⭐⭐⭐⭐ **静态坐标变换**
   - **测试**：`ros2 topic hz /tf_static`

8. **`/robot_description`**
   - **类型**：`std_msgs/msg/String`
   - **发布者**：`robot_state_publisher`
   - **必需性**：⭐⭐⭐⭐ **模型信息**
   - **测试**：`ros2 topic echo /robot_description -n 1`

### 🔴 必须启动的 Nodes

以下 Nodes **必须**运行：

1. **`/controller_manager`** - 控制器管理器
2. **`/robot_state_publisher`** - TF 和机器人描述发布
3. **`/joint_state_broadcaster`** - 关节状态广播
4. **`/fr3_arm_controller`** - 机械臂控制器
5. **`/franka_robot_state_broadcaster`** - Franka 状态广播
6. **`/fr3_gripper`** - 夹爪节点

### 🔴 必须启动的 Services

以下 Services **必须**可用：

1. **`/controller_manager/list_controllers`** - 列出控制器
2. **`/controller_manager/switch_controllers`** - 切换控制器
3. **`/controller_manager/load_controller`** - 加载控制器
4. **`/fr3_gripper/stop`** - 紧急停止夹爪

---

## 自动测试必需组件

### 使用测试脚本

我们提供了一个自动化测试脚本来检查所有必需的组件：

```bash
# 在启动机器人后，运行测试脚本
ros2 run role_ros2 test_required_components
```

**测试脚本会检查**：
- ✅ 所有必需的 Nodes 是否运行
- ✅ 所有必需的 Action Servers 是否可用
- ✅ 所有必需的 Topics 是否存在且有数据流
- ✅ 所有必需的 Services 是否可用

**输出示例**：
```
======================================================================
Franka 机器人系统必需组件测试
======================================================================

[1/4] 测试必需的 Nodes...
✓ controller_manager
✓ robot_state_publisher
✓ joint_state_broadcaster
✓ fr3_arm_controller
✓ franka_robot_state_broadcaster
✓ fr3_gripper

[2/4] 测试必需的 Action Servers...
✓ /fr3_arm_controller/follow_joint_trajectory
✓ /fr3_gripper/move
✓ /fr3_gripper/grasp

[3/4] 测试必需的 Topics...
✓ /joint_states (exists: True, has_data: True)
✓ /franka/joint_states (exists: True, has_data: True)
✓ /fr3_gripper/joint_states (exists: True, has_data: True)
✓ /franka_robot_state_broadcaster/robot_state (exists: True, has_data: True)
✓ /franka_robot_state_broadcaster/current_pose (exists: True, has_data: True)
✓ /tf (exists: True, has_data: True)
✓ /tf_static (exists: True, has_data: True)
✓ /robot_description (exists: True, has_data: True)

[4/4] 测试必需的 Services...
✓ /controller_manager/list_controllers
✓ /controller_manager/switch_controllers
✓ /controller_manager/load_controller
✓ /fr3_gripper/stop

======================================================================
测试报告
======================================================================
...
总结:
  Nodes:      ✓ 全部通过
  Actions:    ✓ 全部通过
  Topics:     ✓ 全部通过
  Services:   ✓ 全部通过
======================================================================
```

### 手动测试命令

如果测试脚本不可用，可以使用以下命令手动检查：

#### 检查 Action Servers
```bash
# 列出所有 actions
ros2 action list

# 检查特定 action 服务器
ros2 action info /fr3_arm_controller/follow_joint_trajectory
ros2 action info /fr3_gripper/move
ros2 action info /fr3_gripper/grasp
```

#### 检查 Topics
```bash
# 列出所有 topics
ros2 topic list

# 检查 topic 数据流
ros2 topic hz /joint_states
ros2 topic hz /franka_robot_state_broadcaster/robot_state
ros2 topic hz /tf

# 查看 topic 数据
ros2 topic echo /joint_states -n 1
```

#### 检查 Nodes
```bash
# 列出所有 nodes
ros2 node list

# 检查特定 node 信息
ros2 node info /controller_manager
ros2 node info /fr3_arm_controller
```

#### 检查 Services
```bash
# 列出所有 services
ros2 service list

# 检查特定 service
ros2 service type /controller_manager/list_controllers
ros2 service call /controller_manager/list_controllers controller_manager_msgs/srv/ListControllers
```

---

## 应该关注的关键信息

### 🔴 必须监控（系统健康）

1. **控制器状态**
   - 检查：`ros2 control list_controllers`
   - 确保：所有必需控制器处于 `active` 状态
   - 关注：`fr3_arm_controller`, `joint_state_broadcaster`, `franka_robot_state_broadcaster`

2. **硬件连接**
   - 检查：启动日志中的 "Successfully connected to robot"
   - 监控：`/franka_robot_state_broadcaster/robot_state` 话题是否有数据
   - 警告：如果连接断开，所有控制将失效

3. **关节状态**
   - 监控：`/joint_states` 话题
   - 检查：数据更新频率是否正常
   - 验证：关节值是否在合理范围内

### 🟡 重要监控（功能正常）

4. **控制器执行状态**
   - 监控：`/fr3_arm_controller/state` 话题
   - 检查：轨迹执行是否正常
   - 关注：错误状态和完成状态

5. **末端执行器状态**
   - 监控：`/franka_robot_state_broadcaster/current_pose` 话题
   - 检查：位姿更新是否正常
   - 验证：位置和姿态是否合理

6. **夹爪状态**
   - 监控：`/fr3_gripper/joint_states` 话题
   - 检查：夹爪宽度是否在 [0, 0.08] 米范围内
   - 验证：动作执行是否成功

### 🟢 可选监控（高级功能）

7. **力/力矩信息**
   - 监控：`/franka_robot_state_broadcaster/external_wrench_in_base_frame`
   - 用途：力控制、碰撞检测
   - 注意：异常大的力可能表示碰撞

8. **控制器切换**
   - 监控：`/controller_manager/switch_controller` 服务调用（注意：单数）
   - 用途：在不同控制模式间切换
   - 注意：切换时确保安全

9. **参数设置**
   - 监控：通过 `/service_server/*` 服务设置的参数
   - 用途：调整机器人行为（刚度、碰撞行为等）
   - 注意：不当的参数可能导致不安全行为

---

## 常见使用场景

### 场景 1：基本运动控制

**关注话题**：
- `/joint_states` - 获取当前位置
- `/fr3_arm_controller/follow_joint_trajectory` - 发送轨迹目标

**使用动作**：
- `/fr3_arm_controller/follow_joint_trajectory` - 执行轨迹

### 场景 2：力控制

**关注话题**：
- `/franka_robot_state_broadcaster/external_wrench_in_base_frame` - 外部力/力矩
- `/franka_robot_state_broadcaster/current_pose` - 当前位置

**使用服务**：
- `/service_server/set_cartesian_stiffness` - 设置刚度
- `/service_server/set_force_torque_collision_behavior` - 设置碰撞行为

### 场景 3：抓取任务

**关注话题**：
- `/fr3_gripper/joint_states` - 夹爪状态
- `/franka_robot_state_broadcaster/current_pose` - 末端位置

**使用动作**：
- `/fr3_gripper/move` - 移动到目标宽度
- `/fr3_gripper/grasp` - 执行抓取

### 场景 4：状态监控

**关注话题**：
- `/franka_robot_state_broadcaster/robot_state` - 完整状态
- `/joint_states` - 关节状态
- `/tf` - 坐标变换

---

## 故障排除

### 问题 1：控制器未激活

**检查**：
```bash
ros2 control list_controllers
```

**解决**：
```bash
ros2 control set_controller_state <controller_name> active
```

### 问题 2：话题无数据

**检查**：
```bash
ros2 topic echo /joint_states
ros2 topic hz /joint_states
```

**解决**：检查硬件连接和控制器状态

### 问题 3：动作服务器不可用

**检查**：
```bash
ros2 action list
ros2 action info /fr3_arm_controller/follow_joint_trajectory
```

**解决**：确保控制器已激活

### 问题 4：TF 变换缺失

**检查**：
```bash
ros2 run tf2_ros tf2_echo base fr3_link8
```

**解决**：确保 `robot_state_publisher` 节点运行正常

---

## 最佳实践

1. **启动后验证**：
   ```bash
   # 检查所有控制器
   ros2 control list_controllers
   
   # 检查话题数据流
   ros2 topic hz /joint_states
   ros2 topic hz /franka_robot_state_broadcaster/robot_state
   
   # 检查动作服务器
   ros2 action list
   ```

2. **监控关键话题**：
   - 始终监控 `/joint_states` 确保数据流正常
   - 监控 `/franka_robot_state_broadcaster/robot_state` 获取完整状态
   - 监控 `/fr3_arm_controller/state` 了解控制器执行状态

3. **安全设置**：
   - 在开始控制前设置碰撞行为
   - 根据负载设置负载参数
   - 使用适当的刚度值

4. **错误处理**：
   - 监听 `/action_server/error_recovery` 动作
   - 监控控制器状态转换事件
   - 实现超时和错误恢复机制

---

## 相关文档

- [Franka ROS2 官方文档](https://github.com/frankarobotics/franka_ros2)
- [ROS2 Control 文档](https://control.ros.org/)
- [role_ros2 包文档](README.md)
- [控制测试文档](CONTROL_TESTING.md)

