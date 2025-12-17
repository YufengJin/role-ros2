# Franka Robot Controller 测试指南

本指南说明如何测试 Franka 机器人的 arm 和 gripper controller 是否正确启动。

## 前置要求

1. ROS2 Humble 环境已配置
2. `role_ros2` 包已构建
3. Franka 机器人已连接（或使用仿真模式）

## 启动机器人

### 方式 1: 使用默认配置

```bash
# 激活环境
source ros2_ws/start_env.sh

# 启动机器人（需要提供 robot_ip）
ros2 launch role_ros2 franka_robot.launch.py robot_ip:=172.16.0.2
```

### 方式 2: 使用配置文件

编辑 `config/franka_robot_config.yaml` 设置参数，然后通过 launch 参数传递：

```bash
ros2 launch role_ros2 franka_robot.launch.py \
    robot_ip:=172.16.0.2 \
    arm_id:=fr3 \
    load_gripper:=true \
    controller_name:=fr3_arm_controller
```

### 方式 3: 使用仿真模式（测试）

```bash
ros2 launch role_ros2 franka_robot.launch.py \
    robot_ip:=172.16.0.2 \
    use_fake_hardware:=true \
    load_gripper:=true
```

## 运行测试脚本

### 自动测试（推荐）

测试脚本会自动检查所有控制器和状态：

```bash
# 在另一个终端中运行（确保已 source 环境）
source ros2_ws/start_env.sh

# 运行测试
ros2 run role_ros2 test_franka_controllers

# 或指定 arm_id
ros2 run role_ros2 test_franka_controllers --arm-id fr3

# 或指定超时时间
ros2 run role_ros2 test_franka_controllers --timeout 60
```

### 手动测试

#### 1. 检查控制器管理器

```bash
# 列出所有控制器
ros2 service call /controller_manager/list_controllers \
    controller_manager_msgs/srv/ListControllers
```

#### 2. 检查控制器状态

```bash
# 检查控制器是否激活
ros2 control list_controllers
```

预期输出应包含：
- `joint_state_broadcaster` (active)
- `franka_robot_state_broadcaster` (active)
- `fr3_arm_controller` 或您配置的控制器 (active)
- `fr3_gripper_controller` (active, 如果启用了 gripper)

#### 3. 检查关节状态

```bash
# 监听关节状态
ros2 topic echo /joint_states

# 检查发布频率
ros2 topic hz /joint_states
```

#### 4. 检查机器人状态

```bash
# 监听机器人状态（仅真实硬件）
ros2 topic echo /fr3/robot_state

# 检查发布频率
ros2 topic hz /fr3/robot_state
```

#### 5. 检查 Gripper 控制器

```bash
# 检查 gripper action server
ros2 action list | grep gripper

# 应该看到类似：
# /fr3_gripper/gripper_action
```

#### 6. 测试 Arm 控制器

```bash
# 检查 arm action server
ros2 action list | grep follow_joint_trajectory

# 应该看到类似：
# /fr3_arm_controller/follow_joint_trajectory
```

## 测试脚本输出说明

测试脚本会检查以下项目：

### ✓ 必须通过的测试（Critical）

- **controller_manager**: 控制器管理器服务可用
- **joint_state_broadcaster**: 关节状态广播器已激活
- **arm_controller**: 机械臂控制器已激活
- **joint_states**: 关节状态消息正在发布

### ⚠ 可选测试（Optional）

- **franka_robot_state_broadcaster**: 机器人状态广播器（仅真实硬件）
- **robot_state**: 机器人状态消息（仅真实硬件）
- **gripper_controller**: 夹爪控制器（如果启用了 gripper）

## 常见问题

### 问题 1: 控制器管理器服务不可用

**症状**: `Controller manager service not available!`

**解决方案**:
- 确保 `ros2_control_node` 正在运行
- 检查 launch 文件是否正确启动
- 查看日志: `ros2 topic echo /rosout`

### 问题 2: 控制器未激活

**症状**: 控制器在列表中但状态不是 `active`

**解决方案**:
- 检查控制器配置是否正确
- 查看控制器日志: `ros2 topic echo /rosout | grep controller`
- 尝试手动启动: `ros2 control switch_controllers --start <controller_name>`

### 问题 3: 关节状态未发布

**症状**: `/joint_states` 话题没有消息

**解决方案**:
- 检查 `joint_state_broadcaster` 是否激活
- 检查硬件连接（真实机器人）
- 检查 `use_fake_hardware` 设置是否正确

### 问题 4: Gripper 控制器未找到

**症状**: 测试显示 gripper 控制器未激活

**解决方案**:
- 检查 `load_gripper` 参数是否为 `true`
- 检查 gripper launch 文件是否正确加载
- 查看 gripper 相关日志

## 快速验证命令

一键检查所有关键组件：

```bash
#!/bin/bash
echo "=== Controller Status ==="
ros2 control list_controllers

echo ""
echo "=== Joint States ==="
timeout 2 ros2 topic echo /joint_states --once || echo "No joint states"

echo ""
echo "=== Robot State ==="
timeout 2 ros2 topic echo /fr3/robot_state --once || echo "No robot state (may be using fake hardware)"

echo ""
echo "=== Action Servers ==="
ros2 action list | grep -E "(gripper|trajectory)"
```

## 下一步

如果所有测试通过，您可以：

1. **测试机器人运动**: 使用 `ros2 action send_goal` 发送轨迹目标
2. **测试 Gripper**: 使用 `ros2 action send_goal` 控制夹爪
3. **使用 Python API**: 参考 `role_ros2/robot/robot.py` 中的示例代码

