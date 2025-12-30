# FrankaRobot 与 polymetis_bridge_node 接口验证

## 接口匹配检查结果

### ✅ Topics (已匹配)

| robot.py 使用 | polymetis_bridge_node 提供 | 状态 |
|--------------|---------------------------|------|
| `/polymetis/robot_state` (订阅) | `/polymetis/robot_state` (发布) | ✅ 匹配 |
| `/polymetis/gripper_state` (订阅) | `/polymetis/gripper_state` (发布) | ✅ 匹配 |
| `/polymetis/robot_command` (发布) | `/polymetis/robot_command` (订阅) | ✅ 匹配 |
| `/polymetis/gripper/command` (发布) | `/polymetis/gripper/command` (订阅) | ✅ 匹配 |

### ✅ Services (已匹配)

| robot.py 使用 | polymetis_bridge_node 提供 | 状态 |
|--------------|---------------------------|------|
| `/polymetis/reset` | `/polymetis/reset` | ✅ 匹配 |
| `/polymetis/arm/start_cartesian_impedance` | `/polymetis/arm/start_cartesian_impedance` | ✅ 匹配 |
| `/polymetis/arm/start_joint_impedance` | `/polymetis/arm/start_joint_impedance` | ✅ 匹配 |
| `/polymetis/arm/terminate_policy` | `/polymetis/arm/terminate_policy` | ✅ 匹配 |
| `/polymetis/arm/move_to_joint_positions` | `/polymetis/arm/move_to_joint_positions` | ✅ 匹配 |
| `/polymetis/arm/move_to_ee_pose` | `/polymetis/arm/move_to_ee_pose` | ✅ 匹配 |
| `/polymetis/arm/solve_ik` | `/polymetis/arm/solve_ik` | ✅ 匹配 |
| `/polymetis/arm/compute_fk` | `/polymetis/arm/compute_fk` | ✅ 匹配 |
| `/polymetis/arm/compute_time_to_go` | `/polymetis/arm/compute_time_to_go` | ✅ 匹配 |

### ✅ 已修复的问题

1. **Home Service 已删除**
   - `polymetis_bridge_node.py`: 已删除 `/polymetis/home` service
   - `robot.py`: 已更新，`home()` 方法现在调用 `reset()`
   - 状态: ✅ 已修复

2. **接口一致性**
   - 所有 topics 和 services 名称完全匹配
   - 消息类型匹配
   - 状态: ✅ 已验证

## 测试脚本

### 使用方法

```bash
# 基本测试（需要运行 polymetis_bridge_node）
python3 test_robot.py

# 跳过运动测试（只测试状态获取和接口）
python3 test_robot.py --skip-motion

# 跳过 reset 测试
python3 test_robot.py --skip-reset

# 组合选项
python3 test_robot.py --skip-reset --skip-motion
```

### 测试覆盖

测试脚本 `test_robot.py` 覆盖以下功能：

1. **状态获取**
   - `get_joint_positions()`
   - `get_joint_velocities()`
   - `get_gripper_position()`
   - `get_gripper_state()`
   - `get_ee_pose()`
   - `get_robot_state()`

2. **机器人控制**
   - `reset()` / `home()`
   - `update_gripper()` (blocking/non-blocking)
   - `update_joints()` (blocking/non-blocking, position/velocity)
   - `update_pose()` (blocking/non-blocking, position/velocity)
   - `update_command()`

3. **控制器管理**
   - `start_joint_impedance()`
   - `start_cartesian_impedance()`
   - `terminate_current_policy()`

4. **运动学计算**
   - `solve_inverse_kinematics()`
   - `compute_forward_kinematics()`
   - `compute_time_to_go()`

5. **工具函数**
   - `create_action_dict()`

## 运行测试

### 前置条件

1. 确保 `polymetis_bridge_node` 正在运行：
   ```bash
   ros2 run role_ros2 polymetis_bridge_node
   ```

2. 确保 ROS2 workspace 已构建和 sourced：
   ```bash
   cd /home/yjin/repos/ros2_ws
   colcon build --packages-select role_ros2
   source install/setup.bash
   ```

### 执行测试

```bash
cd /home/yjin/repos/ros2_ws/src/role-ros2/scripts
python3 test_robot.py
```

### 预期输出

测试脚本会：
- 测试每个方法的功能
- 显示通过/失败状态
- 在最后提供测试总结

示例输出：
```
============================================================
FrankaRobot Interface Test
============================================================
...

--- Testing State Getters ---
✅ PASS: get_joint_positions
✅ PASS: get_joint_velocities
...

============================================================
TEST SUMMARY
============================================================
Passed:  25
Failed:  0
Skipped: 0
Total:   25
============================================================
```

## 注意事项

1. **运动测试**: 如果使用真实机器人，确保机器人有足够的运动空间
2. **超时设置**: 某些服务调用可能需要更长时间，可以通过 `--timeout` 参数调整
3. **Mock 模式**: 当前测试脚本不支持 mock 模式，需要实际的 `polymetis_bridge_node` 运行

## 接口变更历史

- **2024-12-29**: 删除 `/polymetis/home` service，`home()` 方法改为调用 `reset()`
- **2024-12-29**: 添加启动时自动 reset 功能到 `polymetis_bridge_node`

