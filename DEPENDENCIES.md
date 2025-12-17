# Role-ROS2 依赖说明

## 当前状态

### ✅ 已移除的依赖（不再需要）

1. **franka_description** - 已移除
   - 原因：使用 role_ros2 自己的 URDF（从 fr3.xml 转换）
   - 位置：`role_ros2/robot_ik/franka/fr3.urdf`

2. **franka_bringup** - 已移除
   - 原因：不再使用 franka_ros2 的硬件接口和控制器

3. **franka_fr3_moveit_config** - 已移除
   - 原因：不再使用 MoveIt 配置

4. **controller_manager** - 已移除
   - 原因：使用 Polymetis 直接控制，不需要 ROS2 control 框架

5. **controller_manager_msgs** - 已移除
   - 原因：同上

### ⚠️ 可选依赖（根据使用情况）

1. **franka_gripper** - 已注释（可选）
   - 原因：polymetis_manager 已经通过 Polymetis 控制夹爪
   - 如果需要使用 franka_ros2 的夹爪节点，可以取消注释
   - 位置：`package.xml` 和 `launch/franka_robot.launch.py`

### ✅ 保留的依赖（仍在使用）

1. **franka_msgs** - 保留
   - 原因：`role_ros2/robot/robot.py` 使用 `FrankaRobotState` 和 `GripperMove` 消息
   - 注意：`robot.py` 是用于通过 franka_ros2 控制机器人的接口
   - 如果只使用 `polymetis_manager`，可以考虑移除 `robot.py` 和 `franka_msgs`

## 使用 Polymetis 的完整流程

如果**只使用 Polymetis**（推荐），则：

1. ✅ **不需要** `franka_description` - 使用自己的 URDF
2. ✅ **不需要** `franka_ros2` 硬件接口 - 使用 `polymetis_manager`
3. ✅ **不需要** `franka_gripper` - Polymetis 已包含夹爪控制
4. ⚠️ **可能需要** `franka_msgs` - 仅当使用 `robot.py` 时需要

## 检查依赖

运行以下命令检查是否还有未使用的依赖：

```bash
# 检查 package.xml 中的依赖
grep -r "franka_description\|franka_bringup\|franka_fr3_moveit_config\|controller_manager" ros2_ws/src/role-ros2/

# 检查代码中的导入
grep -r "from franka\|import franka" ros2_ws/src/role-ros2/role_ros2/
```

## 完全移除 franka_ros2 的步骤

如果要完全移除对 franka_ros2 的依赖：

1. 移除 `franka_msgs` 依赖（如果不再使用 `robot.py`）
2. 移除或重构 `role_ros2/robot/robot.py`（如果不再需要）
3. 移除测试脚本中对 franka_ros2 的引用

