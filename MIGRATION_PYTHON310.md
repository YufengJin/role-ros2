# Python 3.10 迁移指南

## 为什么需要升级？

ROS2 Humble 需要 Python 3.10。如果使用 Python 3.8 或 3.9，会出现以下错误：

```
ModuleNotFoundError: No module named 'rclpy._rclpy_pybind11'
The C extension '/opt/ros/humble/lib/python3.10/site-packages/_rclpy_pybind11.cpython-38-x86_64-linux-gnu.so' isn't present on the system.
```

这是因为 ROS2 系统安装的 C 扩展模块是为 Python 3.10 编译的，与 Python 3.8 不兼容。

## 迁移步骤

### 1. 备份当前环境（可选但推荐）

```bash
# 导出当前环境的包列表
conda activate robot
conda env export -n robot > robot_env_py38_backup.yml
pip freeze > robot_pip_py38_backup.txt
```

### 2. 创建新的 Python 3.10 环境

```bash
# 创建新环境
conda create -n robot python=3.10

# 激活新环境
conda activate robot
```

### 3. 重新安装项目依赖

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

### 4. 验证安装

```bash
# 激活 ROS2 环境
source ros2_ws/src/droid_ros2/start_env.sh

# 验证 Python 版本
python --version  # 应该显示 Python 3.10.x

# 验证 rclpy
python -c "import rclpy; print('rclpy version:', rclpy.__version__)"

# 验证其他关键包
python -c "import pyzed; print('pyzed available')"
```

### 5. 清理旧环境（可选）

```bash
# 如果新环境工作正常，可以删除旧环境
conda env remove -n robot_py38_backup  # 如果重命名了旧环境
```

## 常见问题

### Q: 如何同时保留两个环境？

A: 可以重命名旧环境：

```bash
# 重命名旧环境
conda create -n robot_py38 --clone robot
conda env remove -n robot

# 创建新的 Python 3.10 环境
conda create -n robot python=3.10
```

### Q: 某些包在 Python 3.10 下不兼容怎么办？

A: 大多数现代 Python 包都支持 Python 3.10。如果遇到兼容性问题：

1. 检查包的最新版本是否支持 Python 3.10
2. 查看包的 GitHub issues 或文档
3. 考虑使用替代包或等待更新

### Q: 如何验证所有依赖都已正确安装？

A: 运行项目的测试或启动脚本：

```bash
# 测试 ROS2 节点
ros2 launch droid_ros2 droid_cameras.launch.py

# 测试 Oculus Reader
ros2 launch droid_ros2 oculus_reader.launch.py
```

## 回滚（如果需要）

如果新环境有问题，可以回滚到旧环境：

```bash
# 如果备份了环境文件
conda env create -n robot_py38 --file robot_env_py38_backup.yml
conda activate robot_py38
pip install -r robot_pip_py38_backup.txt
```

