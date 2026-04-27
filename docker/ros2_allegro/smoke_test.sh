#!/usr/bin/env bash
# Smoke test for ros2_allegro container.
# Verifies ROS 2 Humble + ros2_control + the official Allegro driver build,
# without requiring real hardware. No CAN device is touched.
set -e

if [ -z "${ROS_DISTRO:-}" ] && [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
fi

if [ -f /app/ros2_ws/install/setup.bash ]; then
    source /app/ros2_ws/install/setup.bash
fi

echo "[1/5] ROS2 CLI ..."
ros2 --help >/dev/null
echo "      ok"

echo "[2/5] ros2_control_node binary ..."
which ros2_control_node >/dev/null
echo "      ok"

echo "[3/5] official allegro packages built ..."
ros2 pkg list | grep -E '^allegro_hand_(bringup|control_msgs|v4_hardware|io|utility)$' >/dev/null
echo "      ok"

echo "[4/5] role_ros2 messages built ..."
ros2 interface show role_ros2/msg/HandState >/dev/null
ros2 interface show role_ros2/srv/GraspPrimitive >/dev/null
echo "      ok"

echo "[5/5] allegro_hand_interface_node imports cleanly ..."
python3 -c "
import sys, importlib
sys.path.insert(0, '/app/ros2_ws/src/role-ros2/nodes')
spec = importlib.util.spec_from_file_location(
    'allegro_hand_interface_node',
    '/app/ros2_ws/src/role-ros2/nodes/allegro_hand_interface_node.py')
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)
assert hasattr(m, 'AllegroHandInterfaceNode')
assert hasattr(m, 'main')
print('      ok')
"

echo ""
echo "All smoke tests PASSED"
