#!/usr/bin/env bash
# Smoke test for ros2_xarm container.
# Verifies ROS2 Humble + xarm-python-sdk work together on the same Python.
# No real xArm required.
set -e

# Make sure ROS2 is sourced (in case the script is invoked non-interactively)
if [ -z "${ROS_DISTRO:-}" ] && [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
fi

TESTS_DIR="${TESTS_DIR:-/opt/ros2_xarm_tests}"

echo "[1/5] ROS2 CLI ..."
ros2 --help >/dev/null
echo "      ok"

echo "[2/5] ROS2 daemon ..."
ros2 daemon stop >/dev/null 2>&1 || true
ros2 daemon start >/dev/null
ros2 daemon status >/dev/null
echo "      ok"

echo "[3/5] test_ros2.py (rclpy pub/sub roundtrip) ..."
python3 "${TESTS_DIR}/test_ros2.py"

echo "[4/5] test_xarm_sdk.py (xarm import + do_not_open instance) ..."
python3 "${TESTS_DIR}/test_xarm_sdk.py"

echo "[5/5] test_combined.py (rclpy + xarm in one process) ..."
python3 "${TESTS_DIR}/test_combined.py"

echo ""
echo "All smoke tests PASSED"
