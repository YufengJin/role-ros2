#!/usr/bin/env python3
"""
Test script for ROS2 and Polymetis integration.
Verifies that both can be imported and used together without conflicts.
"""

import sys


def test_polymetis():
    """Test Polymetis import."""
    print("[Test 1] Importing Polymetis...")
    try:
        from polymetis import RobotInterface, GripperInterface
        print("  ✓ Polymetis imported successfully")
        return True
    except Exception as e:
        print(f"  ✗ Failed: {e}")
        return False


def test_ros2():
    """Test ROS2 import."""
    print("[Test 2] Importing ROS2...")
    try:
        import rclpy
        from rclpy.node import Node
        print("  ✓ rclpy imported successfully")
        return True
    except Exception as e:
        print(f"  ✗ Failed: {e}")
        return False


def test_combined():
    """Test combined import (critical test for spdlog conflict)."""
    print("[Test 3] Combined import test...")
    try:
        from polymetis import RobotInterface
        import rclpy
        rclpy.init()
        rclpy.shutdown()
        print("  ✓ Both Polymetis and ROS2 work together!")
        return True
    except Exception as e:
        print(f"  ✗ Failed: {e}")
        return False


def test_bridge():
    """Test polymetis_bridge import."""
    print("[Test 4] Importing polymetis_bridge...")
    try:
        sys.path.insert(0, '/app/ros2_ws/src/role-ros2/scripts')
        from polymetis_bridge import PolymetisCombinedNode, MockRobotInterface
        print("  ✓ polymetis_bridge imported successfully")
        return True
    except Exception as e:
        print(f"  ✗ Failed: {e}")
        return False


def main():
    print("=" * 60)
    print("Testing ROS2 and Polymetis Integration")
    print("=" * 60)
    
    results = [
        test_polymetis(),
        test_ros2(),
        test_combined(),
        test_bridge(),
    ]
    
    print("\n" + "=" * 60)
    passed = sum(results)
    total = len(results)
    
    if passed == total:
        print(f"✓ All {total} tests passed!")
        return 0
    else:
        print(f"✗ {total - passed}/{total} tests failed")
        return 1


if __name__ == '__main__':
    sys.exit(main())

