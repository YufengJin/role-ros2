#!/usr/bin/env python3
"""
Test script for teleoperation_node.

This script tests the teleoperation functionality using RobotEnv and ROS2VRPolicy.

Usage:
    python3 test_teleoperation_node.py [--skip-motion] [--mock-oculus]
    
    --skip-motion: Skip actual robot movement tests
    --mock-oculus: Use mock Oculus controller (publish fake pose/button messages)
"""

import sys
import time
import argparse
import numpy as np
import threading
from pathlib import Path

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from std_msgs.msg import Header
from role_ros2.msg import OculusButtons

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from role_ros2.robot_env import RobotEnv
from role_ros2.scripts.ros2_vr_policy import ROS2VRPolicy


class MockOculusPublisher(Node):
    """Mock Oculus controller publisher for testing."""
    
    def __init__(self):
        super().__init__('mock_oculus_publisher')
        
        self.right_pose_pub = self.create_publisher(
            PoseStamped, 'oculus/right_controller/pose', 10
        )
        self.buttons_pub = self.create_publisher(
            OculusButtons, 'oculus/buttons', 10
        )
        
        self.timer = self.create_timer(0.02, self.publish_mock_data)  # 50 Hz
        
        self.get_logger().info("Mock Oculus publisher started")
    
    def publish_mock_data(self):
        """Publish mock controller data."""
        # Mock right controller pose (slight movement)
        right_pose = PoseStamped()
        right_pose.header = Header()
        right_pose.header.stamp = self.get_clock().now().to_msg()
        right_pose.header.frame_id = 'oculus_base'
        right_pose.pose.position = Point(x=0.3, y=0.0, z=0.1)
        right_pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        self.right_pose_pub.publish(right_pose)
        
        # Mock buttons (grip pressed, trigger at 0.5)
        buttons = OculusButtons()
        buttons.header = Header()
        buttons.header.stamp = self.get_clock().now().to_msg()
        buttons.right_grip_pressed = True
        buttons.right_trigger_value = 0.5
        buttons.a = False
        buttons.b = False
        self.buttons_pub.publish(buttons)


def test_ros2_vr_policy(node, results, skip_motion=False):
    """Test ROS2VRPolicy."""
    print("\n--- Testing ROS2VRPolicy ---")
    
    try:
        print("  Initializing ROS2VRPolicy...")
        policy = ROS2VRPolicy(
            node=node,
            right_controller=True,
            max_lin_vel=0.5,
            max_rot_vel=0.5,
            max_gripper_vel=0.5,
        )
        print("  ✓ ROS2VRPolicy initialized")
        
        # Wait for messages
        print("  Waiting for Oculus messages...")
        time.sleep(2)
        
        # Test get_info
        info = policy.get_info()
        print(f"  Controller info: {info}")
        assert "controller_on" in info
        assert "movement_enabled" in info
        print("  ✓ get_info() works")
        
        # Test forward (if not skipping motion)
        if not skip_motion:
            print("  Testing forward() with mock state...")
            mock_state = {
                "cartesian_position": [0.5, 0.0, 0.5, 0.0, 0.0, 0.0],
                "gripper_position": 0.5,
            }
            obs_dict = {"robot_state": mock_state}
            
            action = policy.forward(obs_dict)
            assert len(action) == 7, f"Expected 7D action, got {len(action)}"
            assert all(isinstance(x, (int, float, np.floating)) for x in action)
            print(f"  ✓ forward() returns action: {action[:3]}...")
        
        results.add_pass("ROS2VRPolicy")
    except Exception as e:
        results.add_fail("ROS2VRPolicy", str(e))
        import traceback
        traceback.print_exc()


def test_teleoperation_integration(node, results, skip_motion=False, mock_oculus=False):
    """Test teleoperation integration."""
    print("\n--- Testing Teleoperation Integration ---")
    
    if skip_motion:
        print("  ⏭️  SKIPPED (use without --skip-motion to test)")
        results.add_skip("Teleoperation Integration", "Skipped by user request")
        return
    
    try:
        # Start mock publisher if requested
        mock_pub = None
        if mock_oculus:
            print("  Starting mock Oculus publisher...")
            mock_pub = MockOculusPublisher()
            time.sleep(1)
        
        print("  Initializing RobotEnv...")
        env = RobotEnv(
            action_space="cartesian_velocity",
            gripper_action_space="velocity",
            do_reset=False,  # Don't reset for testing
            node=node
        )
        print("  ✓ RobotEnv initialized")
        
        print("  Initializing ROS2VRPolicy...")
        controller = ROS2VRPolicy(
            node=node,
            right_controller=True,
            max_lin_vel=0.1,  # Small velocities for testing
            max_rot_vel=0.1,
            max_gripper_vel=0.1,
        )
        print("  ✓ ROS2VRPolicy initialized")
        
        # Wait for controller to receive messages
        print("  Waiting for controller messages...")
        time.sleep(2)
        
        # Test control loop
        print("  Testing control loop (5 iterations)...")
        for i in range(5):
            state_dict, _ = env.get_state()
            obs_dict = {"robot_state": state_dict}
            action = controller.forward(obs_dict)
            env.step(action)
            time.sleep(0.1)
        
        print("  ✓ Control loop executed successfully")
        
        # Check controller info
        info = controller.get_info()
        print(f"  Controller info: {info}")
        
        results.add_pass("Teleoperation Integration")
    except Exception as e:
        results.add_fail("Teleoperation Integration", str(e))
        import traceback
        traceback.print_exc()


def test_robot_env_with_shared_node(node, results):
    """Test RobotEnv with shared node."""
    print("\n--- Testing RobotEnv with Shared Node ---")
    
    try:
        print("  Initializing RobotEnv with shared node...")
        env = RobotEnv(
            action_space="cartesian_velocity",
            do_reset=False,
            node=node
        )
        print("  ✓ RobotEnv initialized")
        
        # Test get_state
        state_dict, timestamp_dict = env.get_state()
        assert "cartesian_position" in state_dict
        assert "gripper_position" in state_dict
        print("  ✓ get_state() works")
        
        # Test step (small action)
        print("  Testing step() with zero action...")
        action = np.zeros(7)
        env.step(action)
        print("  ✓ step() works")
        
        results.add_pass("RobotEnv with Shared Node")
    except Exception as e:
        results.add_fail("RobotEnv with Shared Node", str(e))
        import traceback
        traceback.print_exc()


class TestResult:
    """Helper class to track test results."""
    def __init__(self):
        self.passed = 0
        self.failed = 0
        self.skipped = 0
        self.errors = []
    
    def add_pass(self, test_name):
        self.passed += 1
        print(f"✅ PASS: {test_name}")
    
    def add_fail(self, test_name, error):
        self.failed += 1
        self.errors.append((test_name, error))
        print(f"❌ FAIL: {test_name} - {error}")
    
    def add_skip(self, test_name, reason):
        self.skipped += 1
        print(f"⏭️  SKIP: {test_name} - {reason}")
    
    def print_summary(self):
        print("\n" + "="*60)
        print("TEST SUMMARY")
        print("="*60)
        print(f"Passed:  {self.passed}")
        print(f"Failed:  {self.failed}")
        print(f"Skipped: {self.skipped}")
        print(f"Total:   {self.passed + self.failed + self.skipped}")
        
        if self.errors:
            print("\nFAILED TESTS:")
            for test_name, error in self.errors:
                print(f"  - {test_name}: {error}")
        
        print("="*60)


def main():
    parser = argparse.ArgumentParser(description="Test teleoperation_node")
    parser.add_argument('--skip-motion', action='store_true',
                       help='Skip motion control tests')
    parser.add_argument('--mock-oculus', action='store_true',
                       help='Use mock Oculus controller publisher')
    args = parser.parse_args()
    
    print("="*60)
    print("Teleoperation Node Test")
    print("="*60)
    print(f"Skip motion: {args.skip_motion}")
    print(f"Mock Oculus: {args.mock_oculus}")
    print("="*60)
    
    # Initialize ROS2
    rclpy.init()
    node = Node('teleoperation_test_node')
    
    # Start spin thread for callbacks
    _spin_active = threading.Event()
    _spin_active.set()
    
    def spin_node_thread():
        """Background thread to spin node for callbacks."""
        while rclpy.ok() and _spin_active.is_set():
            try:
                rclpy.spin_once(node, timeout_sec=0.1)
            except Exception as e:
                if "generator already executing" not in str(e):
                    pass
            time.sleep(0.01)
    
    spin_thread = threading.Thread(target=spin_node_thread, daemon=True)
    spin_thread.start()
    time.sleep(0.2)
    
    results = TestResult()
    
    try:
        # Run tests
        test_robot_env_with_shared_node(node, results)
        time.sleep(1)
        
        test_ros2_vr_policy(node, results, skip_motion=args.skip_motion)
        time.sleep(1)
        
        test_teleoperation_integration(
            node, results,
            skip_motion=args.skip_motion,
            mock_oculus=args.mock_oculus
        )
    
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
    except Exception as e:
        print(f"\n\nUnexpected error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        _spin_active.clear()
        time.sleep(0.1)
        node.destroy_node()
        rclpy.shutdown()
    
    # Print summary
    results.print_summary()
    
    # Exit with appropriate code
    sys.exit(0 if results.failed == 0 else 1)


if __name__ == '__main__':
    main()

