#!/usr/bin/env python3
"""
Test script for FrankaRobot interface.

This script tests all methods of the FrankaRobot class to ensure proper
communication with polymetis_bridge_node.

Usage:
    python3 test_robot.py [--mock] [--skip-reset] [--skip-motion]
"""

import sys
import time
import argparse
import numpy as np
from pathlib import Path

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from role_ros2.franka.robot import FrankaRobot


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


def test_state_getters(robot, results):
    """Test state getter methods."""
    print("\n--- Testing State Getters ---")
    
    # Test get_joint_positions
    try:
        joint_pos = robot.get_joint_positions()
        assert len(joint_pos) == 7, f"Expected 7 joints, got {len(joint_pos)}"
        assert all(isinstance(x, (int, float)) for x in joint_pos), "Joint positions must be numbers"
        results.add_pass("get_joint_positions")
    except Exception as e:
        results.add_fail("get_joint_positions", str(e))
    
    # Test get_joint_velocities
    try:
        joint_vel = robot.get_joint_velocities()
        assert len(joint_vel) == 7, f"Expected 7 joints, got {len(joint_vel)}"
        assert all(isinstance(x, (int, float)) for x in joint_vel), "Joint velocities must be numbers"
        results.add_pass("get_joint_velocities")
    except Exception as e:
        results.add_fail("get_joint_velocities", str(e))
    
    # Test get_gripper_position
    try:
        gripper_pos = robot.get_gripper_position()
        assert 0.0 <= gripper_pos <= 1.0, f"Gripper position should be [0, 1], got {gripper_pos}"
        results.add_pass("get_gripper_position")
    except Exception as e:
        results.add_fail("get_gripper_position", str(e))
    
    # Test get_gripper_state (alias)
    try:
        gripper_state = robot.get_gripper_state()
        assert 0.0 <= gripper_state <= 1.0, f"Gripper state should be [0, 1], got {gripper_state}"
        results.add_pass("get_gripper_state")
    except Exception as e:
        results.add_fail("get_gripper_state", str(e))
    
    # Test get_ee_pose
    try:
        ee_pose = robot.get_ee_pose()
        assert len(ee_pose) == 6, f"Expected 6 DOF pose, got {len(ee_pose)}"
        assert all(isinstance(x, (int, float)) for x in ee_pose), "EE pose must be numbers"
        results.add_pass("get_ee_pose")
    except Exception as e:
        results.add_fail("get_ee_pose", str(e))
    
    # Test get_robot_state
    try:
        state_dict, timestamp_dict = robot.get_robot_state()
        required_keys = [
            "cartesian_position", "gripper_position", "joint_positions",
            "joint_velocities", "joint_torques_computed"
        ]
        for key in required_keys:
            assert key in state_dict, f"Missing key in state_dict: {key}"
        assert "robot_timestamp_seconds" in timestamp_dict, "Missing timestamp in timestamp_dict"
        results.add_pass("get_robot_state")
    except Exception as e:
        results.add_fail("get_robot_state", str(e))


def test_reset_and_home(robot, results, skip_reset=False):
    """Test reset and home methods."""
    print("\n--- Testing Reset and Home ---")
    
    if skip_reset:
        results.add_skip("reset", "Skipped by user request")
        results.add_skip("home", "Skipped by user request")
        return
    
    # Test reset
    try:
        print("  Calling reset()...")
        robot.reset()
        time.sleep(2)  # Wait for reset to complete
        results.add_pass("reset")
    except Exception as e:
        results.add_fail("reset", str(e))
    
    # Test home (should call reset)
    try:
        print("  Calling home()...")
        robot.home()
        time.sleep(1)  # Wait for home to complete
        results.add_pass("home")
    except Exception as e:
        results.add_fail("home", str(e))


def test_gripper_control(robot, results, skip_motion=False):
    """Test gripper control methods."""
    print("\n--- Testing Gripper Control ---")
    
    if skip_motion:
        results.add_skip("update_gripper", "Skipped by user request")
        return
    
    # Test update_gripper (non-blocking)
    try:
        print("  Testing update_gripper (non-blocking)...")
        robot.update_gripper(0.5, velocity=False, blocking=False)  # Open to 50%
        time.sleep(1)
        pos1 = robot.get_gripper_position()
        
        robot.update_gripper(0.0, velocity=False, blocking=False)  # Close
        time.sleep(1)
        pos2 = robot.get_gripper_position()
        
        results.add_pass("update_gripper (non-blocking)")
    except Exception as e:
        results.add_fail("update_gripper (non-blocking)", str(e))
    
    # Test update_gripper (blocking)
    try:
        print("  Testing update_gripper (blocking)...")
        robot.update_gripper(0.8, velocity=False, blocking=True)  # Open to 80%
        time.sleep(0.5)
        results.add_pass("update_gripper (blocking)")
    except Exception as e:
        results.add_fail("update_gripper (blocking)", str(e))


def test_joint_control(robot, results, skip_motion=False):
    """Test joint control methods."""
    print("\n--- Testing Joint Control ---")
    
    if skip_motion:
        results.add_skip("update_joints (non-blocking)", "Skipped by user request")
        results.add_skip("update_joints (blocking)", "Skipped by user request")
        return
    
    # Get current joint positions
    try:
        current_joints = robot.get_joint_positions()
        
        # Test update_joints (non-blocking position)
        print("  Testing update_joints (non-blocking position)...")
        target_joints = np.array(current_joints) + np.array([0.1, -0.1, 0.1, -0.1, 0.1, -0.1, 0.1])
        robot.update_joints(target_joints.tolist(), velocity=False, blocking=False)
        time.sleep(1)
        results.add_pass("update_joints (non-blocking position)")
    except Exception as e:
        results.add_fail("update_joints (non-blocking position)", str(e))
    
    # Test update_joints (non-blocking velocity)
    try:
        print("  Testing update_joints (non-blocking velocity)...")
        robot.update_joints([0.0] * 7, velocity=True, blocking=False)
        time.sleep(0.5)
        results.add_pass("update_joints (non-blocking velocity)")
    except Exception as e:
        results.add_fail("update_joints (non-blocking velocity)", str(e))
    
    # Test update_joints (blocking) - only if not skipping motion
    try:
        print("  Testing update_joints (blocking)...")
        current_joints = robot.get_joint_positions()
        target_joints = np.array(current_joints) + np.array([0.05, -0.05, 0.05, -0.05, 0.05, -0.05, 0.05])
        robot.update_joints(target_joints.tolist(), velocity=False, blocking=True)
        results.add_pass("update_joints (blocking)")
    except Exception as e:
        results.add_fail("update_joints (blocking)", str(e))


def test_pose_control(robot, results, skip_motion=False):
    """Test pose control methods."""
    print("\n--- Testing Pose Control ---")
    
    if skip_motion:
        results.add_skip("update_pose (non-blocking)", "Skipped by user request")
        results.add_skip("update_pose (blocking)", "Skipped by user request")
        return
    
    # Get current pose
    try:
        current_pose = robot.get_ee_pose()
        
        # Test update_pose (non-blocking position)
        print("  Testing update_pose (non-blocking position)...")
        target_pose = np.array(current_pose) + np.array([0.01, 0.01, 0.01, 0.0, 0.0, 0.0])
        robot.update_pose(target_pose.tolist(), velocity=False, blocking=False)
        time.sleep(1)
        results.add_pass("update_pose (non-blocking position)")
    except Exception as e:
        results.add_fail("update_pose (non-blocking position)", str(e))
    
    # Test update_pose (non-blocking velocity)
    try:
        print("  Testing update_pose (non-blocking velocity)...")
        robot.update_pose([0.0] * 6, velocity=True, blocking=False)
        time.sleep(0.5)
        results.add_pass("update_pose (non-blocking velocity)")
    except Exception as e:
        results.add_fail("update_pose (non-blocking velocity)", str(e))
    
    # Test update_pose (blocking) - only if not skipping motion
    try:
        print("  Testing update_pose (blocking)...")
        current_pose = robot.get_ee_pose()
        target_pose = np.array(current_pose) + np.array([0.005, 0.005, 0.005, 0.0, 0.0, 0.0])
        robot.update_pose(target_pose.tolist(), velocity=False, blocking=True)
        results.add_pass("update_pose (blocking)")
    except Exception as e:
        results.add_fail("update_pose (blocking)", str(e))


def test_command_interface(robot, results, skip_motion=False):
    """Test update_command method."""
    print("\n--- Testing Command Interface ---")
    
    if skip_motion:
        results.add_skip("update_command", "Skipped by user request")
        return
    
    try:
        print("  Testing update_command (cartesian_velocity)...")
        # Test cartesian velocity command
        command = [0.0] * 6 + [0.0]  # 6 DOF cartesian + 1 gripper
        action_dict = robot.update_command(
            command,
            action_space="cartesian_velocity",
            gripper_action_space="velocity",
            blocking=False
        )
        assert "joint_position" in action_dict, "Action dict missing joint_position"
        assert "gripper_position" in action_dict, "Action dict missing gripper_position"
        time.sleep(0.5)
        results.add_pass("update_command (cartesian_velocity)")
    except Exception as e:
        results.add_fail("update_command (cartesian_velocity)", str(e))
    
    try:
        print("  Testing update_command (joint_position)...")
        current_joints = robot.get_joint_positions()
        command = current_joints + [0.5]  # 7 joints + 1 gripper
        action_dict = robot.update_command(
            command,
            action_space="joint_position",
            gripper_action_space="position",
            blocking=False
        )
        assert "joint_position" in action_dict, "Action dict missing joint_position"
        time.sleep(0.5)
        results.add_pass("update_command (joint_position)")
    except Exception as e:
        results.add_fail("update_command (joint_position)", str(e))


def test_controller_management(robot, results):
    """Test controller management methods."""
    print("\n--- Testing Controller Management ---")
    
    # Test start_joint_impedance
    try:
        print("  Testing start_joint_impedance...")
        success = robot.start_joint_impedance()
        # Note: May fail if service not available, but should not raise exception
        results.add_pass("start_joint_impedance")
    except Exception as e:
        results.add_fail("start_joint_impedance", str(e))
    
    # Test start_cartesian_impedance
    try:
        print("  Testing start_cartesian_impedance...")
        success = robot.start_cartesian_impedance()
        # Note: May fail if service not available, but should not raise exception
        results.add_pass("start_cartesian_impedance")
    except Exception as e:
        results.add_fail("start_cartesian_impedance", str(e))
    
    # Test terminate_current_policy
    try:
        print("  Testing terminate_current_policy...")
        success = robot.terminate_current_policy()
        # Note: May fail if service not available, but should not raise exception
        results.add_pass("terminate_current_policy")
    except Exception as e:
        results.add_fail("terminate_current_policy", str(e))


def test_kinematics(robot, results):
    """Test kinematics computation methods."""
    print("\n--- Testing Kinematics ---")
    
    # Test solve_inverse_kinematics
    try:
        print("  Testing solve_inverse_kinematics...")
        position = [0.5, 0.0, 0.5]  # [x, y, z]
        orientation = [0.0, 0.0, 0.0]  # [roll, pitch, yaw] in euler
        joint_pos, success = robot.solve_inverse_kinematics(position, orientation)
        if success and joint_pos is not None:
            assert len(joint_pos) == 7, f"Expected 7 joints, got {len(joint_pos)}"
        results.add_pass("solve_inverse_kinematics")
    except Exception as e:
        results.add_fail("solve_inverse_kinematics", str(e))
    
    # Test compute_forward_kinematics
    try:
        print("  Testing compute_forward_kinematics...")
        joint_positions = robot.get_joint_positions()
        position, orientation = robot.compute_forward_kinematics(joint_positions)
        if position is not None and orientation is not None:
            assert len(position) == 3, f"Expected 3D position, got {len(position)}"
            assert len(orientation) == 4, f"Expected quaternion, got {len(orientation)}"
        results.add_pass("compute_forward_kinematics")
    except Exception as e:
        results.add_fail("compute_forward_kinematics", str(e))
    
    # Test compute_time_to_go
    try:
        print("  Testing compute_time_to_go...")
        current_joints = robot.get_joint_positions()
        target_joints = np.array(current_joints) + np.array([0.1] * 7)
        time_to_go = robot.compute_time_to_go(target_joints.tolist())
        assert isinstance(time_to_go, (int, float)) and time_to_go > 0, f"Invalid time_to_go: {time_to_go}"
        results.add_pass("compute_time_to_go")
    except Exception as e:
        results.add_fail("compute_time_to_go", str(e))


def test_action_dict(robot, results):
    """Test create_action_dict method."""
    print("\n--- Testing Action Dictionary ---")
    
    try:
        print("  Testing create_action_dict (cartesian_velocity)...")
        action = [0.0] * 6 + [0.0]  # 6 DOF cartesian + 1 gripper
        action_dict = robot.create_action_dict(
            action,
            action_space="cartesian_velocity",
            gripper_action_space="velocity"
        )
        required_keys = ["joint_position", "gripper_position", "robot_state"]
        for key in required_keys:
            assert key in action_dict, f"Missing key in action_dict: {key}"
        results.add_pass("create_action_dict (cartesian_velocity)")
    except Exception as e:
        results.add_fail("create_action_dict (cartesian_velocity)", str(e))
    
    try:
        print("  Testing create_action_dict (joint_position)...")
        current_joints = robot.get_joint_positions()
        action = current_joints + [0.5]  # 7 joints + 1 gripper
        action_dict = robot.create_action_dict(
            action,
            action_space="joint_position",
            gripper_action_space="position"
        )
        required_keys = ["joint_position", "gripper_position", "robot_state"]
        for key in required_keys:
            assert key in action_dict, f"Missing key in action_dict: {key}"
        results.add_pass("create_action_dict (joint_position)")
    except Exception as e:
        results.add_fail("create_action_dict (joint_position)", str(e))


def main():
    parser = argparse.ArgumentParser(description="Test FrankaRobot interface")
    parser.add_argument('--mock', action='store_true', help='Use mock mode (no real robot)')
    parser.add_argument('--skip-reset', action='store_true', help='Skip reset/home tests')
    parser.add_argument('--skip-motion', action='store_true', help='Skip motion control tests')
    parser.add_argument('--timeout', type=float, default=10.0, help='Timeout for service calls (seconds)')
    args = parser.parse_args()
    
    print("="*60)
    print("FrankaRobot Interface Test")
    print("="*60)
    print(f"Mock mode: {args.mock}")
    print(f"Skip reset: {args.skip_reset}")
    print(f"Skip motion: {args.skip_motion}")
    print("="*60)
    
    results = TestResult()
    
    # Initialize robot
    print("\nInitializing FrankaRobot...")
    try:
        robot = FrankaRobot()
        print("✅ FrankaRobot initialized successfully")
        time.sleep(2)  # Wait for state to be received
    except Exception as e:
        print(f"❌ Failed to initialize FrankaRobot: {e}")
        print("\nMake sure polymetis_bridge_node is running:")
        print("  ros2 run role_ros2 polymetis_bridge_node")
        sys.exit(1)
    
    # Run tests
    try:
        test_state_getters(robot, results)
        test_reset_and_home(robot, results, skip_reset=args.skip_reset)
        test_gripper_control(robot, results, skip_motion=args.skip_motion)
        test_joint_control(robot, results, skip_motion=args.skip_motion)
        test_pose_control(robot, results, skip_motion=args.skip_motion)
        test_command_interface(robot, results, skip_motion=args.skip_motion)
        test_controller_management(robot, results)
        test_kinematics(robot, results)
        test_action_dict(robot, results)
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
    except Exception as e:
        print(f"\n\nUnexpected error during testing: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # Cleanup
        try:
            robot.shutdown()
        except:
            pass
    
    # Print summary
    results.print_summary()
    
    # Exit with appropriate code
    sys.exit(0 if results.failed == 0 else 1)


if __name__ == '__main__':
    main()

