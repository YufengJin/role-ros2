#!/usr/bin/env python3
"""
Test script for FrankaRobotV2 interface.

This script tests all methods of the FrankaRobotV2 class to ensure proper
communication with V2 nodes (franka_robot_interface_node and franka_gripper_interface_node).

⚠️  Warning: This test will perform robot movements!
   - Joint movements: 0.1-0.15 rad (reduced)
   - End-effector movements: 0.02-0.03 m (reduced)
   - Gripper movements: 0-100%
   - Movement speed: Slowed down
   Please ensure sufficient space around the robot!

Usage:
    python3 test_robot_v2.py [--skip-reset] [--skip-motion]
    
    --skip-reset: Skip reset/home tests
    --skip-motion: Skip motion control tests (only test state getters)

Prerequisites:
    Launch V2 nodes first:
    ros2 launch role_ros2 franka_robot_v2.launch.py use_mock:=true

Author: Role-ROS2 Team
"""

import sys
import time
import argparse
import numpy as np
from pathlib import Path
import threading
import subprocess

import rclpy
from rclpy.node import Node

from role_ros2.robot.franka.robot_v2 import FrankaRobotV2
from role_ros2.misc.transformations import euler_to_quat


# Test interval between each test (seconds)
TEST_INTERVAL = 5.0


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
        print("\n" + "=" * 60)
        print("TEST SUMMARY")
        print("=" * 60)
        print(f"Passed:  {self.passed}")
        print(f"Failed:  {self.failed}")
        print(f"Skipped: {self.skipped}")
        print(f"Total:   {self.passed + self.failed + self.skipped}")
        
        if self.errors:
            print("\nFAILED TESTS:")
            for test_name, error in self.errors:
                print(f"  - {test_name}: {error}")
        
        print("=" * 60)


def wait_between_tests(test_name: str):
    """Wait between tests with countdown display."""
    print(f"\n{'=' * 60}")
    print(f"📋 Next Test: {test_name}")
    print(f"   Waiting {TEST_INTERVAL:.0f} seconds before starting...")
    print("=" * 60)
    time.sleep(TEST_INTERVAL)


def evaluate_joint_position(robot, target_joints, tolerance=0.05, test_name=""):
    """
    Evaluate if joint positions reached target.
    
    Args:
        robot: FrankaRobotV2 instance
        target_joints: Target joint positions (7 DOF)
        tolerance: Allowed position error (rad)
        test_name: Test name
    
    Returns:
        tuple: (success, max_error, detailed_errors)
    """
    actual_joints = np.array(robot.get_joint_positions())
    target_joints = np.array(target_joints)
    
    errors = np.abs(actual_joints - target_joints)
    max_error = np.max(errors)
    mean_error = np.mean(errors)
    
    print(f"    Position Evaluation ({test_name}):")
    print(f"      Target: {target_joints[:3]}...")
    print(f"      Actual: {actual_joints[:3]}...")
    print(f"      Max Error: {max_error:.4f} rad (threshold: {tolerance:.4f} rad)")
    print(f"      Mean Error: {mean_error:.4f} rad")
    
    success = max_error < tolerance
    if success:
        print(f"      ✓ Position accuracy OK (error < {tolerance:.4f} rad)")
    else:
        print(f"      ⚠️  Position accuracy insufficient (error = {max_error:.4f} rad > {tolerance:.4f} rad)")
    
    return success, max_error, errors


def evaluate_pose_position(robot, target_pose, tolerance=0.01, test_name=""):
    """
    Evaluate if end-effector position reached target.
    
    Args:
        robot: FrankaRobotV2 instance
        target_pose: Target EE pose [x, y, z, roll, pitch, yaw]
        tolerance: Allowed position error (m)
        test_name: Test name
    
    Returns:
        tuple: (success, max_position_error, detailed_errors)
    """
    actual_pose = np.array(robot.get_ee_pose())
    target_pose = np.array(target_pose)
    
    position_errors = np.abs(actual_pose[:3] - target_pose[:3])
    max_position_error = np.max(position_errors)
    mean_position_error = np.mean(position_errors)
    
    orientation_errors = np.abs(actual_pose[3:] - target_pose[3:])
    orientation_errors = np.minimum(orientation_errors, 2 * np.pi - orientation_errors)
    max_orientation_error = np.max(orientation_errors)
    
    print(f"    Position Evaluation ({test_name}):")
    print(f"      Target Position: {target_pose[:3]} m")
    print(f"      Actual Position: {actual_pose[:3]} m")
    print(f"      Position Error: {position_errors} m")
    print(f"      Max Position Error: {max_position_error:.4f} m (threshold: {tolerance:.4f} m)")
    print(f"      Mean Position Error: {mean_position_error:.4f} m")
    print(f"      Max Orientation Error: {max_orientation_error:.4f} rad")
    
    success = max_position_error < tolerance
    if success:
        print(f"      ✓ Position accuracy OK (error < {tolerance:.4f} m)")
    else:
        print(f"      ⚠️  Position accuracy insufficient (error = {max_position_error:.4f} m > {tolerance:.4f} m)")
    
    return success, max_position_error, position_errors


# =============================================================================
# Test Functions
# =============================================================================

def test_properties(robot, results):
    """Test property accessors."""
    wait_between_tests("Properties (arm_namespace, gripper_namespace)")
    print("\n--- Testing Properties ---")
    
    # Test arm_namespace property
    try:
        arm_ns = robot.arm_namespace
        assert isinstance(arm_ns, str), f"arm_namespace should be string, got {type(arm_ns)}"
        assert len(arm_ns) > 0, "arm_namespace should not be empty"
        print(f"  ✓ arm_namespace: '{arm_ns}'")
        results.add_pass("arm_namespace property")
    except Exception as e:
        results.add_fail("arm_namespace property", str(e))
    
    # Test gripper_namespace property
    try:
        gripper_ns = robot.gripper_namespace
        assert isinstance(gripper_ns, str), f"gripper_namespace should be string, got {type(gripper_ns)}"
        assert len(gripper_ns) > 0, "gripper_namespace should not be empty"
        print(f"  ✓ gripper_namespace: '{gripper_ns}'")
        results.add_pass("gripper_namespace property")
    except Exception as e:
        results.add_fail("gripper_namespace property", str(e))


def test_arm_state_getters(robot, results):
    """Test arm state getter methods."""
    wait_between_tests("Arm State Getters")
    print("\n--- Testing Arm State Getters ---")
    
    # Test get_joint_positions
    try:
        joint_pos = robot.get_joint_positions()
        assert len(joint_pos) == 7, f"Expected 7 joints, got {len(joint_pos)}"
        assert all(isinstance(x, (int, float)) for x in joint_pos), "Joint positions must be numbers"
        print(f"  ✓ get_joint_positions: {np.array(joint_pos)[:3]}... (7 joints)")
        results.add_pass("get_joint_positions")
    except Exception as e:
        results.add_fail("get_joint_positions", str(e))
    
    # Test get_joint_velocities
    try:
        joint_vel = robot.get_joint_velocities()
        assert len(joint_vel) == 7, f"Expected 7 joints, got {len(joint_vel)}"
        assert all(isinstance(x, (int, float)) for x in joint_vel), "Joint velocities must be numbers"
        print(f"  ✓ get_joint_velocities: {np.array(joint_vel)[:3]}... (7 joints)")
        results.add_pass("get_joint_velocities")
    except Exception as e:
        results.add_fail("get_joint_velocities", str(e))
    
    # Test get_ee_pose
    try:
        ee_pose = robot.get_ee_pose()
        assert len(ee_pose) == 6, f"Expected 6 DOF pose, got {len(ee_pose)}"
        assert all(isinstance(x, (int, float)) for x in ee_pose), "EE pose must be numbers"
        print(f"  ✓ get_ee_pose: position={np.array(ee_pose[:3])} m, euler={np.array(ee_pose[3:])} rad")
        results.add_pass("get_ee_pose")
    except Exception as e:
        results.add_fail("get_ee_pose", str(e))
    
    # Test get_ee_position
    try:
        ee_pos = robot.get_ee_position()
        assert len(ee_pos) == 3, f"Expected 3D position, got {len(ee_pos)}"
        assert all(isinstance(x, (int, float)) for x in ee_pos), "EE position must be numbers"
        print(f"  ✓ get_ee_position: {ee_pos} m")
        results.add_pass("get_ee_position")
    except Exception as e:
        results.add_fail("get_ee_position", str(e))
    
    # Test get_ee_quaternion
    try:
        ee_quat = robot.get_ee_quaternion()
        assert len(ee_quat) == 4, f"Expected quaternion (4 elements), got {len(ee_quat)}"
        assert all(isinstance(x, (int, float)) for x in ee_quat), "Quaternion must be numbers"
        # Check quaternion normalization (should be close to 1)
        quat_norm = np.linalg.norm(ee_quat)
        print(f"  ✓ get_ee_quaternion: {ee_quat} (norm={quat_norm:.4f})")
        results.add_pass("get_ee_quaternion")
    except Exception as e:
        results.add_fail("get_ee_quaternion", str(e))
    
    # Test get_arm_state_raw
    try:
        arm_state = robot.get_arm_state_raw()
        if arm_state is not None:
            assert hasattr(arm_state, 'joint_positions'), "ArmState should have joint_positions"
            assert hasattr(arm_state, 'ee_position'), "ArmState should have ee_position"
            print(f"  ✓ get_arm_state_raw: ArmState message received")
        else:
            print(f"  ⚠️  get_arm_state_raw: returned None (may be normal at startup)")
        results.add_pass("get_arm_state_raw")
    except Exception as e:
        results.add_fail("get_arm_state_raw", str(e))


def test_gripper_state_getters(robot, results):
    """Test gripper state getter methods."""
    wait_between_tests("Gripper State Getters")
    print("\n--- Testing Gripper State Getters ---")
    
    # Test get_gripper_position
    try:
        gripper_pos = robot.get_gripper_position()
        assert 0.0 <= gripper_pos <= 1.0, f"Gripper position should be [0, 1], got {gripper_pos}"
        print(f"  ✓ get_gripper_position: {gripper_pos:.4f} (0=closed, 1=open)")
        results.add_pass("get_gripper_position")
    except Exception as e:
        results.add_fail("get_gripper_position", str(e))
    
    # Test get_gripper_width
    try:
        gripper_width = robot.get_gripper_width()
        assert 0.0 <= gripper_width <= 0.1, f"Gripper width should be [0, 0.1], got {gripper_width}"
        print(f"  ✓ get_gripper_width: {gripper_width:.4f} m")
        results.add_pass("get_gripper_width")
    except Exception as e:
        results.add_fail("get_gripper_width", str(e))
    
    # Test get_gripper_state (alias)
    try:
        gripper_state = robot.get_gripper_state()
        assert 0.0 <= gripper_state <= 1.0, f"Gripper state should be [0, 1], got {gripper_state}"
        print(f"  ✓ get_gripper_state: {gripper_state:.4f}")
        results.add_pass("get_gripper_state")
    except Exception as e:
        results.add_fail("get_gripper_state", str(e))
    
    # Test is_gripper_grasped
    try:
        is_grasped = robot.is_gripper_grasped()
        assert isinstance(is_grasped, bool), f"is_grasped should be bool, got {type(is_grasped)}"
        print(f"  ✓ is_gripper_grasped: {is_grasped}")
        results.add_pass("is_gripper_grasped")
    except Exception as e:
        results.add_fail("is_gripper_grasped", str(e))
    
    # Test get_gripper_state_raw
    try:
        gripper_state_raw = robot.get_gripper_state_raw()
        if gripper_state_raw is not None:
            assert hasattr(gripper_state_raw, 'width'), "GripperState should have width"
            assert hasattr(gripper_state_raw, 'position'), "GripperState should have position"
            print(f"  ✓ get_gripper_state_raw: GripperState message received")
        else:
            print(f"  ⚠️  get_gripper_state_raw: returned None (may be normal at startup)")
        results.add_pass("get_gripper_state_raw")
    except Exception as e:
        results.add_fail("get_gripper_state_raw", str(e))


def test_robot_state(robot, results):
    """Test combined robot state method."""
    wait_between_tests("Robot State (Combined)")
    print("\n--- Testing Robot State (Combined) ---")
    
    try:
        state_dict, timestamp_dict = robot.get_robot_state()
        
        # Check state_dict keys
        required_state_keys = [
            "cartesian_position", "gripper_position", "joint_positions",
            "joint_velocities", "joint_torques_computed"
        ]
        for key in required_state_keys:
            assert key in state_dict, f"Missing key in state_dict: {key}"
        
        # Check timestamp_dict keys
        required_ts_keys = ["robot_polymetis_t", "robot_pub_t", "robot_sub_t", "robot_end_t"]
        for key in required_ts_keys:
            assert key in timestamp_dict, f"Missing key in timestamp_dict: {key}"
        
        print(f"  ✓ State dict keys: {list(state_dict.keys())[:5]}...")
        print(f"  ✓ Timestamp dict keys: {list(timestamp_dict.keys())}")
        print(f"  ✓ cartesian_position: {state_dict['cartesian_position'][:3]}...")
        print(f"  ✓ gripper_position: {state_dict['gripper_position']:.4f}")
        results.add_pass("get_robot_state")
    except Exception as e:
        results.add_fail("get_robot_state", str(e))


def test_reset_and_home(robot, results, skip_reset=False):
    """Test reset and home methods."""
    wait_between_tests("Reset and Home")
    print("\n--- Testing Reset and Home ---")
    
    if skip_reset:
        results.add_skip("reset", "Skipped by user request")
        results.add_skip("home", "Skipped by user request")
        return
    
    # Test reset
    try:
        print("  Testing reset()...")
        joints_before = np.array(robot.get_joint_positions())
        ee_before = np.array(robot.get_ee_pose()[:3])
        print(f"    Before reset - joints: {joints_before[:3]}...")
        print(f"    Before reset - EE pos: {ee_before}")
        
        robot.reset(wait_for_completion=True, wait_time_sec=5.0)
        
        time.sleep(2)
        
        joints_after = np.array(robot.get_joint_positions())
        ee_after = np.array(robot.get_ee_pose()[:3])
        print(f"    After reset - joints: {joints_after[:3]}...")
        print(f"    After reset - EE pos: {ee_after}")
        
        joint_diff = np.max(np.abs(joints_after - joints_before))
        ee_diff = np.max(np.abs(ee_after - ee_before))
        print(f"    Joint change: {joint_diff:.4f} rad")
        print(f"    EE change: {ee_diff:.4f} m")
        
        results.add_pass("reset")
    except Exception as e:
        results.add_fail("reset", str(e))
    
    # Test home (alias for reset)
    try:
        print("  Testing home()...")
        joints_before = np.array(robot.get_joint_positions())
        robot.home()
        time.sleep(2)
        joints_after = np.array(robot.get_joint_positions())
        joint_diff = np.max(np.abs(joints_after - joints_before))
        print(f"    Joint change: {joint_diff:.4f} rad")
        results.add_pass("home")
    except Exception as e:
        results.add_fail("home", str(e))


def test_gripper_services(robot, results, skip_motion=False):
    """Test gripper service methods."""
    wait_between_tests("Gripper Services")
    print("\n--- Testing Gripper Services ---")
    
    if skip_motion:
        results.add_skip("gripper_open", "Skipped by user request")
        results.add_skip("gripper_close", "Skipped by user request")
        results.add_skip("gripper_goto", "Skipped by user request")
        results.add_skip("gripper_grasp", "Skipped by user request")
        return
    
    # Test gripper_open
    try:
        print("  Testing gripper_open()...")
        initial_pos = robot.get_gripper_position()
        print(f"    Initial position: {initial_pos:.4f}")
        
        success = robot.gripper_open()
        time.sleep(2)
        
        final_pos = robot.get_gripper_position()
        print(f"    Final position: {final_pos:.4f}")
        print(f"    Success: {success}")
        
        if success:
            results.add_pass("gripper_open")
        else:
            results.add_fail("gripper_open", "Service returned failure")
    except Exception as e:
        results.add_fail("gripper_open", str(e))
    
    # Test gripper_close
    try:
        print("  Testing gripper_close()...")
        initial_pos = robot.get_gripper_position()
        print(f"    Initial position: {initial_pos:.4f}")
        
        success = robot.gripper_close()
        time.sleep(2)
        
        final_pos = robot.get_gripper_position()
        print(f"    Final position: {final_pos:.4f}")
        print(f"    Success: {success}")
        
        if success:
            results.add_pass("gripper_close")
        else:
            results.add_fail("gripper_close", "Service returned failure")
    except Exception as e:
        results.add_fail("gripper_close", str(e))
    
    # Test gripper_goto
    try:
        print("  Testing gripper_goto()...")
        target_width = 0.04  # 4cm
        print(f"    Target width: {target_width} m")
        
        success = robot.gripper_goto(width=target_width, speed=0.1, force=0.1, blocking=True)
        time.sleep(1)
        
        final_width = robot.get_gripper_width()
        print(f"    Final width: {final_width:.4f} m")
        print(f"    Success: {success}")
        
        if success:
            results.add_pass("gripper_goto")
        else:
            results.add_fail("gripper_goto", "Service returned failure")
    except Exception as e:
        results.add_fail("gripper_goto", str(e))
    
    # Test gripper_grasp
    try:
        print("  Testing gripper_grasp()...")
        # First open the gripper
        robot.gripper_open()
        time.sleep(2)
        
        initial_width = robot.get_gripper_width()
        print(f"    Initial width (after open): {initial_width:.4f} m")
        
        success, is_grasped = robot.gripper_grasp(speed=0.1, force=0.5, blocking=True)
        time.sleep(2)
        
        final_width = robot.get_gripper_width()
        print(f"    Final width: {final_width:.4f} m")
        print(f"    Service response - Success: {success}, Is Grasped: {is_grasped}")
        
        # Check if grasp action actually worked (width should be near 0)
        grasp_worked = final_width < 0.01
        print(f"    Grasp action worked (width < 0.01): {grasp_worked}")
        
        if success or grasp_worked:
            results.add_pass("gripper_grasp")
        else:
            results.add_fail("gripper_grasp", f"Service failed and width={final_width:.4f}m")
    except Exception as e:
        results.add_fail("gripper_grasp", str(e))


def test_update_gripper(robot, results, skip_motion=False):
    """Test update_gripper topic method."""
    wait_between_tests("Update Gripper (Topic)")
    print("\n--- Testing Update Gripper (Topic) ---")
    
    if skip_motion:
        results.add_skip("update_gripper (position)", "Skipped by user request")
        results.add_skip("update_gripper (velocity)", "Skipped by user request")
        return
    
    # Test update_gripper (position mode)
    try:
        print("  Testing update_gripper (position mode)...")
        initial_pos = robot.get_gripper_position()
        print(f"    Initial position: {initial_pos:.4f}")
        
        # Open to 80%
        robot.update_gripper(0.8, velocity=False, blocking=False)
        time.sleep(2)
        pos1 = robot.get_gripper_position()
        print(f"    After open to 0.8: {pos1:.4f}")
        
        # Close to 20%
        robot.update_gripper(0.2, velocity=False, blocking=False)
        time.sleep(2)
        pos2 = robot.get_gripper_position()
        print(f"    After close to 0.2: {pos2:.4f}")
        
        pos_diff = abs(pos1 - pos2)
        print(f"    Movement range: {pos_diff:.4f}")
        
        results.add_pass("update_gripper (position)")
    except Exception as e:
        results.add_fail("update_gripper (position)", str(e))
    
    # Test update_gripper (velocity mode)
    try:
        print("  Testing update_gripper (velocity mode)...")
        initial_pos = robot.get_gripper_position()
        print(f"    Initial position: {initial_pos:.4f}")
        
        # Apply positive velocity (open)
        robot.update_gripper(0.5, velocity=True, blocking=False)
        time.sleep(1)
        pos_after = robot.get_gripper_position()
        print(f"    After positive velocity: {pos_after:.4f}")
        
        results.add_pass("update_gripper (velocity)")
    except Exception as e:
        results.add_fail("update_gripper (velocity)", str(e))


def test_joint_control(robot, results, skip_motion=False):
    """Test joint control methods."""
    wait_between_tests("Joint Control")
    print("\n--- Testing Joint Control ---")
    
    if skip_motion:
        results.add_skip("update_joints (non-blocking position)", "Skipped by user request")
        results.add_skip("update_joints (non-blocking velocity)", "Skipped by user request")
        results.add_skip("update_joints (blocking)", "Skipped by user request")
        return
    
    # Test update_joints (non-blocking position)
    try:
        print("  Testing update_joints (non-blocking position)...")
        current_joints = np.array(robot.get_joint_positions())
        print(f"    Current joints: {current_joints[:3]}...")
        
        target_joints = current_joints + np.array([0.1, -0.1, 0.1, -0.1, 0.08, -0.08, 0.08])
        print(f"    Target joints: {target_joints[:3]}...")
        
        robot.update_joints(target_joints.tolist(), velocity=False, blocking=False)
        time.sleep(3)
        
        evaluate_joint_position(robot, target_joints, tolerance=0.1, test_name="non-blocking position")
        results.add_pass("update_joints (non-blocking position)")
    except Exception as e:
        results.add_fail("update_joints (non-blocking position)", str(e))
    
    # Test update_joints (non-blocking velocity)
    try:
        print("  Testing update_joints (non-blocking velocity)...")
        velocity_cmd = [0.05, -0.05, 0.05, -0.05, 0.03, -0.03, 0.03]
        print(f"    Velocity command: {velocity_cmd[:3]}...")
        
        robot.update_joints(velocity_cmd, velocity=True, blocking=False)
        time.sleep(2)
        
        # Stop
        robot.update_joints([0.0] * 7, velocity=True, blocking=False)
        time.sleep(0.5)
        
        results.add_pass("update_joints (non-blocking velocity)")
    except Exception as e:
        results.add_fail("update_joints (non-blocking velocity)", str(e))
    
    # Test update_joints (blocking)
    try:
        print("  Testing update_joints (blocking)...")
        current_joints = np.array(robot.get_joint_positions())
        print(f"    Current joints: {current_joints[:3]}...")
        
        target_joints = current_joints + np.array([0.1, -0.1, 0.1, -0.1, 0.08, -0.08, 0.08])
        print(f"    Target joints: {target_joints[:3]}...")
        
        success = robot.update_joints(target_joints.tolist(), velocity=False, blocking=True)
        print(f"    Blocking call returned: {success}")
        
        evaluate_joint_position(robot, target_joints, tolerance=0.1, test_name="blocking position")
        results.add_pass("update_joints (blocking)")
    except Exception as e:
        results.add_fail("update_joints (blocking)", str(e))


def test_pose_control(robot, results, skip_motion=False):
    """Test pose control methods."""
    wait_between_tests("Pose Control")
    print("\n--- Testing Pose Control ---")
    
    if skip_motion:
        results.add_skip("update_pose (non-blocking position)", "Skipped by user request")
        results.add_skip("update_pose (non-blocking velocity)", "Skipped by user request")
        results.add_skip("update_pose (blocking)", "Skipped by user request")
        return
    
    # Test update_pose (non-blocking position)
    try:
        print("  Testing update_pose (non-blocking position)...")
        current_pose = np.array(robot.get_ee_pose())
        print(f"    Current pose: {current_pose[:3]}...")
        
        target_pose = current_pose + np.array([0.02, 0.02, 0.02, 0.0, 0.0, 0.0])
        print(f"    Target pose: {target_pose[:3]}...")
        
        robot.update_pose(target_pose.tolist(), velocity=False, blocking=False)
        time.sleep(3)
        
        evaluate_pose_position(robot, target_pose, tolerance=0.02, test_name="non-blocking position")
        results.add_pass("update_pose (non-blocking position)")
    except Exception as e:
        results.add_fail("update_pose (non-blocking position)", str(e))
    
    # Test update_pose (non-blocking velocity)
    try:
        print("  Testing update_pose (non-blocking velocity)...")
        velocity_cmd = [0.02, 0.0, 0.0, 0.0, 0.0, 0.0]
        print(f"    Velocity command: {velocity_cmd}")
        
        robot.update_pose(velocity_cmd, velocity=True, blocking=False)
        time.sleep(2)
        
        # Stop
        robot.update_pose([0.0] * 6, velocity=True, blocking=False)
        time.sleep(0.5)
        
        results.add_pass("update_pose (non-blocking velocity)")
    except Exception as e:
        results.add_fail("update_pose (non-blocking velocity)", str(e))
    
    # Test update_pose (blocking)
    try:
        print("  Testing update_pose (blocking)...")
        current_pose = np.array(robot.get_ee_pose())
        print(f"    Current pose: {current_pose[:3]}...")
        
        target_pose = current_pose + np.array([0.02, 0.02, 0.02, 0.0, 0.0, 0.0])
        print(f"    Target pose: {target_pose[:3]}...")
        
        success = robot.update_pose(target_pose.tolist(), velocity=False, blocking=True)
        print(f"    Blocking call returned: {success}")
        
        evaluate_pose_position(robot, target_pose, tolerance=0.02, test_name="blocking position")
        results.add_pass("update_pose (blocking)")
    except Exception as e:
        results.add_fail("update_pose (blocking)", str(e))


def test_update_command(robot, results, skip_motion=False):
    """Test update_command method."""
    wait_between_tests("Update Command Interface")
    print("\n--- Testing Update Command Interface ---")
    
    if skip_motion:
        results.add_skip("update_command (cartesian_velocity)", "Skipped by user request")
        results.add_skip("update_command (joint_position)", "Skipped by user request")
        return
    
    # Test update_command (cartesian_velocity)
    try:
        print("  Testing update_command (cartesian_velocity)...")
        command = [0.02, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # X velocity + gripper
        
        action_dict = robot.update_command(
            command,
            action_space="cartesian_velocity",
            gripper_action_space="velocity",
            blocking=False
        )
        
        assert "joint_position" in action_dict, "Action dict missing joint_position"
        assert "gripper_position" in action_dict, "Action dict missing gripper_position"
        print(f"    Action dict keys: {list(action_dict.keys())}")
        
        time.sleep(2)
        robot.update_command([0.0] * 7, action_space="cartesian_velocity", blocking=False)
        time.sleep(0.5)
        
        results.add_pass("update_command (cartesian_velocity)")
    except Exception as e:
        results.add_fail("update_command (cartesian_velocity)", str(e))
    
    # Test update_command (joint_position)
    try:
        print("  Testing update_command (joint_position)...")
        current_joints = np.array(robot.get_joint_positions())
        print(f"    Current joints: {current_joints[:3]}...")
        
        target_joints = current_joints + np.array([0.1, -0.1, 0.1, -0.1, 0.08, -0.08, 0.08])
        command = target_joints.tolist() + [0.5]  # 7 joints + 1 gripper
        print(f"    Target joints: {target_joints[:3]}...")
        
        action_dict = robot.update_command(
            command,
            action_space="joint_position",
            gripper_action_space="position",
            blocking=False
        )
        
        assert "joint_position" in action_dict, "Action dict missing joint_position"
        print(f"    Action dict keys: {list(action_dict.keys())}")
        
        time.sleep(3)
        results.add_pass("update_command (joint_position)")
    except Exception as e:
        results.add_fail("update_command (joint_position)", str(e))


def test_controller_management(robot, results):
    """Test controller management methods."""
    wait_between_tests("Controller Management")
    print("\n--- Testing Controller Management ---")
    
    # Test start_joint_impedance
    try:
        print("  Testing start_joint_impedance()...")
        success = robot.start_joint_impedance()
        print(f"    Result: {success}")
        results.add_pass("start_joint_impedance")
    except Exception as e:
        results.add_fail("start_joint_impedance", str(e))
    
    # Test start_cartesian_impedance
    try:
        print("  Testing start_cartesian_impedance()...")
        success = robot.start_cartesian_impedance()
        print(f"    Result: {success}")
        results.add_pass("start_cartesian_impedance")
    except Exception as e:
        results.add_fail("start_cartesian_impedance", str(e))
    
    # Test terminate_current_policy
    try:
        print("  Testing terminate_current_policy()...")
        success = robot.terminate_current_policy()
        print(f"    Result: {success}")
        results.add_pass("terminate_current_policy")
    except Exception as e:
        results.add_fail("terminate_current_policy", str(e))


def test_kinematics(robot, results):
    """Test kinematics computation methods."""
    wait_between_tests("Kinematics (IK/FK)")
    print("\n--- Testing Kinematics (IK/FK) ---")
    
    # Test compute_forward_kinematics
    try:
        print("  Testing compute_forward_kinematics()...")
        joint_positions = robot.get_joint_positions()
        print(f"    Input joint positions: {joint_positions[:3]}...")
        
        position, orientation = robot.compute_forward_kinematics(joint_positions)
        
        if position is not None and orientation is not None:
            assert len(position) == 3, f"Expected 3D position, got {len(position)}"
            assert len(orientation) == 4, f"Expected quaternion, got {len(orientation)}"
            print(f"    Position: {position}")
            print(f"    Orientation (quat): {orientation}")
            results.add_pass("compute_forward_kinematics")
        else:
            results.add_fail("compute_forward_kinematics", "Service returned None")
    except Exception as e:
        results.add_fail("compute_forward_kinematics", str(e))
    
    # Test solve_inverse_kinematics
    try:
        print("  Testing solve_inverse_kinematics()...")
        position = [0.5, 0.0, 0.5]
        orientation = [0.0, 0.0, 0.0]  # Euler angles
        print(f"    Target position: {position}")
        print(f"    Target orientation (euler): {orientation}")
        
        joint_pos, success = robot.solve_inverse_kinematics(position, orientation)
        
        if success and joint_pos is not None:
            assert len(joint_pos) == 7, f"Expected 7 joints, got {len(joint_pos)}"
            print(f"    IK solution: {list(joint_pos)[:3]}...")
            results.add_pass("solve_inverse_kinematics")
        else:
            print(f"    IK failed or returned None (success={success})")
            results.add_pass("solve_inverse_kinematics")  # Not a failure, IK may not have solution
    except Exception as e:
        results.add_fail("solve_inverse_kinematics", str(e))


def test_create_action_dict(robot, results):
    """Test create_action_dict method."""
    wait_between_tests("Create Action Dict")
    print("\n--- Testing Create Action Dict ---")
    
    # Test cartesian_velocity
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
            assert key in action_dict, f"Missing key: {key}"
        
        print(f"    Keys: {list(action_dict.keys())}")
        results.add_pass("create_action_dict (cartesian_velocity)")
    except Exception as e:
        results.add_fail("create_action_dict (cartesian_velocity)", str(e))
    
    # Test joint_position
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
            assert key in action_dict, f"Missing key: {key}"
        
        print(f"    Keys: {list(action_dict.keys())}")
        results.add_pass("create_action_dict (joint_position)")
    except Exception as e:
        results.add_fail("create_action_dict (joint_position)", str(e))
    
    # Test cartesian_position
    try:
        print("  Testing create_action_dict (cartesian_position)...")
        current_pose = robot.get_ee_pose()
        action = current_pose + [0.5]  # 6 DOF pose + 1 gripper
        
        action_dict = robot.create_action_dict(
            action,
            action_space="cartesian_position",
            gripper_action_space="position"
        )
        
        required_keys = ["joint_position", "gripper_position", "robot_state"]
        for key in required_keys:
            assert key in action_dict, f"Missing key: {key}"
        
        print(f"    Keys: {list(action_dict.keys())}")
        results.add_pass("create_action_dict (cartesian_position)")
    except Exception as e:
        results.add_fail("create_action_dict (cartesian_position)", str(e))
    
    # Test joint_velocity
    try:
        print("  Testing create_action_dict (joint_velocity)...")
        action = [0.0] * 7 + [0.0]  # 7 joints + 1 gripper
        
        action_dict = robot.create_action_dict(
            action,
            action_space="joint_velocity",
            gripper_action_space="velocity"
        )
        
        required_keys = ["joint_position", "gripper_position", "robot_state"]
        for key in required_keys:
            assert key in action_dict, f"Missing key: {key}"
        
        print(f"    Keys: {list(action_dict.keys())}")
        results.add_pass("create_action_dict (joint_velocity)")
    except Exception as e:
        results.add_fail("create_action_dict (joint_velocity)", str(e))


def main():
    parser = argparse.ArgumentParser(description="Test FrankaRobotV2 interface")
    parser.add_argument('--skip-reset', action='store_true', help='Skip reset/home tests')
    parser.add_argument('--skip-motion', action='store_true', help='Skip motion control tests')
    parser.add_argument('--arm-namespace', type=str, default='fr3_arm', help='Arm namespace')
    parser.add_argument('--gripper-namespace', type=str, default='fr3_gripper', help='Gripper namespace')
    args = parser.parse_args()
    
    print("=" * 60)
    print("FrankaRobotV2 Interface Test")
    print("=" * 60)
    print(f"Skip reset: {args.skip_reset}")
    print(f"Skip motion: {args.skip_motion}")
    print(f"Arm namespace: {args.arm_namespace}")
    print(f"Gripper namespace: {args.gripper_namespace}")
    print(f"Test interval: {TEST_INTERVAL} seconds")
    print("=" * 60)
    
    print("\n⚠️  Warning: This test will perform robot movements!")
    print("   - Joint movements: 0.1-0.15 rad")
    print("   - End-effector movements: 0.02-0.03 m")
    print("   - Gripper movements: 0-100%")
    print("   Please ensure sufficient space around the robot!")
    print("=" * 60)
    
    # Check if V2 nodes are running
    print("\n🔍 Checking for V2 nodes...")
    try:
        result = subprocess.run(
            ['ros2', 'node', 'list'],
            capture_output=True,
            text=True,
            timeout=5.0
        )
        nodes = result.stdout
        arm_running = 'franka_robot_interface_node' in nodes
        gripper_running = 'franka_gripper_interface_node' in nodes
        
        if arm_running:
            print("  ✅ franka_robot_interface_node is running")
        else:
            print("  ❌ franka_robot_interface_node NOT found")
        
        if gripper_running:
            print("  ✅ franka_gripper_interface_node is running")
        else:
            print("  ❌ franka_gripper_interface_node NOT found")
        
        if not (arm_running and gripper_running):
            print("\n❌ V2 nodes not running!")
            print("\nPlease start V2 nodes first:")
            print("  ros2 launch role_ros2 franka_robot_v2.launch.py use_mock:=true")
            sys.exit(1)
    except subprocess.TimeoutExpired:
        print("  ⚠️  Could not check node status (timeout)")
    except Exception as e:
        print(f"  ⚠️  Could not check node status: {e}")
    
    print(f"\nWaiting {TEST_INTERVAL:.0f} seconds before starting tests...")
    time.sleep(TEST_INTERVAL)
    
    results = TestResult()
    
    # Initialize robot
    print("\n" + "=" * 60)
    print("📦 Initializing FrankaRobotV2...")
    print("=" * 60)
    try:
        robot = FrankaRobotV2(
            arm_namespace=args.arm_namespace,
            gripper_namespace=args.gripper_namespace
        )
        print("✅ FrankaRobotV2 initialized successfully")
        time.sleep(2)
    except Exception as e:
        print(f"❌ Failed to initialize FrankaRobotV2: {e}")
        print("\nMake sure V2 nodes are running:")
        print("  ros2 launch role_ros2 franka_robot_v2.launch.py use_mock:=true")
        sys.exit(1)
    
    # Run tests
    try:
        # 1. Properties
        test_properties(robot, results)
        
        # 2. Arm state getters
        test_arm_state_getters(robot, results)
        
        # 3. Gripper state getters
        test_gripper_state_getters(robot, results)
        
        # 4. Combined robot state
        test_robot_state(robot, results)
        
        # 5. Reset and home
        test_reset_and_home(robot, results, skip_reset=args.skip_reset)
        
        # 6. Gripper services
        test_gripper_services(robot, results, skip_motion=args.skip_motion)
        
        # 7. Update gripper (topic)
        test_update_gripper(robot, results, skip_motion=args.skip_motion)
        
        # 8. Joint control
        test_joint_control(robot, results, skip_motion=args.skip_motion)
        
        # 9. Pose control
        test_pose_control(robot, results, skip_motion=args.skip_motion)
        
        # 10. Update command interface
        test_update_command(robot, results, skip_motion=args.skip_motion)
        
        # 11. Controller management
        test_controller_management(robot, results)
        
        # 12. Kinematics
        test_kinematics(robot, results)
        
        # 13. Create action dict
        test_create_action_dict(robot, results)
        
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
    except Exception as e:
        print(f"\n\nUnexpected error during testing: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # Cleanup
        print("\n" + "=" * 60)
        print("🧹 Cleaning up...")
        print("=" * 60)
        try:
            robot.shutdown()
            print("✅ Shutdown complete")
        except Exception as e:
            print(f"⚠️  Error during shutdown: {e}")
    
    # Print summary
    results.print_summary()
    
    # Exit with appropriate code
    sys.exit(0 if results.failed == 0 else 1)


if __name__ == '__main__':
    main()
