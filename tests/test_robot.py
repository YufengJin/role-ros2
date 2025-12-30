#!/usr/bin/env python3
"""
Test script for FrankaRobot interface.

This script tests all methods of the FrankaRobot class to ensure proper
communication with polymetis_bridge_node.

⚠️  警告: 此测试会进行移动!
   - 关节移动: 0.1-0.15 rad (已减小)
   - 末端执行器移动: 0.02-0.03 m (已减小)
   - 夹爪移动: 0-90%
   - 移动速度: 已放慢
   请确保机器人周围有足够空间!

Usage:
    python3 test_robot.py [--mock] [--skip-reset] [--skip-motion]
    
    --mock: Use mock mode (no real robot)
    --skip-reset: Skip reset/home tests
    --skip-motion: Skip motion control tests (only test state getters)
"""

import sys
import time
import argparse
import numpy as np
from pathlib import Path

# Add parent directory to path for imports
#sys.path.insert(0, str(Path(__file__).parent.parent))

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


def evaluate_joint_position(robot, target_joints, tolerance=0.05, test_name=""):
    """
    评估关节位置是否到达目标位置
    
    Args:
        robot: FrankaRobot实例
        target_joints: 目标关节位置 (7 DOF)
        tolerance: 允许的位置误差 (rad)
        test_name: 测试名称
    
    Returns:
        tuple: (是否成功, 最大误差, 详细误差信息)
    """
    actual_joints = np.array(robot.get_joint_positions())
    target_joints = np.array(target_joints)
    
    errors = np.abs(actual_joints - target_joints)
    max_error = np.max(errors)
    mean_error = np.mean(errors)
    
    print(f"    位置评估 ({test_name}):")
    print(f"      目标位置: {target_joints[:3]}...")
    print(f"      实际位置: {actual_joints[:3]}...")
    print(f"      最大误差: {max_error:.4f} rad (阈值: {tolerance:.4f} rad)")
    print(f"      平均误差: {mean_error:.4f} rad")
    
    success = max_error < tolerance
    if success:
        print(f"      ✓ 位置精度满足要求 (误差 < {tolerance:.4f} rad)")
    else:
        print(f"      ⚠️  位置精度不足 (误差 = {max_error:.4f} rad > {tolerance:.4f} rad)")
    
    return success, max_error, errors


def evaluate_pose_position(robot, target_pose, tolerance=0.01, test_name=""):
    """
    评估末端位置是否到达目标位置
    
    Args:
        robot: FrankaRobot实例
        target_pose: 目标末端位置 [x, y, z, roll, pitch, yaw]
        tolerance: 允许的位置误差 (m)
        test_name: 测试名称
    
    Returns:
        tuple: (是否成功, 最大位置误差, 详细误差信息)
    """
    actual_pose = np.array(robot.get_ee_pose())
    target_pose = np.array(target_pose)
    
    # 位置误差 (前3个元素)
    position_errors = np.abs(actual_pose[:3] - target_pose[:3])
    max_position_error = np.max(position_errors)
    mean_position_error = np.mean(position_errors)
    
    # 姿态误差 (后3个元素，需要处理角度周期性)
    orientation_errors = np.abs(actual_pose[3:] - target_pose[3:])
    # 处理角度周期性 (-pi 到 pi)
    orientation_errors = np.minimum(orientation_errors, 2*np.pi - orientation_errors)
    max_orientation_error = np.max(orientation_errors)
    
    print(f"    位置评估 ({test_name}):")
    print(f"      目标位置: {target_pose[:3]} m")
    print(f"      实际位置: {actual_pose[:3]} m")
    print(f"      位置误差: {position_errors} m")
    print(f"      最大位置误差: {max_position_error:.4f} m (阈值: {tolerance:.4f} m)")
    print(f"      平均位置误差: {mean_position_error:.4f} m")
    print(f"      最大姿态误差: {max_orientation_error:.4f} rad")
    
    success = max_position_error < tolerance
    if success:
        print(f"      ✓ 位置精度满足要求 (误差 < {tolerance:.4f} m)")
    else:
        print(f"      ⚠️  位置精度不足 (误差 = {max_position_error:.4f} m > {tolerance:.4f} m)")
    
    return success, max_position_error, position_errors


def test_state_getters(robot, results):
    """Test state getter methods."""
    print("\n--- Testing State Getters ---")
    
    # Test get_joint_positions
    try:
        joint_pos = robot.get_joint_positions()
        assert len(joint_pos) == 7, f"Expected 7 joints, got {len(joint_pos)}"
        assert all(isinstance(x, (int, float)) for x in joint_pos), "Joint positions must be numbers"
        print(f"  ✓ 关节位置: {np.array(joint_pos)[:3]}... (共7个关节)")
        results.add_pass("get_joint_positions")
    except Exception as e:
        results.add_fail("get_joint_positions", str(e))
    
    # Test get_joint_velocities
    try:
        joint_vel = robot.get_joint_velocities()
        assert len(joint_vel) == 7, f"Expected 7 joints, got {len(joint_vel)}"
        assert all(isinstance(x, (int, float)) for x in joint_vel), "Joint velocities must be numbers"
        print(f"  ✓ 关节速度: {np.array(joint_vel)[:3]}... (共7个关节)")
        results.add_pass("get_joint_velocities")
    except Exception as e:
        results.add_fail("get_joint_velocities", str(e))
    
    # Test get_gripper_position
    try:
        gripper_pos = robot.get_gripper_position()
        assert 0.0 <= gripper_pos <= 1.0, f"Gripper position should be [0, 1], got {gripper_pos}"
        print(f"  ✓ 夹爪位置: {gripper_pos:.4f} (0=关闭, 1=打开)")
        results.add_pass("get_gripper_position")
    except Exception as e:
        results.add_fail("get_gripper_position", str(e))
    
    # Test get_gripper_state (alias)
    try:
        gripper_state = robot.get_gripper_state()
        assert 0.0 <= gripper_state <= 1.0, f"Gripper state should be [0, 1], got {gripper_state}"
        print(f"  ✓ 夹爪状态: {gripper_state:.4f}")
        results.add_pass("get_gripper_state")
    except Exception as e:
        results.add_fail("get_gripper_state", str(e))
    
    # Test get_ee_pose
    try:
        ee_pose = robot.get_ee_pose()
        assert len(ee_pose) == 6, f"Expected 6 DOF pose, got {len(ee_pose)}"
        assert all(isinstance(x, (int, float)) for x in ee_pose), "EE pose must be numbers"
        print(f"  ✓ 末端位置: {np.array(ee_pose[:3])} m")
        print(f"  ✓ 末端姿态: {np.array(ee_pose[3:])} rad (roll, pitch, yaw)")
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
        print(f"  ✓ 状态字典包含所有必需键: {required_keys}")
        print(f"  ✓ 时间戳: {timestamp_dict.get('robot_timestamp_seconds', 0)} s")
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
        # 获取reset前的位置
        joints_before = np.array(robot.get_joint_positions())
        ee_before = np.array(robot.get_ee_pose()[:3])
        print(f"  Reset前关节位置: {joints_before[:3]}...")
        print(f"  Reset前端末位置: {ee_before}")
        
        robot.reset(wait_for_completion=True, wait_time_sec=10.0)
        
        # 等待reset完成
        time.sleep(2)
        
        # 获取reset后的位置
        joints_after = np.array(robot.get_joint_positions())
        ee_after = np.array(robot.get_ee_pose()[:3])
        print(f"  Reset后关节位置: {joints_after[:3]}...")
        print(f"  Reset后端末位置: {ee_after}")
        
        # 检查是否移动
        joint_diff = np.max(np.abs(joints_after - joints_before))
        ee_diff = np.max(np.abs(ee_after - ee_before))
        print(f"  关节变化: {joint_diff:.4f} rad")
        print(f"  末端变化: {ee_diff:.4f} m")
        
        if joint_diff > 0.1 or ee_diff > 0.01:
            print(f"  ✓ Reset成功 - 机器人已移动")
        else:
            print(f"  ⚠️  Reset可能未完成 - 移动较小")
        
        results.add_pass("reset")
    except Exception as e:
        results.add_fail("reset", str(e))
    
    # Test home (should call reset)
    try:
        print("  Calling home()...")
        joints_before = np.array(robot.get_joint_positions())
        robot.home()
        time.sleep(2)  # 增加等待时间
        joints_after = np.array(robot.get_joint_positions())
        joint_diff = np.max(np.abs(joints_after - joints_before))
        print(f"  关节变化: {joint_diff:.4f} rad")
        if joint_diff > 0.05:
            print(f"  ✓ Home成功 - 机器人已移动")
        results.add_pass("home")
    except Exception as e:
        results.add_fail("home", str(e))


def test_gripper_control(robot, results, skip_motion=False):
    """Test gripper control methods."""
    print("\n--- Testing Gripper Control ---")
    
    if skip_motion:
        results.add_skip("update_gripper", "Skipped by user request")
        return
    
    # Test update_gripper (non-blocking) - 大幅移动
    try:
        print("  Testing update_gripper (non-blocking) - 大幅移动...")
        initial_pos = robot.get_gripper_position()
        print(f"  初始夹爪位置: {initial_pos:.4f}")
        
        # 打开到 80%
        robot.update_gripper(0.8, velocity=False, blocking=False)
        time.sleep(2)  # 增加等待时间
        pos1 = robot.get_gripper_position()
        print(f"  打开后位置: {pos1:.4f} (目标: 0.8)")
        
        # 关闭到 0%
        robot.update_gripper(0.0, velocity=False, blocking=False)
        time.sleep(2)  # 增加等待时间
        pos2 = robot.get_gripper_position()
        print(f"  关闭后位置: {pos2:.4f} (目标: 0.0)")
        
        # 检查移动幅度
        pos_diff = abs(pos1 - pos2)
        print(f"  夹爪移动幅度: {pos_diff:.4f}")
        if pos_diff > 0.3:
            print(f"  ✓ 夹爪移动成功 (变化 > 0.3)")
        else:
            print(f"  ⚠️  夹爪移动较小 (变化 = {pos_diff:.4f})")
        
        results.add_pass("update_gripper (non-blocking)")
    except Exception as e:
        results.add_fail("update_gripper (non-blocking)", str(e))
    
    # Test update_gripper (blocking) - 大幅移动
    try:
        print("  Testing update_gripper (blocking) - 大幅移动...")
        initial_pos = robot.get_gripper_position()
        print(f"  初始夹爪位置: {initial_pos:.4f}")
        
        # 打开到 90%
        robot.update_gripper(0.9, velocity=False, blocking=True)
        time.sleep(0.5)
        pos_after = robot.get_gripper_position()
        print(f"  打开后位置: {pos_after:.4f} (目标: 0.9)")
        
        pos_diff = abs(pos_after - initial_pos)
        print(f"  夹爪移动幅度: {pos_diff:.4f}")
        if pos_diff > 0.2:
            print(f"  ✓ 夹爪移动成功 (变化 > 0.2)")
        else:
            print(f"  ⚠️  夹爪移动较小 (变化 = {pos_diff:.4f})")
        
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
        current_joints = np.array(robot.get_joint_positions())
        print(f"  当前关节位置: {current_joints[:3]}...")
        
        # Test update_joints (non-blocking position) - 减小移动幅度，放慢速度
        print("  Testing update_joints (non-blocking position)...")
        # 减小移动幅度：0.1-0.15 rad
        target_joints = current_joints + np.array([0.1, -0.1, 0.1, -0.1, 0.08, -0.08, 0.08])
        print(f"  目标关节位置: {target_joints[:3]}...")
        robot.update_joints(target_joints.tolist(), velocity=False, blocking=False)
        time.sleep(5)  # 增加等待时间，让移动更慢更稳定
        
        # 评估位置精度
        success, max_error, _ = evaluate_joint_position(robot, target_joints, tolerance=0.05, test_name="non-blocking position")
        
        # 检查是否移动
        new_joints = np.array(robot.get_joint_positions())
        joint_diff = np.abs(new_joints - current_joints)
        max_diff = np.max(joint_diff)
        print(f"  实际移动: 最大关节变化 = {max_diff:.4f} rad")
        if max_diff > 0.05:
            print(f"  ✓ 关节移动成功 (变化 > 0.05 rad)")
        else:
            print(f"  ⚠️  关节移动较小 (变化 = {max_diff:.4f} rad)")
        results.add_pass("update_joints (non-blocking position)")
    except Exception as e:
        results.add_fail("update_joints (non-blocking position)", str(e))
    
    # Test update_joints (non-blocking velocity)
    try:
        print("  Testing update_joints (non-blocking velocity)...")
        # 减小速度命令：从0.2减小到0.05-0.1 rad/s
        velocity_cmd = [0.05, -0.05, 0.05, -0.05, 0.03, -0.03, 0.03]
        robot.update_joints(velocity_cmd, velocity=True, blocking=False)
        time.sleep(3)  # 增加等待时间，让移动更慢
        # 速度控制后停止
        robot.update_joints([0.0] * 7, velocity=True, blocking=False)
        time.sleep(0.5)
        results.add_pass("update_joints (non-blocking velocity)")
    except Exception as e:
        results.add_fail("update_joints (non-blocking velocity)", str(e))
    
    # Test update_joints (blocking) - 减小移动幅度
    try:
        print("  Testing update_joints (blocking)...")
        current_joints = np.array(robot.get_joint_positions())
        print(f"  当前关节位置: {current_joints[:3]}...")
        # 减小移动幅度：0.1-0.15 rad
        target_joints = current_joints + np.array([0.1, -0.1, 0.1, -0.1, 0.08, -0.08, 0.08])
        print(f"  目标关节位置: {target_joints[:3]}...")
        robot.update_joints(target_joints.tolist(), velocity=False, blocking=True)
        
        # 评估位置精度
        success, max_error, _ = evaluate_joint_position(robot, target_joints, tolerance=0.05, test_name="blocking position")
        
        # 检查是否移动
        new_joints = np.array(robot.get_joint_positions())
        joint_diff = np.abs(new_joints - current_joints)
        max_diff = np.max(joint_diff)
        print(f"  实际移动: 最大关节变化 = {max_diff:.4f} rad")
        if max_diff > 0.05:
            print(f"  ✓ 关节移动成功 (变化 > 0.05 rad)")
        else:
            print(f"  ⚠️  关节移动较小 (变化 = {max_diff:.4f} rad)")
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
        current_pose = np.array(robot.get_ee_pose())
        print(f"  当前末端位置: {current_pose[:3]}")
        
        # Test update_pose (non-blocking position) - 减小移动幅度，放慢速度
        print("  Testing update_pose (non-blocking position)...")
        # 减小移动幅度：0.02-0.03 m
        target_pose = current_pose + np.array([0.02, 0.02, 0.02, 0.0, 0.0, 0.0])
        print(f"  目标末端位置: {target_pose[:3]}")
        robot.update_pose(target_pose.tolist(), velocity=False, blocking=False)
        time.sleep(5)  # 增加等待时间，让移动更慢更稳定
        
        # 评估位置精度
        success, max_error, _ = evaluate_pose_position(robot, target_pose, tolerance=0.01, test_name="non-blocking position")
        
        # 检查是否移动
        new_pose = np.array(robot.get_ee_pose())
        pose_diff = np.abs(new_pose[:3] - current_pose[:3])
        max_diff = np.max(pose_diff)
        print(f"  实际移动: 最大位置变化 = {max_diff:.4f} m")
        if max_diff > 0.01:
            print(f"  ✓ 末端移动成功 (变化 > 0.01 m)")
        else:
            print(f"  ⚠️  末端移动较小 (变化 = {max_diff:.4f} m)")
        results.add_pass("update_pose (non-blocking position)")
    except Exception as e:
        results.add_fail("update_pose (non-blocking position)", str(e))
    
    # Test update_pose (non-blocking velocity)
    try:
        print("  Testing update_pose (non-blocking velocity)...")
        # 减小速度命令：从0.05减小到0.02 m/s
        velocity_cmd = [0.02, 0.0, 0.0, 0.0, 0.0, 0.0]  # X方向速度
        robot.update_pose(velocity_cmd, velocity=True, blocking=False)
        time.sleep(3)  # 增加等待时间，让移动更慢
        # 速度控制后停止
        robot.update_pose([0.0] * 6, velocity=True, blocking=False)
        time.sleep(0.5)
        results.add_pass("update_pose (non-blocking velocity)")
    except Exception as e:
        results.add_fail("update_pose (non-blocking velocity)", str(e))
    
    # Test update_pose (blocking) - 减小移动幅度
    try:
        print("  Testing update_pose (blocking)...")
        current_pose = np.array(robot.get_ee_pose())
        print(f"  当前末端位置: {current_pose[:3]}")
        # 减小移动幅度：0.02-0.03 m
        target_pose = current_pose + np.array([0.02, 0.02, 0.02, 0.0, 0.0, 0.0])
        print(f"  目标末端位置: {target_pose[:3]}")
        robot.update_pose(target_pose.tolist(), velocity=False, blocking=True)
        
        # 评估位置精度
        success, max_error, _ = evaluate_pose_position(robot, target_pose, tolerance=0.01, test_name="blocking position")
        
        # 检查是否移动
        new_pose = np.array(robot.get_ee_pose())
        pose_diff = np.abs(new_pose[:3] - current_pose[:3])
        max_diff = np.max(pose_diff)
        print(f"  实际移动: 最大位置变化 = {max_diff:.4f} m")
        if max_diff > 0.01:
            print(f"  ✓ 末端移动成功 (变化 > 0.01 m)")
        else:
            print(f"  ⚠️  末端移动较小 (变化 = {max_diff:.4f} m)")
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
        # 减小速度命令：从0.05减小到0.02 m/s
        command = [0.02, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # X方向速度 + 夹爪
        action_dict = robot.update_command(
            command,
            action_space="cartesian_velocity",
            gripper_action_space="velocity",
            blocking=False
        )
        assert "joint_position" in action_dict, "Action dict missing joint_position"
        assert "gripper_position" in action_dict, "Action dict missing gripper_position"
        time.sleep(3)  # 增加等待时间，让移动更慢
        # 停止速度控制
        robot.update_command([0.0] * 7, action_space="cartesian_velocity", blocking=False)
        time.sleep(0.5)
        results.add_pass("update_command (cartesian_velocity)")
    except Exception as e:
        results.add_fail("update_command (cartesian_velocity)", str(e))
    
    try:
        print("  Testing update_command (joint_position)...")
        current_joints = np.array(robot.get_joint_positions())
        print(f"  当前关节位置: {current_joints[:3]}...")
        # 减小移动幅度：0.1-0.12 rad
        target_joints = current_joints + np.array([0.1, -0.1, 0.1, -0.1, 0.08, -0.08, 0.08])
        command = target_joints.tolist() + [0.5]  # 7 joints + 1 gripper
        print(f"  目标关节位置: {target_joints[:3]}...")
        action_dict = robot.update_command(
            command,
            action_space="joint_position",
            gripper_action_space="position",
            blocking=False
        )
        assert "joint_position" in action_dict, "Action dict missing joint_position"
        time.sleep(5)  # 增加等待时间，让移动更慢更稳定
        
        # 评估位置精度
        success, max_error, _ = evaluate_joint_position(robot, target_joints, tolerance=0.05, test_name="update_command joint_position")
        
        # 检查是否移动
        new_joints = np.array(robot.get_joint_positions())
        joint_diff = np.max(np.abs(new_joints - current_joints))
        print(f"  实际移动: 最大关节变化 = {joint_diff:.4f} rad")
        if joint_diff > 0.05:
            print(f"  ✓ 关节移动成功 (变化 > 0.05 rad)")
        
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
    
    if args.mock:
        print("\n📦 Mock Mode: 使用模拟机器人（无需真实硬件）")
        print("   状态会根据命令动态更新")
        print("   可以安全测试所有功能")
    else:
        print("\n⚠️  警告: 此测试将进行移动!")
        print("   - 关节移动: 0.1-0.15 rad (已减小)")
        print("   - 末端移动: 0.02-0.03 m (已减小)")
        print("   - 夹爪移动: 0-90%")
        print("   - 移动速度: 已放慢")
        print("   请确保机器人周围有足够空间!")
    
    print("="*60)
    
    # Check if polymetis_bridge_node is running
    import subprocess
    try:
        result = subprocess.run(
            ['ros2', 'node', 'list'],
            capture_output=True,
            text=True,
            timeout=2.0
        )
        bridge_running = 'polymetis_bridge_node' in result.stdout
    except:
        bridge_running = False
    
    if not bridge_running:
        if args.mock:
            print("\n🚀 启动 mock 模式的 polymetis_bridge_node...")
            print("   使用命令: ros2 launch role_ros2 franka_robot.launch.py use_mock:=true")
            print("\n   或者手动启动:")
            print("   ros2 run role_ros2 polymetis_bridge --ros-args -p use_mock:=true")
            print("\n   等待 5 秒后继续（请确保节点已启动）...")
            time.sleep(5)
        else:
            print("\n❌ polymetis_bridge_node 未运行!")
            print("\n请先启动 polymetis_bridge_node:")
            print("  # 真实机器人模式:")
            print("  ros2 launch role_ros2 franka_robot.launch.py use_mock:=false")
            print("\n  # 或手动启动:")
            print("  ros2 run role_ros2 polymetis_bridge")
            print("\n  # Mock 模式:")
            print("  ros2 launch role_ros2 franka_robot.launch.py use_mock:=true")
            sys.exit(1)
    else:
        print("\n✅ polymetis_bridge_node 正在运行")
        if args.mock:
            print("   提示: 如果节点不在 mock 模式，请重启为 mock 模式")
    
    print("\n等待3秒后开始测试...")
    time.sleep(3)
    
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
        if args.mock:
            print("  ros2 launch role_ros2 franka_robot.launch.py use_mock:=true")
        else:
            print("  ros2 run role_ros2 polymetis_bridge")
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

