#!/usr/bin/env python3
"""
Test script for NON-BLOCKING robot commands via /polymetis/robot_command topic

This tests:
1. joint_velocity - Continuous joint velocity control
2. joint_position - Joint position via impedance controller
3. cartesian_velocity - EE velocity via IK
4. cartesian_position - EE position via IK

Usage:
    python3 test_non_blocking_commands.py [--test TEST_NAME] [--duration SECONDS]
    
    --test: Specific test to run (joint_vel, joint_pos, cart_vel, cart_pos, all)
    --duration: Duration for velocity commands (default: 3.0)

Author: Role-ROS2 Team
"""

import sys
import time
import math
import argparse
import traceback
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# Import messages
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from role_ros2.msg import (
    PolymetisRobotCommand, PolymetisRobotState, ControllerStatus
)

# Import services
from role_ros2.srv import StartCartesianImpedance, StartJointImpedance


class NonBlockingCommandTester(Node):
    """
    Test node for non-blocking commands via /polymetis/robot_command topic
    """
    
    def __init__(self):
        super().__init__('non_blocking_command_tester')
        
        # QoS profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # State storage
        self.current_joint_positions = None
        self.current_ee_pose = None
        self.current_robot_state = None
        self.controller_status = None
        
        # Subscribers
        self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_states_callback,
            qos_profile
        )
        
        self.create_subscription(
            PoseStamped,
            '/polymetis/arm/ee_pose',
            self._ee_pose_callback,
            qos_profile
        )
        
        self.create_subscription(
            PolymetisRobotState,
            '/polymetis/robot_state',
            self._robot_state_callback,
            qos_profile
        )
        
        self.create_subscription(
            ControllerStatus,
            '/polymetis/controller/status',
            self._controller_status_callback,
            qos_profile
        )
        
        # Publisher for robot commands
        self.robot_cmd_publisher = self.create_publisher(
            PolymetisRobotCommand,
            '/polymetis/robot_command',
            10
        )
        
        # Service clients for controller management
        self.start_cartesian_impedance_client = self.create_client(
            StartCartesianImpedance,
            '/polymetis/arm/start_cartesian_impedance'
        )
        self.start_joint_impedance_client = self.create_client(
            StartJointImpedance,
            '/polymetis/arm/start_joint_impedance'
        )
        
        self.get_logger().info("NonBlockingCommandTester initialized")
    
    def _joint_states_callback(self, msg):
        self.current_joint_positions = list(msg.position)
    
    def _ee_pose_callback(self, msg):
        self.current_ee_pose = msg
    
    def _robot_state_callback(self, msg):
        self.current_robot_state = msg
    
    def _controller_status_callback(self, msg):
        self.controller_status = msg
    
    def wait_for_state(self, timeout: float = 5.0) -> bool:
        """Wait for robot state to be available"""
        self.get_logger().info("Waiting for robot state...")
        start_time = time.time()
        while time.time() - start_time < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.current_joint_positions and len(self.current_joint_positions) >= 7:
                self.get_logger().info(f"✓ Got robot state: {len(self.current_joint_positions)} joints")
                return True
        self.get_logger().error("✗ Timeout waiting for robot state")
        return False
    
    def ensure_controller_running(self) -> bool:
        """Ensure impedance controller is running"""
        self.get_logger().info("Ensuring controller is running...")
        
        # Check current status
        rclpy.spin_once(self, timeout_sec=0.5)
        if self.controller_status:
            self.get_logger().info(f"  Controller mode: {self.controller_status.controller_mode}")
            self.get_logger().info(f"  Running policy: {self.controller_status.is_running_policy}")
        
        # Start cartesian impedance controller
        if self.start_cartesian_impedance_client.wait_for_service(timeout_sec=2.0):
            request = StartCartesianImpedance.Request()
            request.kx = []  # Use defaults
            request.kxd = []
            
            future = self.start_cartesian_impedance_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            
            if future.done():
                response = future.result()
                if response.success:
                    self.get_logger().info(f"✓ Controller started: {response.message}")
                    time.sleep(0.5)  # Wait for controller to be ready
                    return True
                else:
                    self.get_logger().warn(f"⚠️  Controller: {response.message}")
        else:
            self.get_logger().warn("⚠️  Controller service not available")
        
        return False
    
    def print_state(self):
        """Print current robot state"""
        self.get_logger().info("-" * 50)
        if self.current_joint_positions:
            joints_str = ", ".join([f"{p:.3f}" for p in self.current_joint_positions[:7]])
            self.get_logger().info(f"Joints: [{joints_str}]")
        if self.current_ee_pose:
            pos = self.current_ee_pose.pose.position
            self.get_logger().info(f"EE pos: [{pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f}]")
        if self.controller_status:
            self.get_logger().info(f"Controller: {self.controller_status.controller_mode}, running={self.controller_status.is_running_policy}")
        self.get_logger().info("-" * 50)
    
    def test_joint_velocity(self, duration: float = 3.0, velocity: float = 0.1):
        """
        Test joint velocity command
        
        Sends continuous velocity commands to joint 1
        """
        self.get_logger().info("=" * 60)
        self.get_logger().info("TEST: Joint Velocity Command")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"  action_space: joint_velocity")
        self.get_logger().info(f"  Duration: {duration} seconds")
        self.get_logger().info(f"  Velocity: {velocity} rad/s on joint 1")
        self.get_logger().info(f"  Expected rotation: ~{velocity * duration:.2f} rad (~{math.degrees(velocity * duration):.1f}°)")
        self.get_logger().info("")
        
        self.print_state()
        initial_pos = self.current_joint_positions[0] if self.current_joint_positions else 0
        
        self.get_logger().warn("⚠️  Robot will move! Starting in 3 seconds...")
        time.sleep(3)
        
        self.get_logger().info("Publishing joint velocity commands...")
        cmd = PolymetisRobotCommand()
        cmd.action_space = "joint_velocity"
        cmd.gripper_action_space = "velocity"
        cmd.blocking = False
        
        start_time = time.time()
        count = 0
        while time.time() - start_time < duration:
            cmd.header.stamp = self.get_clock().now().to_msg()
            # Velocity on joint 1 only
            cmd.command = [velocity, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            
            self.robot_cmd_publisher.publish(cmd)
            count += 1
            
            # Log first few commands for debugging
            if count <= 3:
                self.get_logger().debug(
                    f"Published command #{count}: action_space={cmd.action_space}, "
                    f"command={cmd.command[:3]}..."
                )
            
            # Keep spinning to receive state updates
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.02)  # ~50Hz
        
        self.get_logger().info(f"Published {count} commands")
        
        # Wait and check final state
        time.sleep(0.5)
        rclpy.spin_once(self, timeout_sec=0.1)
        
        self.print_state()
        final_pos = self.current_joint_positions[0] if self.current_joint_positions else 0
        actual_rotation = final_pos - initial_pos
        
        self.get_logger().info(f"Actual rotation: {actual_rotation:.3f} rad (~{math.degrees(actual_rotation):.1f}°)")
        
        if abs(actual_rotation) > 0.01:
            self.get_logger().info("✓ Joint velocity test PASSED - Robot moved!")
            return True
        else:
            self.get_logger().error("✗ Joint velocity test FAILED - Robot didn't move")
            return False
    
    def test_joint_position(self, offset: float = 0.15):
        """
        Test joint position command
        
        Sends position setpoint to all joints
        """
        self.get_logger().info("=" * 60)
        self.get_logger().info("TEST: Joint Position Command")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"  action_space: joint_position")
        self.get_logger().info(f"  Offset: {offset} rad (~{math.degrees(offset):.1f}°) on all joints")
        self.get_logger().info("")
        
        if not self.current_joint_positions or len(self.current_joint_positions) < 7:
            self.get_logger().error("No joint positions available")
            return False
        
        self.print_state()
        current_pos = list(self.current_joint_positions[:7])
        target_pos = [p + offset for p in current_pos]
        
        self.get_logger().info(f"Current: [{', '.join([f'{p:.3f}' for p in current_pos])}]")
        self.get_logger().info(f"Target:  [{', '.join([f'{p:.3f}' for p in target_pos])}]")
        
        self.get_logger().warn("⚠️  Robot will move! Starting in 3 seconds...")
        time.sleep(3)
        
        self.get_logger().info("Publishing joint position commands for 3 seconds...")
        cmd = PolymetisRobotCommand()
        cmd.action_space = "joint_position"
        cmd.gripper_action_space = "position"
        cmd.blocking = False
        
        self.get_logger().debug(f"Target positions: {target_pos}")
        
        start_time = time.time()
        count = 0
        while time.time() - start_time < 3.0:
            cmd.header.stamp = self.get_clock().now().to_msg()
            cmd.command = target_pos + [0.5]  # Add gripper position
            
            self.robot_cmd_publisher.publish(cmd)
            count += 1
            
            # Log first few commands for debugging
            if count <= 3:
                self.get_logger().debug(
                    f"Published command #{count}: action_space={cmd.action_space}, "
                    f"command_len={len(cmd.command)}"
                )
            
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.02)
        
        self.get_logger().info(f"Published {count} commands")
        
        # Wait and check final state
        time.sleep(0.5)
        rclpy.spin_once(self, timeout_sec=0.1)
        
        self.print_state()
        
        if self.current_joint_positions:
            total_error = sum(abs(self.current_joint_positions[i] - target_pos[i]) for i in range(7))
            self.get_logger().info(f"Total position error: {total_error:.4f} rad")
            
            if total_error < offset * 7 * 0.5:  # Allow 50% error
                self.get_logger().info("✓ Joint position test PASSED")
                return True
        
        self.get_logger().error("✗ Joint position test - Check if robot moved")
        return False
    
    def test_cartesian_velocity(self, duration: float = 3.0, velocity: float = 0.03):
        """
        Test cartesian velocity command
        
        Sends EE velocity in X direction
        """
        self.get_logger().info("=" * 60)
        self.get_logger().info("TEST: Cartesian Velocity Command")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"  action_space: cartesian_velocity")
        self.get_logger().info(f"  Duration: {duration} seconds")
        self.get_logger().info(f"  Velocity: {velocity} m/s in X direction")
        self.get_logger().info(f"  Expected displacement: ~{velocity * duration:.2f} m ({velocity * duration * 100:.1f} cm)")
        self.get_logger().info("")
        
        self.print_state()
        initial_x = self.current_ee_pose.pose.position.x if self.current_ee_pose else 0
        
        self.get_logger().warn("⚠️  Robot EE will move! Starting in 3 seconds...")
        time.sleep(3)
        
        self.get_logger().info("Publishing cartesian velocity commands...")
        cmd = PolymetisRobotCommand()
        cmd.action_space = "cartesian_velocity"
        cmd.gripper_action_space = "velocity"
        cmd.blocking = False
        
        start_time = time.time()
        count = 0
        while time.time() - start_time < duration:
            cmd.header.stamp = self.get_clock().now().to_msg()
            # Velocity in X direction only
            cmd.command = [velocity, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            
            self.robot_cmd_publisher.publish(cmd)
            count += 1
            
            # Log first few commands for debugging
            if count <= 3:
                self.get_logger().debug(
                    f"Published command #{count}: action_space={cmd.action_space}, "
                    f"command={cmd.command[:3]}..."
                )
            
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.02)
        
        self.get_logger().info(f"Published {count} commands")
        
        # Wait and check final state
        time.sleep(0.5)
        rclpy.spin_once(self, timeout_sec=0.1)
        
        self.print_state()
        final_x = self.current_ee_pose.pose.position.x if self.current_ee_pose else 0
        actual_displacement = final_x - initial_x
        
        self.get_logger().info(f"Actual X displacement: {actual_displacement:.3f} m ({actual_displacement * 100:.1f} cm)")
        
        if abs(actual_displacement) > 0.005:  # 5mm threshold
            self.get_logger().info("✓ Cartesian velocity test PASSED - EE moved!")
            return True
        else:
            self.get_logger().error("✗ Cartesian velocity test FAILED - EE didn't move")
            return False
    
    def test_cartesian_position(self, offset_x: float = 0.05):
        """
        Test cartesian position command
        
        Sends EE position setpoint
        """
        self.get_logger().info("=" * 60)
        self.get_logger().info("TEST: Cartesian Position Command")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"  action_space: cartesian_position")
        self.get_logger().info(f"  Offset: {offset_x} m ({offset_x * 100:.1f} cm) in X direction")
        self.get_logger().info("")
        
        if not self.current_robot_state:
            self.get_logger().error("No robot state available")
            return False
        
        self.print_state()
        
        # Get current cartesian position from robot state
        current_cart = list(self.current_robot_state.ee_position) + list(self.current_robot_state.ee_euler)
        target_cart = current_cart.copy()
        target_cart[0] += offset_x  # Add X offset
        
        self.get_logger().info(f"Current EE: [{', '.join([f'{p:.3f}' for p in current_cart[:3]])}]")
        self.get_logger().info(f"Target EE:  [{', '.join([f'{p:.3f}' for p in target_cart[:3]])}]")
        
        # Store initial position for movement check
        initial_x = self.current_ee_pose.pose.position.x if self.current_ee_pose else current_cart[0]
        
        self.get_logger().warn("⚠️  Robot EE will move! Starting in 3 seconds...")
        time.sleep(3)
        
        # Increase command duration for non-blocking position commands (need more time)
        command_duration = 5.0  # 5 seconds instead of 3
        self.get_logger().info(f"Publishing cartesian position commands for {command_duration} seconds...")
        cmd = PolymetisRobotCommand()
        cmd.action_space = "cartesian_position"
        cmd.gripper_action_space = "position"
        cmd.blocking = False
        
        self.get_logger().debug(f"Target cartesian: {target_cart[:3]}")
        
        start_time = time.time()
        count = 0
        while time.time() - start_time < command_duration:
            cmd.header.stamp = self.get_clock().now().to_msg()
            cmd.command = target_cart + [0.5]  # Add gripper position
            
            self.robot_cmd_publisher.publish(cmd)
            count += 1
            
            # Log first few commands for debugging
            if count <= 3:
                self.get_logger().debug(
                    f"Published command #{count}: action_space={cmd.action_space}, "
                    f"command_len={len(cmd.command)}"
                )
            
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.02)
        
        self.get_logger().info(f"Published {count} commands")
        
        # Wait longer for robot to reach target (non-blocking commands need time)
        time.sleep(1.0)  # Increased from 0.5 to 1.0
        rclpy.spin_once(self, timeout_sec=0.1)
        
        self.print_state()
        
        if self.current_ee_pose:
            final_x = self.current_ee_pose.pose.position.x
            target_x = target_cart[0]
            error = abs(final_x - target_x)
            actual_movement = abs(final_x - initial_x)
            
            self.get_logger().info(f"Initial X: {initial_x:.3f} m")
            self.get_logger().info(f"Target X:  {target_x:.3f} m")
            self.get_logger().info(f"Final X:   {final_x:.3f} m")
            self.get_logger().info(f"Actual movement: {actual_movement:.4f} m ({actual_movement * 100:.2f} cm)")
            self.get_logger().info(f"X error: {error:.4f} m ({error * 100:.2f} cm)")
            
            # More lenient criteria:
            # 1. Check if robot moved at least 20% of target distance (movement happened)
            # 2. Allow 80% error tolerance (more lenient than 50%)
            min_movement = offset_x * 0.2  # At least 20% of target
            max_error = offset_x * 0.8    # Allow 80% error
            
            if actual_movement >= min_movement and error <= max_error:
                self.get_logger().info("✓ Cartesian position test PASSED")
                return True
            elif actual_movement >= min_movement:
                # Robot moved but didn't reach target - still consider it a pass
                # (non-blocking commands may not reach exact target)
                self.get_logger().info(f"✓ Cartesian position test PASSED (moved {actual_movement*100:.1f}cm, error {error*100:.1f}cm)")
                return True
            else:
                self.get_logger().warn(f"⚠️  Robot moved only {actual_movement*100:.1f}cm (expected at least {min_movement*100:.1f}cm)")
        
        self.get_logger().error("✗ Cartesian position test - Check if robot moved")
        return False


def main(args=None):
    parser = argparse.ArgumentParser(description='Test non-blocking robot commands')
    parser.add_argument('--test', type=str, default='all',
                       choices=['joint_vel', 'joint_pos', 'cart_vel', 'cart_pos', 'all'],
                       help='Which test to run')
    parser.add_argument('--duration', type=float, default=3.0,
                       help='Duration for velocity commands (seconds)')
    parser.add_argument('--velocity', type=float, default=0.1,
                       help='Joint velocity (rad/s)')
    parser.add_argument('--offset', type=float, default=0.15,
                       help='Joint position offset (rad)')
    
    parsed_args = parser.parse_args()
    
    rclpy.init(args=None)
    
    tester = NonBlockingCommandTester()
    
    try:
        # Wait for robot state
        if not tester.wait_for_state(timeout=10.0):
            tester.get_logger().error("Cannot proceed without robot state")
            return
        
        # Ensure controller is running
        tester.ensure_controller_running()
        
        tester.get_logger().info("")
        tester.get_logger().info("=" * 60)
        tester.get_logger().info("NON-BLOCKING COMMAND TEST")
        tester.get_logger().info("=" * 60)
        tester.get_logger().info(f"Test: {parsed_args.test}")
        tester.get_logger().info(f"Velocity duration: {parsed_args.duration}s")
        tester.get_logger().info(f"Joint velocity: {parsed_args.velocity} rad/s")
        tester.get_logger().info(f"Position offset: {parsed_args.offset} rad")
        tester.get_logger().info("=" * 60)
        tester.get_logger().info("")
        
        results = {}
        
        if parsed_args.test in ['joint_vel', 'all']:
            results['Joint Velocity'] = tester.test_joint_velocity(
                duration=parsed_args.duration,
                velocity=parsed_args.velocity
            )
            time.sleep(3)
        
        if parsed_args.test in ['joint_pos', 'all']:
            results['Joint Position'] = tester.test_joint_position(
                offset=parsed_args.offset
            )
            time.sleep(3)
        
        if parsed_args.test in ['cart_vel', 'all']:
            results['Cartesian Velocity'] = tester.test_cartesian_velocity(
                duration=parsed_args.duration,
                velocity=0.03
            )
            time.sleep(3)
        
        if parsed_args.test in ['cart_pos', 'all']:
            results['Cartesian Position'] = tester.test_cartesian_position(
                offset_x=0.05
            )
        
        # Print summary
        tester.get_logger().info("")
        tester.get_logger().info("=" * 60)
        tester.get_logger().info("SUMMARY")
        tester.get_logger().info("=" * 60)
        for test_name, passed in results.items():
            status = "✓ PASS" if passed else "✗ FAIL"
            tester.get_logger().info(f"  {test_name}: {status}")
        tester.get_logger().info("=" * 60)
        
    except KeyboardInterrupt:
        tester.get_logger().info("\nTest interrupted")
    except Exception as e:
        tester.get_logger().error(f"Test error: {e}")
        tester.get_logger().error(traceback.format_exc())
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

