#!/usr/bin/env python3
"""
Test script to verify robot end-effector follows VR actions.

This script:
1. Publishes fake VRPolicyAction messages in different directions
2. Subscribes to robot state to monitor end-effector pose
3. Verifies that robot EE follows the action commands

Usage:
    # 1) Start robot bridge (mock or real)
    ros2 run role_ros2 polymetis_bridge_node --ros-args -p use_mock:=true
    
    # 2) Start teleoperation_node
    ros2 run role_ros2 teleoperation_node
    
    # 3) Run this test script
    python3 test_vr_action_follow.py [--direction DIRECTION] [--duration DURATION] [--rate RATE]

Test directions:
    - x_forward: Move forward in X direction (+X)
    - x_backward: Move backward in X direction (-X)
    - y_left: Move left in Y direction (+Y)
    - y_right: Move right in Y direction (-Y)
    - z_up: Move up in Z direction (+Z)
    - z_down: Move down in Z direction (-Z)
    - all: Test all directions sequentially
"""

import argparse
import time
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from threading import Lock

from role_ros2.msg import VRPolicyAction, PolymetisRobotState


class VRActionFollowTester(Node):
    """Test if robot follows VR actions by publishing actions and monitoring robot state."""
    
    def __init__(self, direction="x_forward", rate=15.0, duration=5.0):
        super().__init__("vr_action_follow_tester")
        
        self.direction = direction
        self.rate = rate
        self.duration = duration
        self.dt = 1.0 / rate
        
        # Publisher for fake VR actions
        self.action_pub = self.create_publisher(
            VRPolicyAction,
            "vr_policy/action",
            10
        )
        
        # Subscriber for robot state
        self.robot_state_sub = self.create_subscription(
            PolymetisRobotState,
            "polymetis/robot_state",
            self._robot_state_callback,
            10
        )
        
        # State tracking
        self._state_lock = Lock()
        self.initial_ee_pos = None
        self.current_ee_pos = None
        self.initial_ee_euler = None
        self.current_ee_euler = None
        self.robot_state_received = False
        self.start_time = None
        self.step_count = 0
        self.ee_positions = []  # Store EE positions over time
        self.ee_timestamps = []
        
        # Timer for publishing actions
        self.timer = self.create_timer(self.dt, self._publish_and_check)
        
        self.get_logger().info("=" * 70)
        self.get_logger().info("VR Action Follow Test")
        self.get_logger().info("=" * 70)
        self.get_logger().info(f"  Direction: {direction}")
        self.get_logger().info(f"  Duration: {duration} seconds")
        self.get_logger().info(f"  Rate: {rate} Hz")
        self.get_logger().info("=" * 70)
        self.get_logger().info("Waiting for robot state...")
    
    def _robot_state_callback(self, msg: PolymetisRobotState):
        """Callback for robot state messages."""
        with self._state_lock:
            if len(msg.ee_position) >= 3 and len(msg.ee_euler) >= 3:
                self.current_ee_pos = np.array([
                    msg.ee_position[0],
                    msg.ee_position[1],
                    msg.ee_position[2]
                ])
                self.current_ee_euler = np.array([
                    msg.ee_euler[0],
                    msg.ee_euler[1],
                    msg.ee_euler[2]
                ])
                
                # Record initial position on first received state
                if self.initial_ee_pos is None:
                    self.initial_ee_pos = self.current_ee_pos.copy()
                    self.initial_ee_euler = self.current_ee_euler.copy()
                    self.robot_state_received = True
                    self.get_logger().info(
                        f"✅ Robot state received!\n"
                        f"   Initial EE position: {self.initial_ee_pos}\n"
                        f"   Initial EE euler: {self.initial_ee_euler}"
                    )
                
                # Record position history
                self.ee_positions.append(self.current_ee_pos.copy())
                self.ee_timestamps.append(time.time())
    
    def _get_action_for_direction(self, direction):
        """
        Get action vector for specified direction.
        
        NOTE: Actions are NORMALIZED velocities (-1.0 to 1.0), not actual velocities (m/s).
        The IK solver will convert these to actual deltas using max_lin_delta (0.075 m) and max_rot_delta (0.15 rad).
        At 15 Hz, max velocity = 0.075 * 15 = 1.125 m/s for linear, 0.15 * 15 = 2.25 rad/s for rotation.
        
        For testing, we use moderate values (0.3-0.5) to get noticeable but safe movement.
        """
        if direction == "x_forward":
            return np.array([0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # +X (normalized, 50% max)
        elif direction == "x_backward":
            return np.array([-0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # -X (normalized, 50% max)
        elif direction == "y_left":
            return np.array([0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0])  # +Y (normalized, 50% max)
        elif direction == "y_right":
            return np.array([0.0, -0.5, 0.0, 0.0, 0.0, 0.0, 0.0])  # -Y (normalized, 50% max)
        elif direction == "z_up":
            return np.array([0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.0])  # +Z (normalized, 50% max)
        elif direction == "z_down":
            return np.array([0.0, 0.0, -0.5, 0.0, 0.0, 0.0, 0.0])  # -Z (normalized, 50% max)
        else:
            return np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    
    def _publish_and_check(self):
        """Publish action and check robot response."""
        if not self.robot_state_received:
            return
        
        if self.start_time is None:
            self.start_time = time.time()
            self.get_logger().info("=" * 70)
            self.get_logger().info("▶️  Starting action test...")
            self.get_logger().info("=" * 70)
        
        elapsed = time.time() - self.start_time
        if elapsed > self.duration:
            # Test complete - analyze results
            self._analyze_results()
            self.timer.cancel()
            return
        
        # Get action for current direction
        if self.direction == "all":
            # Cycle through directions
            cycle_time = self.duration / 6
            cycle = int(elapsed / cycle_time) % 6
            directions = ["x_forward", "x_backward", "y_left", "y_right", "z_up", "z_down"]
            current_dir = directions[cycle]
            action = self._get_action_for_direction(current_dir)
        else:
            action = self._get_action_for_direction(self.direction)
        
        # Publish action
        msg = VRPolicyAction()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.action = action.tolist()
        msg.movement_enabled = True
        msg.controller_on = True
        msg.success = False
        msg.failure = False
        msg.target_cartesian_position = [0.0] * 6
        msg.target_gripper_position = 0.0
        
        self.action_pub.publish(msg)
        self.step_count += 1
        
        # Log periodically
        if self.step_count % int(self.rate) == 0:  # Once per second
            with self._state_lock:
                if self.current_ee_pos is not None and self.initial_ee_pos is not None:
                    displacement = self.current_ee_pos - self.initial_ee_pos
                    # Calculate expected displacement based on action
                    expected_displacement = action[:3] * elapsed
                    direction_match = np.dot(
                        action[:3] / (np.linalg.norm(action[:3]) + 1e-9),
                        displacement / (np.linalg.norm(displacement) + 1e-9)
                    )
                    self.get_logger().info(
                        f"📊 Step {self.step_count} (t={elapsed:.1f}s):\n"
                        f"   Action (velocity): {action[:3]} (norm: {np.linalg.norm(action[:3]):.4f})\n"
                        f"   Expected displacement: {expected_displacement}\n"
                        f"   Actual displacement: {displacement} (norm: {np.linalg.norm(displacement):.4f})\n"
                        f"   Direction match: {direction_match:.3f}\n"
                        f"   EE position: {self.current_ee_pos}"
                    )
    
    def _analyze_results(self):
        """Analyze test results and verify robot followed actions."""
        self.get_logger().info("=" * 70)
        self.get_logger().info("📊 Test Results Analysis")
        self.get_logger().info("=" * 70)
        
        if len(self.ee_positions) < 2:
            self.get_logger().error("❌ Not enough data collected!")
            return
        
        with self._state_lock:
            if self.initial_ee_pos is None or self.current_ee_pos is None:
                self.get_logger().error("❌ Missing initial or final position!")
                return
            
            # Calculate total displacement
            total_displacement = self.current_ee_pos - self.initial_ee_pos
            total_distance = np.linalg.norm(total_displacement)
            
            self.get_logger().info(f"Initial EE position: {self.initial_ee_pos}")
            self.get_logger().info(f"Final EE position: {self.current_ee_pos}")
            self.get_logger().info(f"Total displacement: {total_displacement}")
            self.get_logger().info(f"Total distance moved: {total_distance:.4f} m")
            
            # Expected displacement based on action
            if self.direction == "all":
                self.get_logger().info("Testing all directions - checking each cycle...")
            else:
                action = self._get_action_for_direction(self.direction)
                expected_velocity = action[:3]
                expected_displacement = expected_velocity * self.duration
                
                self.get_logger().info("=" * 70)
                self.get_logger().info("Expected vs Actual:")
                self.get_logger().info(f"  Expected velocity: {expected_velocity}")
                self.get_logger().info(f"  Expected displacement: {expected_displacement}")
                self.get_logger().info(f"  Actual displacement: {total_displacement}")
                
                # Check if direction matches
                if np.linalg.norm(expected_velocity) > 0.001:
                    expected_dir = expected_velocity / np.linalg.norm(expected_velocity)
                    actual_dir = total_displacement / (np.linalg.norm(total_displacement) + 1e-9)
                    direction_match = np.dot(expected_dir, actual_dir)
                    
                    self.get_logger().info(f"  Direction match (dot product): {direction_match:.3f}")
                    
                    if direction_match > 0.7:  # Roughly same direction
                        self.get_logger().info("✅ Direction matches expected!")
                    else:
                        self.get_logger().warn("⚠️  Direction does not match expected!")
                
                # Check magnitude
                # Expected displacement calculation:
                # - Action is normalized velocity (-1 to 1)
                # - IK solver converts to delta: delta = velocity * max_lin_delta (0.075 m)
                # - At 15 Hz, each step = 1/15 s, so velocity = delta * 15
                # - Over duration, expected_displacement = velocity * max_lin_delta * duration * control_hz
                #   = action * max_lin_delta * duration * control_hz
                max_lin_delta = 0.075  # From robot_ik_solver.py
                control_hz = 15.0  # From robot_ik_solver.py
                expected_displacement_calc = expected_velocity * max_lin_delta * self.duration * control_hz
                expected_magnitude = np.linalg.norm(expected_displacement_calc)
                actual_magnitude = total_distance
                magnitude_ratio = actual_magnitude / (expected_magnitude + 1e-9)
                
                self.get_logger().info(f"  Expected magnitude (calc): {expected_magnitude:.4f} m")
                self.get_logger().info(f"    (action={expected_velocity}, max_lin_delta={max_lin_delta}, duration={self.duration}, hz={control_hz})")
                self.get_logger().info(f"  Actual magnitude: {actual_magnitude:.4f} m")
                self.get_logger().info(f"  Magnitude ratio: {magnitude_ratio:.3f}")
                
                if 0.3 < magnitude_ratio < 2.0:  # Within reasonable range
                    self.get_logger().info("✅ Magnitude is reasonable!")
                else:
                    self.get_logger().warn("⚠️  Magnitude seems incorrect!")
            
            # Analyze movement over time
            if len(self.ee_positions) > 10:
                positions_array = np.array(self.ee_positions)
                # Calculate velocity from position changes
                velocities = []
                for i in range(1, len(positions_array)):
                    dt = self.ee_timestamps[i] - self.ee_timestamps[i-1]
                    if dt > 0:
                        vel = (positions_array[i] - positions_array[i-1]) / dt
                        velocities.append(vel)
                
                if velocities:
                    avg_velocity = np.mean(velocities, axis=0)
                    self.get_logger().info(f"Average velocity: {avg_velocity}")
            
            self.get_logger().info("=" * 70)


def main():
    parser = argparse.ArgumentParser(
        description="Test if robot follows VR actions"
    )
    parser.add_argument(
        "--direction",
        type=str,
        default="x_forward",
        choices=["x_forward", "x_backward", "y_left", "y_right", "z_up", "z_down", "all"],
        help="Movement direction (default: x_forward)"
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=5.0,
        help="Test duration in seconds (default: 5.0)"
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=15.0,
        help="Publish rate in Hz (default: 15.0)"
    )
    
    args = parser.parse_args()
    
    print("=" * 70)
    print("VR Action Follow Test")
    print("=" * 70)
    print(f"Direction: {args.direction}")
    print(f"Duration: {args.duration} seconds")
    print(f"Rate: {args.rate} Hz")
    print("=" * 70)
    print("\n⚠️  This will send commands to the robot!")
    print("   Make sure teleoperation_node is running and robot has space.")
    print("\nStarting in 3 seconds...")
    time.sleep(3)
    
    rclpy.init()
    node = VRActionFollowTester(
        direction=args.direction,
        rate=args.rate,
        duration=args.duration
    )
    
    try:
        # Wait for robot state
        timeout = 10.0
        start_wait = time.time()
        while not node.robot_state_received:
            rclpy.spin_once(node, timeout_sec=0.1)
            if time.time() - start_wait > timeout:
                node.get_logger().error("❌ Timeout waiting for robot state!")
                break
        
        if node.robot_state_received:
            # Run test
            node.get_logger().info("Starting test...")
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.1)
                if node.timer.is_canceled():
                    break
        else:
            node.get_logger().error("Cannot proceed without robot state!")
        
    except KeyboardInterrupt:
        print("\n\n⚠️  Test interrupted by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

