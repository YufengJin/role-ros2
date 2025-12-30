#!/usr/bin/env python3
"""
Test script for teleoperation_node using fake VRPolicyAction messages.

This script publishes fake VRPolicyAction messages to test if teleoperation_node
can correctly receive and execute actions.

Usage:
    # 1) Start robot bridge (mock or real)
    ros2 run role_ros2 polymetis_bridge_node --ros-args -p use_mock:=true
    
    # 2) Start teleoperation_node
    ros2 run role_ros2 teleoperation_node
    
    # 3) Run this test script
    python3 test_teleoperation_fake.py [--mode MODE] [--duration DURATION] [--rate RATE]

Test modes:
    - zero: Zero action (no movement)
    - small_x: Small movement in X direction (0.02 m/s)
    - small_y: Small movement in Y direction (0.02 m/s)
    - small_z: Small movement in Z direction (0.02 m/s)
    - big_x: Big movement in X direction (0.05 m/s)
    - circle: Circular motion (X and Y)
    - gripper: Gripper open/close test
    - all: Test all movements sequentially
"""

import argparse
import time
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header

from role_ros2.msg import VRPolicyAction


class FakeVRPolicyPublisher(Node):
    """Publishes fake VRPolicyAction messages for testing."""
    
    def __init__(self, mode="zero", rate=15.0):
        super().__init__("fake_vr_policy_publisher")
        
        self.mode = mode
        self.rate = rate
        self.dt = 1.0 / rate
        
        # Publisher
        self.action_pub = self.create_publisher(
            VRPolicyAction,
            "vr_policy/action",
            10
        )
        
        # Timer for publishing
        self.timer = self.create_timer(self.dt, self.publish_action)
        
        # State tracking
        self.start_time = time.time()
        self.step_count = 0
        
        self.get_logger().info(
            f"Fake VR Policy Publisher started: mode={mode}, rate={rate} Hz"
        )
    
    def publish_action(self):
        """Publish fake action based on mode."""
        msg = VRPolicyAction()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        
        # Get action based on mode
        action = self._get_action()
        
        msg.action = action.tolist()
        msg.movement_enabled = True  # Always enabled for testing
        msg.controller_on = True
        msg.success = False
        msg.failure = False
        msg.target_cartesian_position = [0.0] * 6
        msg.target_gripper_position = 0.0
        
        self.action_pub.publish(msg)
        
        # Log periodically
        if self.step_count % int(self.rate) == 0:  # Log once per second
            self.get_logger().info(
                f"Published action: {[f'{x:.3f}' for x in action[:3]]}... "
                f"(mode={self.mode}, step={self.step_count})"
            )
        
        self.step_count += 1
    
    def _get_action(self):
        """Get action vector based on current mode and time."""
        t = time.time() - self.start_time
        
        if self.mode == "zero":
            # Zero action
            return np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        
        elif self.mode == "small_x":
            # Small movement in X: 0.02 m/s
            return np.array([0.02, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        
        elif self.mode == "small_y":
            # Small movement in Y: 0.02 m/s
            return np.array([0.0, 0.02, 0.0, 0.0, 0.0, 0.0, 0.0])
        
        elif self.mode == "small_z":
            # Small movement in Z: 0.02 m/s
            return np.array([0.0, 0.0, 0.02, 0.0, 0.0, 0.0, 0.0])
        
        elif self.mode == "big_x":
            # Big movement in X: 0.05 m/s
            return np.array([0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        
        elif self.mode == "circle":
            # Circular motion: X and Y with sine/cosine
            radius = 0.03  # m/s
            period = 5.0  # seconds
            vx = radius * np.cos(2 * np.pi * t / period)
            vy = radius * np.sin(2 * np.pi * t / period)
            return np.array([vx, vy, 0.0, 0.0, 0.0, 0.0, 0.0])
        
        elif self.mode == "gripper":
            # Gripper open/close: sine wave
            period = 3.0  # seconds
            gripper_vel = 0.3 * np.sin(2 * np.pi * t / period)
            return np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, gripper_vel])
        
        elif self.mode == "all":
            # Sequential test: cycle through different movements
            cycle_time = 5.0  # seconds per movement
            cycle = int(t / cycle_time) % 6
            
            if cycle == 0:
                # Zero
                return np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            elif cycle == 1:
                # Small X
                return np.array([0.02, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            elif cycle == 2:
                # Small Y
                return np.array([0.0, 0.02, 0.0, 0.0, 0.0, 0.0, 0.0])
            elif cycle == 3:
                # Small Z
                return np.array([0.0, 0.0, 0.02, 0.0, 0.0, 0.0, 0.0])
            elif cycle == 4:
                # Big X
                return np.array([0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            elif cycle == 5:
                # Gripper
                local_t = t % cycle_time
                gripper_vel = 0.3 * np.sin(2 * np.pi * local_t / 2.0)
                return np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, gripper_vel])
        
        # Default: zero
        return np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])


def main():
    parser = argparse.ArgumentParser(
        description="Test teleoperation_node with fake VRPolicyAction messages"
    )
    parser.add_argument(
        "--mode",
        type=str,
        default="zero",
        choices=["zero", "small_x", "small_y", "small_z", "big_x", "circle", "gripper", "all"],
        help="Test mode (default: zero)"
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=10.0,
        help="Test duration in seconds (default: 10.0)"
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=15.0,
        help="Publish rate in Hz (default: 15.0)"
    )
    
    args = parser.parse_args()
    
    print("=" * 70)
    print("Fake VR Policy Action Publisher")
    print("=" * 70)
    print(f"Mode: {args.mode}")
    print(f"Duration: {args.duration} seconds")
    print(f"Rate: {args.rate} Hz")
    print("=" * 70)
    print("\n⚠️  This will send commands to the robot!")
    print("   Make sure teleoperation_node is running and robot has space.")
    print("\nStarting in 3 seconds...")
    time.sleep(3)
    
    rclpy.init()
    node = FakeVRPolicyPublisher(mode=args.mode, rate=args.rate)
    
    try:
        # Run for specified duration
        start_time = time.time()
        while time.time() - start_time < args.duration:
            rclpy.spin_once(node, timeout_sec=0.1)
            time.sleep(0.01)  # Small sleep to avoid busy waiting
        
        print(f"\n✅ Test completed after {args.duration} seconds")
        print(f"   Published {node.step_count} messages")
        
    except KeyboardInterrupt:
        print("\n\n⚠️  Test interrupted by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

