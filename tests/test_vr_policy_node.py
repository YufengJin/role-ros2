#!/usr/bin/env python3
"""
Test script for vr_policy_node

This script tests the functionality of vr_policy_node by:
1. Publishing mock Oculus controller data
2. Publishing mock robot state
3. Verifying that vr_policy_node computes and publishes actions

Usage:
    # In one terminal, start vr_policy_node:
    ros2 run role_ros2 vr_policy_node --ros-args -p right_controller:=true
    
    # In another terminal, run this test:
    python3 tests/test_vr_policy_node.py
"""

import sys
import time
import numpy as np
from pathlib import Path

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from role_ros2.msg import OculusButtons, VRPolicyAction, PolymetisRobotState


class MockOculusPublisher(Node):
    """Mock publisher for Oculus controller data."""
    
    def __init__(self):
        super().__init__('mock_oculus_publisher')
        
        # Use BEST_EFFORT QoS to match oculus_reader_node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        qos_profile_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        self.right_pose_pub = self.create_publisher(
            PoseStamped, 'oculus/right_controller/pose', qos_profile_best_effort
        )
        self.buttons_pub = self.create_publisher(
            OculusButtons, 'oculus/buttons', qos_profile_best_effort
        )
        
        self.timer = self.create_timer(0.02, self.publish_data)  # 50 Hz
        self.start_time = time.time()
        self.get_logger().info("Mock Oculus Publisher started")
    
    def publish_data(self):
        """Publish mock controller data."""
        current_time = self.get_clock().now()
        elapsed = time.time() - self.start_time
        
        # Simulate controller movement (circular motion)
        radius = 0.1  # 10 cm
        angle = elapsed * 0.5  # Slow rotation
        
        # Publish right controller pose
        pose_msg = PoseStamped()
        pose_msg.header.stamp = current_time.to_msg()
        pose_msg.header.frame_id = "oculus_base"
        pose_msg.pose.position.x = radius * np.cos(angle)
        pose_msg.pose.position.y = radius * np.sin(angle)
        pose_msg.pose.position.z = 0.5
        
        # Simple quaternion (no rotation)
        pose_msg.pose.orientation.w = 1.0
        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = 0.0
        
        self.right_pose_pub.publish(pose_msg)
        
        # Publish buttons (simulate grip pressed after 2 seconds)
        buttons_msg = OculusButtons()
        buttons_msg.header.stamp = current_time.to_msg()
        buttons_msg.header.frame_id = "oculus_base"
        
        # Simulate grip pressed after 2 seconds
        if elapsed > 2.0:
            buttons_msg.right_grip_pressed = True
            buttons_msg.right_trigger_value = 0.5  # Half trigger
        else:
            buttons_msg.right_grip_pressed = False
            buttons_msg.right_trigger_value = 0.0
        
        buttons_msg.right_joystick_pressed = False
        buttons_msg.a = False
        buttons_msg.b = False
        buttons_msg.x = False
        buttons_msg.y = False
        
        # Left controller (not used, but set defaults)
        buttons_msg.left_grip_pressed = False
        buttons_msg.left_trigger_value = 0.0
        buttons_msg.left_joystick_pressed = False
        
        self.buttons_pub.publish(buttons_msg)


class MockRobotStatePublisher(Node):
    """Mock publisher for robot state."""
    
    def __init__(self):
        super().__init__('mock_robot_state_publisher')
        
        # Use RELIABLE QoS to match polymetis_bridge_node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        qos_profile_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        self.robot_state_pub = self.create_publisher(
            PolymetisRobotState, 'polymetis/robot_state', qos_profile_reliable
        )
        
        self.timer = self.create_timer(0.1, self.publish_state)  # 10 Hz
        self.get_logger().info("Mock Robot State Publisher started")
    
    def publish_state(self):
        """Publish mock robot state."""
        state_msg = PolymetisRobotState()
        state_msg.header.stamp = self.get_clock().now().to_msg()
        state_msg.header.frame_id = "base_link"
        
        # Set joint positions (7 DOF)
        state_msg.joint_positions = [0.0] * 7
        
        # Set joint velocities
        state_msg.joint_velocities = [0.0] * 7
        
        # Set cartesian position [x, y, z, roll, pitch, yaw]
        state_msg.cartesian_position = [0.3, 0.0, 0.5, 0.0, 0.0, 0.0]
        
        # Set gripper position (normalized 0.0-1.0)
        state_msg.gripper_position = 0.5
        
        self.robot_state_pub.publish(state_msg)


class VRPolicyActionSubscriber(Node):
    """Subscriber to verify VR policy actions."""
    
    def __init__(self):
        super().__init__('vr_policy_action_subscriber')
        
        self.action_sub = self.create_subscription(
            VRPolicyAction, 'vr_policy/action', self.action_callback, 10
        )
        
        self.received_actions = []
        self.get_logger().info("VR Policy Action Subscriber started")
    
    def action_callback(self, msg: VRPolicyAction):
        """Callback for action messages."""
        self.received_actions.append({
            'timestamp': time.time(),
            'action': msg.action,
            'movement_enabled': msg.movement_enabled,
            'controller_on': msg.controller_on,
            'success': msg.success,
            'failure': msg.failure,
        })
        
        self.get_logger().info(
            f"Received action: movement_enabled={msg.movement_enabled}, "
            f"controller_on={msg.controller_on}, "
            f"action={[f'{x:.3f}' for x in msg.action[:3]]}..."
        )


def test_vr_policy_node():
    """Test vr_policy_node functionality."""
    print("=" * 70)
    print("Testing vr_policy_node")
    print("=" * 70)
    
    rclpy.init()
    
    # Create mock publishers and subscriber
    mock_oculus = MockOculusPublisher()
    mock_robot = MockRobotStatePublisher()
    action_sub = VRPolicyActionSubscriber()
    
    # Create executor
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(mock_oculus)
    executor.add_node(mock_robot)
    executor.add_node(action_sub)
    
    print("\n[INFO] Starting test...")
    print("[INFO] Make sure vr_policy_node is running in another terminal:")
    print("       ros2 run role_ros2 vr_policy_node --ros-args -p right_controller:=true")
    print("\n[INFO] Test will run for 10 seconds...")
    print("[INFO] After 2 seconds, grip will be 'pressed' to enable movement\n")
    
    # Run for 10 seconds
    start_time = time.time()
    timeout = 10.0
    
    try:
        while time.time() - start_time < timeout:
            executor.spin_once(timeout_sec=0.1)
            elapsed = time.time() - start_time
            
            # Print status every 2 seconds
            if int(elapsed) % 2 == 0 and elapsed > 0.1:
                print(f"[INFO] Test running... {elapsed:.1f}s elapsed, "
                      f"actions received: {len(action_sub.received_actions)}")
                time.sleep(0.1)  # Avoid duplicate prints
    
    except KeyboardInterrupt:
        print("\n[INFO] Test interrupted by user")
    
    finally:
        # Results
        print("\n" + "=" * 70)
        print("Test Results")
        print("=" * 70)
        print(f"Total actions received: {len(action_sub.received_actions)}")
        
        if len(action_sub.received_actions) > 0:
            print("\n✅ SUCCESS: vr_policy_node is publishing actions!")
            
            # Analyze actions
            enabled_actions = [a for a in action_sub.received_actions if a['movement_enabled']]
            print(f"   - Actions with movement enabled: {len(enabled_actions)}")
            print(f"   - Actions with controller on: {sum(1 for a in action_sub.received_actions if a['controller_on'])}")
            
            if len(enabled_actions) > 0:
                sample_action = enabled_actions[0]
                print(f"\n   Sample action (movement enabled):")
                print(f"     - Action vector: {sample_action['action']}")
                print(f"     - Controller on: {sample_action['controller_on']}")
        else:
            print("\n❌ FAILURE: No actions received from vr_policy_node")
            print("   Possible issues:")
            print("   1. vr_policy_node is not running")
            print("   2. QoS mismatch (check if fixed)")
            print("   3. Topic names don't match")
        
        # Cleanup
        mock_oculus.destroy_node()
        mock_robot.destroy_node()
        action_sub.destroy_node()
        rclpy.shutdown()
        
        print("\n" + "=" * 70)


if __name__ == "__main__":
    test_vr_policy_node()

