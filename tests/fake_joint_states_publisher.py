#!/usr/bin/env python3
"""
Fake Joint States Publisher for testing without real robot.

Publishes fake joint_states to /joint_states topic for testing robot_state_publisher and TF.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
import time


class FakeJointStatesPublisher(Node):
    """Publish fake joint states for testing."""
    
    def __init__(self, arm_id='fr3', publish_rate=50.0):
        super().__init__('fake_joint_states_publisher')
        
        self.arm_id = arm_id
        self.publish_rate = publish_rate
        
        # Joint names based on arm_id
        if arm_id == 'fr3':
            self.arm_joint_names = [
                'fr3_panda_joint1', 'fr3_panda_joint2', 'fr3_panda_joint3', 'fr3_panda_joint4',
                'fr3_panda_joint5', 'fr3_panda_joint6', 'fr3_panda_joint7'
            ]
            self.gripper_joint_names = [
                'fr3_panda_finger_joint1', 'fr3_panda_finger_joint2'
            ]
        else:  # panda
            self.arm_joint_names = [
                'panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4',
                'panda_joint5', 'panda_joint6', 'panda_joint7'
            ]
            self.gripper_joint_names = [
                'panda_finger_joint1', 'panda_finger_joint2'
            ]
        
        self.all_joint_names = self.arm_joint_names + self.gripper_joint_names
        
        # Publisher
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        
        # Timer
        timer_period = 1.0 / publish_rate
        self.timer = self.create_timer(timer_period, self.publish_joint_states)
        
        # Animation state
        self.animation_time = 0.0
        
        # Home position (slightly modified for visibility)
        self.home_positions = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
        
        self.get_logger().info(f'FakeJointStatesPublisher started for {arm_id}')
        self.get_logger().info(f'  Publishing to: /joint_states at {publish_rate} Hz')
        self.get_logger().info(f'  Joint names: {self.all_joint_names}')
    
    def publish_joint_states(self):
        """Publish animated joint states."""
        # Animate joints slightly for visual feedback
        self.animation_time += 1.0 / self.publish_rate
        
        # Generate smooth motion for demonstration
        # Home position with slight oscillation
        animated_positions = [
            self.home_positions[i] + 0.1 * math.sin(self.animation_time * 0.5 + i * 0.5)
            for i in range(7)
        ]
        
        # Gripper width (oscillate between open and closed)
        gripper_width = 0.02 + 0.02 * math.sin(self.animation_time * 0.3)
        finger1_position = gripper_width / 2.0
        finger2_position = gripper_width / 2.0
        
        # Create message
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ''
        msg.name = self.all_joint_names
        msg.position = animated_positions + [finger1_position, finger2_position]
        msg.velocity = [0.0] * len(self.all_joint_names)
        msg.effort = [0.0] * len(self.all_joint_names)
        
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    
    # Get arm_id from parameter or default
    node = Node('fake_joint_states_publisher_main')
    node.declare_parameter('arm_id', 'fr3')
    node.declare_parameter('publish_rate', 50.0)
    arm_id = node.get_parameter('arm_id').get_parameter_value().string_value
    publish_rate = node.get_parameter('publish_rate').get_parameter_value().double_value
    node.destroy_node()
    
    # Create publisher node
    publisher_node = FakeJointStatesPublisher(arm_id=arm_id, publish_rate=publish_rate)
    
    try:
        rclpy.spin(publisher_node)
    except KeyboardInterrupt:
        pass
    finally:
        publisher_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

