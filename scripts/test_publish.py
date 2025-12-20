#!/usr/bin/env python3
"""
Simple ROS2 Publisher Test Script
Publishes a string message to /test_topic for cross-container communication testing.
"""

import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TestPublisher(Node):
    def __init__(self):
        super().__init__('test_publisher')
        self.publisher = self.create_publisher(String, '/test_topic', 10)
        self.timer = self.create_timer(1.0, self.publish_callback)
        self.counter = 0
        
    def publish_callback(self):
        msg = String()
        msg.data = f'Hello ROS2! Message #{self.counter}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Published: "{msg.data}"')
        self.counter += 1


def main(args=None):
    rclpy.init(args=args)
    node = TestPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
