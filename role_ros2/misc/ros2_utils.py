"""Utility functions for role-ros2."""

import time as python_time

import numpy as np
import rclpy
from rclpy.node import Node


def get_ros_time_ns(node: Node) -> int:
    """
    Get current ROS time in nanoseconds from a node.
    
    Args:
        node: ROS2 node with clock
    
    Returns:
        Current ROS time in nanoseconds
    """
    if node is not None and hasattr(node, 'get_clock'):
        return node.get_clock().now().nanoseconds
    else:
        # Fallback to system time
        return python_time.time_ns()


def get_ros_time_ms(node: Node) -> int:
    """
    Get current ROS time in milliseconds from a node.
    
    Args:
        node: ROS2 node with clock
    
    Returns:
        Current ROS time in milliseconds
    """
    return get_ros_time_ns(node) // 1_000_000


def ros_time_to_ns(ros_time: rclpy.time.Time) -> int:
    """
    Convert ROS time to nanoseconds.
    
    Args:
        ros_time: ROS2 time object
        
    Returns:
        Time in nanoseconds (integer)
    """
    return int(ros_time.nanoseconds)


def quaternion_to_rotation_matrix(quaternion) -> np.ndarray:
    """
    Convert quaternion to 3x3 rotation matrix.
    
    Args:
        quaternion: Quaternion object with x, y, z, w attributes
                   or tuple/list (x, y, z, w)
        
    Returns:
        3x3 rotation matrix as numpy array
    """
    if hasattr(quaternion, 'x'):
        # ROS2 geometry_msgs.msg.Quaternion
        x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w
    else:
        # Tuple or list
        x, y, z, w = quaternion
    
    # Normalize quaternion
    norm = np.sqrt(x*x + y*y + z*z + w*w)
    if norm > 0:
        x, y, z, w = x/norm, y/norm, z/norm, w/norm
    
    # Convert to rotation matrix
    R = np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)]
    ])
    
    return R


def transform_to_matrix(transform) -> np.ndarray:
    """
    Convert ROS2 TransformStamped or Transform to 4x4 transformation matrix.
    
    Args:
        transform: ROS2 TransformStamped or Transform message
                   Must have transform.translation and transform.rotation attributes
        
    Returns:
        4x4 transformation matrix as numpy array
    """
    # Extract translation
    t = transform.transform.translation
    translation = np.array([t.x, t.y, t.z])
    
    # Extract rotation and convert to matrix
    rotation_matrix = quaternion_to_rotation_matrix(transform.transform.rotation)
    
    # Build 4x4 transformation matrix
    transform_matrix = np.eye(4)
    transform_matrix[:3, :3] = rotation_matrix
    transform_matrix[:3, 3] = translation
    
    return transform_matrix

