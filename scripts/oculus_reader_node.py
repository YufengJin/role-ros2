#!/usr/bin/env python3
# Copyright 2025 DROID Team
#
# Licensed under the Apache License, Version 2.0 (the 'License');
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an 'AS IS' BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

import numpy as np
import time
import threading

from geometry_msgs.msg import PoseStamped, TransformStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray
from role_ros2.msg import OculusButtons
from tf2_ros import TransformBroadcaster

try:
    from oculus_reader.reader import OculusReader
except ImportError:
    print("WARNING: oculus_reader package not found. Please install it:")
    print("  pip install -e /path/to/droid/droid/oculus_reader")
    OculusReader = None


def transformation_matrix_to_pose(transform_matrix):
    """Convert 4x4 transformation matrix to ROS2 Pose message"""
    # Extract position (translation)
    position = Point()
    position.x = float(transform_matrix[0, 3])
    position.y = float(transform_matrix[1, 3])
    position.z = float(transform_matrix[2, 3])
    
    # Extract rotation (convert rotation matrix to quaternion)
    rotation_matrix = transform_matrix[:3, :3]
    quat = rotation_matrix_to_quaternion(rotation_matrix)
    
    pose = Pose()
    pose.position = position
    pose.orientation = quat
    
    return pose


def rotation_matrix_to_quaternion(rot_matrix):
    """Convert 3x3 rotation matrix to quaternion (w, x, y, z)"""
    from geometry_msgs.msg import Quaternion
    
    # Use scipy or manual conversion
    # Trace of rotation matrix
    trace = np.trace(rot_matrix)
    
    if trace > 0:
        s = np.sqrt(trace + 1.0) * 2  # s = 4 * qw
        w = 0.25 * s
        x = (rot_matrix[2, 1] - rot_matrix[1, 2]) / s
        y = (rot_matrix[0, 2] - rot_matrix[2, 0]) / s
        z = (rot_matrix[1, 0] - rot_matrix[0, 1]) / s
    else:
        if rot_matrix[0, 0] > rot_matrix[1, 1] and rot_matrix[0, 0] > rot_matrix[2, 2]:
            s = np.sqrt(1.0 + rot_matrix[0, 0] - rot_matrix[1, 1] - rot_matrix[2, 2]) * 2
            w = (rot_matrix[2, 1] - rot_matrix[1, 2]) / s
            x = 0.25 * s
            y = (rot_matrix[0, 1] + rot_matrix[1, 0]) / s
            z = (rot_matrix[0, 2] + rot_matrix[2, 0]) / s
        elif rot_matrix[1, 1] > rot_matrix[2, 2]:
            s = np.sqrt(1.0 + rot_matrix[1, 1] - rot_matrix[0, 0] - rot_matrix[2, 2]) * 2
            w = (rot_matrix[0, 2] - rot_matrix[2, 0]) / s
            x = (rot_matrix[0, 1] + rot_matrix[1, 0]) / s
            y = 0.25 * s
            z = (rot_matrix[1, 2] + rot_matrix[2, 1]) / s
        else:
            s = np.sqrt(1.0 + rot_matrix[2, 2] - rot_matrix[0, 0] - rot_matrix[1, 1]) * 2
            w = (rot_matrix[1, 0] - rot_matrix[0, 1]) / s
            x = (rot_matrix[0, 2] + rot_matrix[2, 0]) / s
            y = (rot_matrix[1, 2] + rot_matrix[2, 1]) / s
            z = 0.25 * s
    
    quat = Quaternion()
    quat.w = float(w)
    quat.x = float(x)
    quat.y = float(y)
    quat.z = float(z)
    
    return quat


def parse_buttons_to_msg(buttons_dict):
    """Convert buttons dictionary to OculusButtons message"""
    from std_msgs.msg import Header
    
    msg = OculusButtons()
    msg.header = Header()  # Initialize header
    
    # Boolean buttons
    msg.a = buttons_dict.get('A', False)
    msg.b = buttons_dict.get('B', False)
    msg.x = buttons_dict.get('X', False)
    msg.y = buttons_dict.get('Y', False)
    msg.right_thumb_up = buttons_dict.get('RThU', False)
    msg.left_thumb_up = buttons_dict.get('LThU', False)
    msg.right_joystick_pressed = buttons_dict.get('RJ', False)
    msg.left_joystick_pressed = buttons_dict.get('LJ', False)
    msg.right_grip_pressed = buttons_dict.get('RG', False)
    msg.left_grip_pressed = buttons_dict.get('LG', False)
    msg.right_trigger_pressed = buttons_dict.get('RTr', False)
    msg.left_trigger_pressed = buttons_dict.get('LTr', False)
    
    # Analog values
    right_grip = buttons_dict.get('rightGrip', (0.0,))
    left_grip = buttons_dict.get('leftGrip', (0.0,))
    right_trig = buttons_dict.get('rightTrig', (0.0,))
    left_trig = buttons_dict.get('leftTrig', (0.0,))
    
    msg.right_grip_value = float(right_grip[0]) if isinstance(right_grip, tuple) and len(right_grip) > 0 else 0.0
    msg.left_grip_value = float(left_grip[0]) if isinstance(left_grip, tuple) and len(left_grip) > 0 else 0.0
    msg.right_trigger_value = float(right_trig[0]) if isinstance(right_trig, tuple) and len(right_trig) > 0 else 0.0
    msg.left_trigger_value = float(left_trig[0]) if isinstance(left_trig, tuple) and len(left_trig) > 0 else 0.0
    
    # Joystick positions
    right_js = buttons_dict.get('rightJS', (0.0, 0.0))
    left_js = buttons_dict.get('leftJS', (0.0, 0.0))
    
    if isinstance(right_js, tuple) and len(right_js) >= 2:
        msg.right_joystick_x = float(right_js[0])
        msg.right_joystick_y = float(right_js[1])
    else:
        msg.right_joystick_x = 0.0
        msg.right_joystick_y = 0.0
    
    if isinstance(left_js, tuple) and len(left_js) >= 2:
        msg.left_joystick_x = float(left_js[0])
        msg.left_joystick_y = float(left_js[1])
    else:
        msg.left_joystick_x = 0.0
        msg.left_joystick_y = 0.0
    
    return msg


class OculusReaderNode(Node):
    """ROS2 Node for publishing Oculus Quest controller data"""
    
    def __init__(self):
        super().__init__('oculus_reader_node')
        
        # Declare parameters
        self.declare_parameter('publish_rate', 50.0)  # Hz
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('publish_markers', False)  # Publish visualization markers for RViz
        self.declare_parameter('oculus_ip_address', None)  # None for USB, IP for network
        self.declare_parameter('oculus_port', 5555)
        self.declare_parameter('frame_id', 'oculus_base')
        
        # Get parameters
        self.publish_rate = self.get_parameter('publish_rate').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.publish_markers = self.get_parameter('publish_markers').value
        oculus_ip = self.get_parameter('oculus_ip_address').value
        oculus_port = self.get_parameter('oculus_port').value
        self.frame_id = self.get_parameter('frame_id').value
        
        # Initialize Oculus Reader
        if OculusReader is None:
            self.get_logger().error('oculus_reader package not available. Exiting.')
            raise RuntimeError('oculus_reader package not available')
        
        try:
            if oculus_ip:
                self.get_logger().info(f'Connecting to Oculus Quest at {oculus_ip}:{oculus_port}')
                self.oculus_reader = OculusReader(ip_address=oculus_ip, port=oculus_port, run=True)
            else:
                self.get_logger().info('Connecting to Oculus Quest via USB')
                self.oculus_reader = OculusReader(run=True)
        except Exception as e:
            self.get_logger().error(f'Failed to initialize Oculus Reader: {e}')
            raise
        
        # Create publishers
        # Use BEST_EFFORT for real-time controller data (lower latency)
        qos_profile_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # Use RELIABLE for visualization markers (RViz requires RELIABLE)
        qos_profile_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        self.right_pose_pub = self.create_publisher(
            PoseStamped, 
            'oculus/right_controller/pose', 
            qos_profile_best_effort
        )
        self.left_pose_pub = self.create_publisher(
            PoseStamped, 
            'oculus/left_controller/pose', 
            qos_profile_best_effort
        )
        self.buttons_pub = self.create_publisher(
            OculusButtons, 
            'oculus/buttons', 
            qos_profile_best_effort
        )
        
        # Marker publisher for RViz visualization
        # RViz requires RELIABLE QoS for markers to display properly
        if self.publish_markers:
            self.marker_pub = self.create_publisher(
                MarkerArray,
                'oculus/controllers/markers',
                qos_profile_reliable
            )
            self.get_logger().info('Marker visualization enabled for RViz (RELIABLE QoS)')
        
        # TF broadcaster
        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)
        
        # Create timer for publishing
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.publish_data)
        
        self.get_logger().info(f'Oculus Reader Node started. Publishing at {self.publish_rate} Hz')
    
    def create_controller_marker(self, controller_id, pose, timestamp, color_r, color_g, color_b):
        """Create a marker for a controller"""
        marker = Marker()
        marker.header.stamp = timestamp.to_msg()
        marker.header.frame_id = self.frame_id
        marker.ns = 'oculus_controllers'
        marker.id = 0 if controller_id == 'r' else 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        # Set pose
        marker.pose.position.x = pose.position.x
        marker.pose.position.y = pose.position.y
        marker.pose.position.z = pose.position.z
        marker.pose.orientation = pose.orientation
        
        # Set scale (controller size approximation)
        marker.scale.x = 0.05  # 5cm radius
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        
        # Set color
        marker.color.r = color_r
        marker.color.g = color_g
        marker.color.b = color_b
        marker.color.a = 0.8
        
        # Marker lifetime (0 = infinite, use zero duration)
        from builtin_interfaces.msg import Duration
        marker.lifetime = Duration(sec=0, nanosec=0)
        
        return marker
    
    def publish_markers_rviz(self, poses, current_time):
        """Publish visualization markers for controllers in RViz"""
        marker_array = MarkerArray()
        
        # Right controller marker (red)
        if 'r' in poses:
            right_pose = transformation_matrix_to_pose(poses['r'])
            right_marker = self.create_controller_marker(
                'r', right_pose, current_time, 1.0, 0.0, 0.0  # Red
            )
            marker_array.markers.append(right_marker)
        
        # Left controller marker (blue)
        if 'l' in poses:
            left_pose = transformation_matrix_to_pose(poses['l'])
            left_marker = self.create_controller_marker(
                'l', left_pose, current_time, 0.0, 0.0, 1.0  # Blue
            )
            marker_array.markers.append(left_marker)
        
        # Publish marker array
        if marker_array.markers:
            self.marker_pub.publish(marker_array)
    
    def publish_data(self):
        """Publish controller poses and button states"""
        try:
            poses, buttons = self.oculus_reader.get_transformations_and_buttons()
        except Exception as e:
            self.get_logger().warn(f'Error reading from Oculus: {e}')
            return
        
        # 只检查 poses 是否为空，如果 poses 为空说明控制器未连接
        # buttons 即使为空字典也应该发布（表示没有按钮被按下）
        if not poses:
            return
        
        current_time = self.get_clock().now()
        
        # Publish right controller pose
        if 'r' in poses:
            right_pose_msg = PoseStamped()
            right_pose_msg.header = Header()
            right_pose_msg.header.stamp = current_time.to_msg()
            right_pose_msg.header.frame_id = self.frame_id
            right_pose_msg.pose = transformation_matrix_to_pose(poses['r'])
            self.right_pose_pub.publish(right_pose_msg)
            
            # Publish TF for right controller
            if self.publish_tf:
                right_tf = TransformStamped()
                right_tf.header.stamp = current_time.to_msg()
                right_tf.header.frame_id = self.frame_id
                right_tf.child_frame_id = 'oculus_right_controller'
                right_tf.transform.translation.x = right_pose_msg.pose.position.x
                right_tf.transform.translation.y = right_pose_msg.pose.position.y
                right_tf.transform.translation.z = right_pose_msg.pose.position.z
                right_tf.transform.rotation = right_pose_msg.pose.orientation
                self.tf_broadcaster.sendTransform(right_tf)
        
        # Publish left controller pose
        if 'l' in poses:
            left_pose_msg = PoseStamped()
            left_pose_msg.header = Header()
            left_pose_msg.header.stamp = current_time.to_msg()
            left_pose_msg.header.frame_id = self.frame_id
            left_pose_msg.pose = transformation_matrix_to_pose(poses['l'])
            self.left_pose_pub.publish(left_pose_msg)
            
            # Publish TF for left controller
            if self.publish_tf:
                left_tf = TransformStamped()
                left_tf.header.stamp = current_time.to_msg()
                left_tf.header.frame_id = self.frame_id
                left_tf.child_frame_id = 'oculus_left_controller'
                left_tf.transform.translation.x = left_pose_msg.pose.position.x
                left_tf.transform.translation.y = left_pose_msg.pose.position.y
                left_tf.transform.translation.z = left_pose_msg.pose.position.z
                left_tf.transform.rotation = left_pose_msg.pose.orientation
                self.tf_broadcaster.sendTransform(left_tf)
        
        # Publish button states (即使 buttons 为空字典也发布，表示没有按钮被按下)
        # parse_buttons_to_msg 函数会处理空字典，所有值默认为 False/0.0
        buttons_msg = parse_buttons_to_msg(buttons if buttons else {})
        buttons_msg.header.stamp = current_time.to_msg()
        buttons_msg.header.frame_id = self.frame_id
        self.buttons_pub.publish(buttons_msg)
        
        # Publish visualization markers for RViz
        if self.publish_markers:
            self.publish_markers_rviz(poses, current_time)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = OculusReaderNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if 'node' in locals():
            node.oculus_reader.stop()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

