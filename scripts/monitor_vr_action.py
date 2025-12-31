#!/usr/bin/env python3
"""
VR Policy Action Monitor

实时监控和显示 VR Policy Action 话题的内容，方便调试和验证功能。
同时发布可视化 marker 和 TF 来显示运动方向。
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from role_ros2.msg import VRPolicyAction
from visualization_msgs.msg import Marker
from geometry_msgs.msg import TransformStamped, Point, Quaternion
from tf2_ros import TransformBroadcaster
from scipy.spatial.transform import Rotation as R


class VRActionMonitor(Node):
    def __init__(self):
        super().__init__('vr_action_monitor')
        
        # QoS profile for marker publisher (RELIABLE for RViz)
        qos_profile_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # Subscriber for action messages
        self.subscription = self.create_subscription(
            VRPolicyAction,
            '/vr_policy/action',
            self.action_callback,
            10)
        
        # Publisher for movement direction marker
        self.marker_pub = self.create_publisher(
            Marker,
            '/vr_action_monitor/movement_direction',
            qos_profile_reliable
        )
        
        # TF broadcaster for movement direction
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Parameters
        self.marker_scale = 0.1  # Scale factor for arrow length (meters per unit velocity)
        self.min_velocity_threshold = 0.01  # Minimum velocity to show marker/tf
        
        self.get_logger().info('VR Action Monitor started. Listening to /vr_policy/action')
        self.get_logger().info('Publishing movement direction marker to /vr_action_monitor/movement_direction')
        self.get_logger().info('Publishing movement direction TF: oculus_base -> vr_movement_direction')
        self.get_logger().info('Press Ctrl+C to stop.')
        
    def action_callback(self, msg):
        action = msg.action
        
        # Publish marker and TF for movement direction
        self._publish_movement_direction(action, msg.movement_enabled)
        
        # 清屏并显示
        print("\033[2J\033[H", end='')  # 清屏
        print("=" * 80)
        print("VR Policy Action Monitor")
        print("=" * 80)
        print(f"\n📊 Action Vector [vx, vy, vz, wx, wy, wz, gripper_vel]:")
        print(f"   [{action[0]:7.3f}, {action[1]:7.3f}, {action[2]:7.3f}, "
              f"{action[3]:7.3f}, {action[4]:7.3f}, {action[5]:7.3f}, {action[6]:7.3f}]")
        
        print(f"\n🎮 Controller Status:")
        print(f"   Movement Enabled: {'✅ YES' if msg.movement_enabled else '❌ NO'}")
        print(f"   Controller On:    {'✅ YES' if msg.controller_on else '❌ NO'}")
        print(f"   Success Button:   {'✅ PRESSED' if msg.success else '   Released'}")
        print(f"   Failure Button:   {'⚠️  PRESSED' if msg.failure else '   Released'}")
        
        print(f"\n🔄 Joystick Orientation Reset:")
        joystick_status = "✅ PRESSED" if msg.joystick_pressed else "   Released"
        print(f"   Joystick Pressed:  {joystick_status}")
        
        # Show progress bar for hold progress
        if msg.joystick_pressed:
            progress = msg.joystick_hold_progress
            bar_length = 20
            filled = int(progress * bar_length)
            bar = "█" * filled + "░" * (bar_length - filled)
            percentage = progress * 100
            print(f"   Hold Progress:    [{bar}] {percentage:5.1f}%")
            
            if msg.orientation_resetting:
                print(f"   Status:           🔄 RESETTING ORIENTATION")
            elif progress >= 1.0:
                print(f"   Status:           ✅ Long press completed - Release to finish reset")
            else:
                remaining = (1.0 - progress) * 0.5  # 0.5s is the hold duration
                print(f"   Status:           ⏳ Hold for {remaining:.2f}s more to reset")
        else:
            print(f"   Hold Progress:    [                    ]   0.0%")
            print(f"   Status:           ⚪ Not pressed")
        
        print(f"\n🎯 Target Position [x, y, z, roll, pitch, yaw]:")
        target = msg.target_cartesian_position
        print(f"   [{target[0]:7.3f}, {target[1]:7.3f}, {target[2]:7.3f}, "
              f"{target[3]:7.3f}, {target[4]:7.3f}, {target[5]:7.3f}]")
        print(f"   Gripper Target: {msg.target_gripper_position:.3f}")
        
        # 方向提示
        print(f"\n📐 Movement Direction (based on action[0:3]):")
        if abs(action[0]) > 0.01:
            direction = "➡️  Forward" if action[0] > 0 else "⬅️  Backward"
            print(f"   X-axis (vx={action[0]:.3f}): {direction}")
        if abs(action[1]) > 0.01:
            direction = "⬆️  Left" if action[1] > 0 else "⬇️  Right"
            print(f"   Y-axis (vy={action[1]:.3f}): {direction}")
        if abs(action[2]) > 0.01:
            direction = "⬆️  Up" if action[2] > 0 else "⬇️  Down"
            print(f"   Z-axis (vz={action[2]:.3f}): {direction}")
        
        if all(abs(a) < 0.01 for a in action[:3]):
            print("   No linear movement")
        
        # 旋转提示
        if any(abs(a) > 0.01 for a in action[3:6]):
            print(f"\n🔄 Rotation (wx={action[3]:.3f}, wy={action[4]:.3f}, wz={action[5]:.3f})")
        else:
            print(f"\n🔄 No rotation")
        
        print("\n" + "=" * 80)
        print("💡 Tips:")
        print("   - Hold GRIP button to enable movement")
        print("   - Move VR controller forward → action[0] (vx) should be positive")
        print("   - Move VR controller backward → action[0] (vx) should be negative")
        print("   - Long press JOYSTICK (≥0.5s) to reset VR orientation (forward direction)")
        print("   - Watch the hold progress bar to see reset progress")
        print("   - Press Ctrl+C to exit")
        print("=" * 80)
    
    def _publish_movement_direction(self, action, movement_enabled):
        """Publish marker and TF to visualize movement direction based on action[0:3] (vx, vy, vz)"""
        # Extract linear velocity vector
        vx, vy, vz = action[0], action[1], action[2]
        velocity_vec = np.array([vx, vy, vz])
        velocity_magnitude = np.linalg.norm(velocity_vec)
        
        # Get current time
        current_time = self.get_clock().now()
        
        # Only publish if movement is enabled and velocity is significant
        if movement_enabled and velocity_magnitude > self.min_velocity_threshold:
            # Normalize direction vector
            direction_vec = velocity_vec / velocity_magnitude
            
            # Calculate arrow length (proportional to velocity magnitude)
            arrow_length = velocity_magnitude * self.marker_scale
            
            # Create arrow marker (ARROW type uses points: start and end)
            marker = Marker()
            marker.header.stamp = current_time.to_msg()
            marker.header.frame_id = 'oculus_base'
            marker.ns = 'vr_movement_direction'
            marker.id = 0
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            
            # Arrow points: start at origin, end at direction * length
            start_point = Point()
            start_point.x = 0.0
            start_point.y = 0.0
            start_point.z = 0.0
            
            end_point = Point()
            end_point.x = float(direction_vec[0] * arrow_length)
            end_point.y = float(direction_vec[1] * arrow_length)
            end_point.z = float(direction_vec[2] * arrow_length)
            
            marker.points = [start_point, end_point]
            
            # Arrow scale (shaft diameter, head diameter, head length)
            marker.scale.x = 0.02  # Shaft diameter
            marker.scale.y = 0.05  # Head diameter
            marker.scale.z = 0.0   # Not used for ARROW type
            
            # Color: green when moving forward (positive x), red when backward, blue for other directions
            if abs(vx) > max(abs(vy), abs(vz)):
                # X-axis dominant
                if vx > 0:
                    marker.color.r = 0.0
                    marker.color.g = 1.0
                    marker.color.b = 0.0
                else:
                    marker.color.r = 1.0
                    marker.color.g = 0.0
                    marker.color.b = 0.0
            else:
                # Y or Z dominant - use blue
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 1.0
            
            marker.color.a = 0.8
            
            # Marker lifetime (0 = infinite)
            from builtin_interfaces.msg import Duration
            marker.lifetime = Duration(sec=0, nanosec=0)
            
            # Publish marker
            self.marker_pub.publish(marker)
            
            # Publish TF for movement direction
            # Create a transform from oculus_base to vr_movement_direction
            # The transform represents the direction vector
            tf = TransformStamped()
            tf.header.stamp = current_time.to_msg()
            tf.header.frame_id = 'oculus_base'
            tf.child_frame_id = 'vr_movement_direction'
            
            # Translation: direction vector scaled by arrow length
            tf.transform.translation.x = float(direction_vec[0] * arrow_length)
            tf.transform.translation.y = float(direction_vec[1] * arrow_length)
            tf.transform.translation.z = float(direction_vec[2] * arrow_length)
            
            # Rotation: align Z-axis with direction vector
            # If direction is along Z-axis, no rotation needed
            if np.allclose(direction_vec, [0, 0, 1]):
                # Already aligned with Z-axis
                tf.transform.rotation.x = 0.0
                tf.transform.rotation.y = 0.0
                tf.transform.rotation.z = 0.0
                tf.transform.rotation.w = 1.0
            else:
                # Calculate rotation to align Z-axis with direction vector
                # Target: Z-axis should point in direction_vec direction
                z_axis = np.array([0, 0, 1])
                direction_normalized = direction_vec / np.linalg.norm(direction_vec)
                
                # Calculate rotation axis and angle
                if np.allclose(direction_normalized, -z_axis):
                    # 180 degree rotation around any perpendicular axis
                    axis = np.array([1, 0, 0])
                    angle = np.pi
                else:
                    # Calculate rotation axis (cross product)
                    axis = np.cross(z_axis, direction_normalized)
                    axis = axis / np.linalg.norm(axis) if np.linalg.norm(axis) > 1e-9 else np.array([1, 0, 0])
                    # Calculate angle (dot product)
                    cos_angle = np.dot(z_axis, direction_normalized)
                    cos_angle = np.clip(cos_angle, -1.0, 1.0)
                    angle = np.arccos(cos_angle)
                
                # Convert axis-angle to quaternion
                rotation = R.from_rotvec(axis * angle)
                quat = rotation.as_quat()  # Returns [x, y, z, w]
                
                tf.transform.rotation.x = float(quat[0])
                tf.transform.rotation.y = float(quat[1])
                tf.transform.rotation.z = float(quat[2])
                tf.transform.rotation.w = float(quat[3])
            
            # Publish TF
            self.tf_broadcaster.sendTransform(tf)
        else:
            # No movement or velocity too small - delete marker by publishing DELETE action
            marker = Marker()
            marker.header.stamp = current_time.to_msg()
            marker.header.frame_id = 'oculus_base'
            marker.ns = 'vr_movement_direction'
            marker.id = 0
            marker.action = Marker.DELETE
            self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    monitor = VRActionMonitor()
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        print("\n\n👋 Monitor stopped.")
    finally:
        monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

