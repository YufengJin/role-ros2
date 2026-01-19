#!/usr/bin/env python3
"""
Camera Calibration TF Publisher Node

This node reads calibration results from calibration_results.yaml and publishes
static TF transforms for cameras that have been calibrated.

The node:
1. Loads calibration_results.yaml
2. Checks if the camera's serial_number matches a camera_id in the file
3. If found, publishes static TF transform (parent_frame -> child_frame)
4. If not found, logs a message and exits

Usage:
    This node is typically launched automatically by camera launch files
    (zed_camera.launch.py, realsense_camera.launch.py) after the camera starts.

Author: Role-ROS2 Team
"""

import os
import sys
from typing import Optional, Dict

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
from scipy.spatial.transform import Rotation as R
import yaml

from role_ros2.misc.config_loader import get_source_config_path


class CameraCalibrationTFPublisherNode(Node):
    """
    ROS2 node that publishes static TF transforms for calibrated cameras.
    
    The node reads calibration_results.yaml and publishes static TF if the
    camera's serial_number matches a camera_id in the calibration file.
    """
    
    def __init__(self, serial_number: str):
        """
        Initialize the camera calibration TF publisher node.
        
        Args:
            serial_number: Camera serial number (will be compared to camera_id in calibration file)
        """
        super().__init__('camera_calibration_tf_publisher')
        
        self.serial_number = str(serial_number)
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        self.tf_published = False  # Flag to track if TF was successfully published
        
        # Load calibration results
        calibration_data = self._load_calibration_results()
        
        # Find matching camera
        camera_info = self._find_camera_by_id(calibration_data, self.serial_number)
        
        if camera_info is None:
            self.get_logger().warn(
                f"⚠️  Camera with serial_number={self.serial_number} not found in calibration_results.yaml. "
                f"Skipping static TF publication."
            )
            self.get_logger().info(
                f"   To calibrate this camera, run: "
                f"python3 scripts/calibrate_camera.py --camera_id {self.serial_number} --mode <hand|third>"
            )
            # If camera is not calibrated, there's nothing to publish
            # Node will exit in main() if tf_published is False
            return
        
        # Publish static TF
        self._publish_static_tf(camera_info)
        self.tf_published = True
        
        self.get_logger().info(
            f"✅ Published static TF for camera {self.serial_number}: "
            f"{camera_info['parent_frame']} -> {camera_info['child_frame']}"
        )
    
    def _load_calibration_results(self) -> Dict:
        """
        Load calibration results from YAML file.
        
        Returns:
            Dictionary with calibration data (empty dict if file not found or error)
        """
        # Get calibration results file path (use source directory, not install directory)
        config_path = get_source_config_path('calibration_results.yaml')
        
        if not os.path.isfile(config_path):
            self.get_logger().warn(
                f"⚠️  Calibration results file not found: {config_path}"
            )
            return {}
        
        try:
            with open(config_path, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f) or {}
            return data
        except Exception as e:
            self.get_logger().error(
                f"❌ Failed to load calibration results from {config_path}: {e}"
            )
            return {}
    
    def _find_camera_by_id(self, calibration_data: Dict, camera_id: str) -> Optional[Dict]:
        """
        Find camera calibration info by camera_id.
        
        Args:
            calibration_data: Calibration data dictionary
            camera_id: Camera ID to search for
        
        Returns:
            Camera info dictionary if found, None otherwise
        """
        cameras = calibration_data.get("cameras", [])
        
        for cam in cameras:
            # Compare as strings (camera_id might be stored as string or int)
            cam_id = str(cam.get("camera_id", ""))
            if cam_id == str(camera_id):
                return cam
        
        return None
    
    def _publish_static_tf(self, camera_info: Dict):
        """
        Publish static TF transform for calibrated camera.
        
        Args:
            camera_info: Camera calibration info dictionary with:
                - child_frame: Child frame name
                - parent_frame: Parent frame name
                - transform: Dictionary with x, y, z, rx, ry, rz
        """
        # Extract transform data
        transform_data = camera_info.get("transform", {})
        x = float(transform_data.get("x", 0.0))
        y = float(transform_data.get("y", 0.0))
        z = float(transform_data.get("z", 0.0))
        rx = float(transform_data.get("rx", 0.0))
        ry = float(transform_data.get("ry", 0.0))
        rz = float(transform_data.get("rz", 0.0))
        
        # Extract frame names
        parent_frame = camera_info.get("parent_frame", "base_link")
        child_frame = camera_info.get("child_frame", "")
        
        if not child_frame:
            self.get_logger().error("❌ child_frame not specified in calibration data")
            return
        
        # Convert euler angles to quaternion
        quat = R.from_euler("xyz", [rx, ry, rz]).as_quat()
        
        # Create transform message
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = parent_frame
        transform.child_frame_id = child_frame
        
        # Set translation
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.translation.z = z
        
        # Set rotation (quaternion: x, y, z, w)
        transform.transform.rotation.x = float(quat[0])
        transform.transform.rotation.y = float(quat[1])
        transform.transform.rotation.z = float(quat[2])
        transform.transform.rotation.w = float(quat[3])
        
        # Publish static transform
        self.tf_broadcaster.sendTransform(transform)
        
        self.get_logger().info(
            f"📡 Published static TF: {parent_frame} -> {child_frame}\n"
            f"   Translation: [{x:.4f}, {y:.4f}, {z:.4f}] m\n"
            f"   Rotation (euler): [{rx:.4f}, {ry:.4f}, {rz:.4f}] rad"
        )


def main(args=None):
    """Main function."""
    rclpy.init(args=args)
    
    # Get serial_number from command line argument or environment variable
    serial_number = None
    
    # Try to get from command line arguments
    if len(sys.argv) > 1:
        serial_number = sys.argv[1]
    else:
        # Try to get from environment variable
        serial_number = os.environ.get('CAMERA_SERIAL_NUMBER', None)
    
    if serial_number is None:
        print("❌ Error: serial_number not provided!")
        print("   Usage: camera_calibration_tf_publisher_node.py <serial_number>")
        print("   Or set environment variable: CAMERA_SERIAL_NUMBER=<serial_number>")
        sys.exit(1)
    
    try:
        node = CameraCalibrationTFPublisherNode(serial_number=serial_number)
        
        # If TF was not published (camera not calibrated), exit gracefully
        if not node.tf_published:
            node.get_logger().info("   Node exiting (camera not calibrated)")
            node.destroy_node()
            rclpy.shutdown()
            return
        
        # Keep node alive to continuously publish static TF
        # Static TF broadcaster needs to stay alive to maintain the transform
        # Use spin() to keep the node running indefinitely
        rclpy.spin(node)
    except KeyboardInterrupt:
        if 'node' in locals():
            node.get_logger().info("🛑 Shutting down camera calibration TF publisher...")
    except Exception as e:
        print(f"❌ Error in camera_calibration_tf_publisher_node: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
