#!/usr/bin/env python3
"""
Fake camera image publisher for testing ROS2 camera synchronization.

Publishes synthetic RGB, depth, and camera_info messages to simulate camera data.
Supports configurable publishing rate, delay, and image properties.
"""

import sys
import time
import argparse
from pathlib import Path
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
from role_ros2.misc.transformations import euler_to_quat


class FakeCameraPublisher(Node):
    """
    Fake camera publisher that generates synthetic images.
    
    Publishes:
    - RGB image (sensor_msgs/Image)
    - Depth image (sensor_msgs/Image)
    - Camera info (sensor_msgs/CameraInfo)
    """
    
    def __init__(
        self,
        camera_id: str,
        rgb_topic: str,
        depth_topic: str,
        camera_info_topic: str,
        publish_rate: float = 30.0,
        delay_ms: float = 0.0,
        image_width: int = 640,
        image_height: int = 480,
        use_sim_time: bool = False,
        base_frame: str = "base_link",
        camera_frame: Optional[str] = None,
        random_pose: bool = True
    ):
        """
        Initialize fake camera publisher.
        
        Args:
            camera_id: Camera identifier (for logging)
            rgb_topic: RGB image topic name
            depth_topic: Depth image topic name
            camera_info_topic: Camera info topic name
            publish_rate: Publishing rate in Hz (default: 30.0)
            delay_ms: Simulated delay in milliseconds (default: 0.0)
            image_width: Image width in pixels (default: 640)
            image_height: Image height in pixels (default: 480)
            use_sim_time: Whether to use ROS simulation time (default: False)
            base_frame: Base frame for TF transform (default: "base_link")
            camera_frame: Camera frame name for TF transform (default: None, uses camera_id)
            random_pose: Whether to use random pose for static transform (default: True)
        """
        super().__init__(f'fake_camera_publisher_{camera_id}')
        
        self.camera_id = camera_id
        self.publish_rate = publish_rate
        self.delay_ms = delay_ms
        self.image_width = image_width
        self.image_height = image_height
        self.use_sim_time = use_sim_time
        self.base_frame = base_frame
        self.camera_frame = camera_frame or f"{camera_id}_camera_frame_optical"
        
        # Frame counter for generating patterns
        self.frame_counter = 0
        
        # FPS tracking
        self._fps_start_time = None
        self._fps_frame_count = 0
        self._fps_update_interval = 1.0  # Update FPS every 1 second
        self._fps_last_update_time = None
        self._fps_current = 0.0
        
        # Pre-allocate image arrays
        self._rgb_img = np.zeros((self.image_height, self.image_width, 3), dtype=np.uint8)
        self._depth_img = np.zeros((self.image_height, self.image_width), dtype=np.uint16)
        
        # Create publishers with larger queue size for better performance
        self.rgb_pub = self.create_publisher(Image, rgb_topic, 10)
        self.depth_pub = self.create_publisher(Image, depth_topic, 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, camera_info_topic, 10)
        
        # Create timer for publishing
        period = 1.0 / publish_rate if publish_rate > 0 else 1.0
        self.timer = self.create_timer(period, self.publish_images)
        
        # Generate camera info once
        self.camera_info = self._generate_camera_info()
        
        # Setup static transform broadcaster
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        
        # Generate random pose for static transform
        if random_pose:
            self._generate_and_publish_static_transform()
        
        self.get_logger().info(
            f"Fake camera publisher initialized for {camera_id}:\n"
            f"  RGB topic: {rgb_topic}\n"
            f"  Depth topic: {depth_topic}\n"
            f"  Camera info topic: {camera_info_topic}\n"
            f"  Publish rate: {publish_rate} Hz\n"
            f"  Delay: {delay_ms} ms\n"
            f"  Image size: {image_width}x{image_height}\n"
            f"  Use sim time: {use_sim_time}\n"
            f"  Base frame: {base_frame}\n"
            f"  Camera frame: {self.camera_frame}\n"
            f"  Random pose: {random_pose}"
        )
    
    def _generate_and_publish_static_transform(self):
        """
        Generate random pose and publish static transform from base_frame to camera_frame.
        
        Random pose ranges:
        - Position: x, y, z in [-1.0, 1.0] meters
        - Orientation: Random euler angles (roll, pitch, yaw) in [-π, π] radians
        """
        # Generate random position (in meters)
        np.random.seed(hash(self.camera_id) % (2**32))  # Deterministic seed based on camera_id
        position = np.random.uniform(-1.0, 1.0, size=3)
        
        # Generate random orientation (euler angles in radians)
        euler_angles = np.random.uniform(-np.pi, np.pi, size=3)
        quaternion = euler_to_quat(euler_angles)
        
        # Create transform message
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.base_frame
        transform.child_frame_id = self.camera_frame
        
        # Set translation
        transform.transform.translation.x = float(position[0])
        transform.transform.translation.y = float(position[1])
        transform.transform.translation.z = float(position[2])
        
        # Set rotation (quaternion: x, y, z, w)
        transform.transform.rotation.x = float(quaternion[0])
        transform.transform.rotation.y = float(quaternion[1])
        transform.transform.rotation.z = float(quaternion[2])
        transform.transform.rotation.w = float(quaternion[3])
        
        # Publish static transform
        self.tf_broadcaster.sendTransform(transform)
        
        self.get_logger().info(
            f"Published static transform: {self.base_frame} -> {self.camera_frame}\n"
            f"  Position: [{position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f}] m\n"
            f"  Orientation (euler): [{np.degrees(euler_angles[0]):.1f}, "
            f"{np.degrees(euler_angles[1]):.1f}, {np.degrees(euler_angles[2]):.1f}] deg"
        )
    
    def _generate_camera_info(self) -> CameraInfo:
        """
        Generate a synthetic camera info message.
        
        Returns:
            CameraInfo message with intrinsics
        """
        camera_info = CameraInfo()
        camera_info.width = self.image_width
        camera_info.height = self.image_height
        
        # Set intrinsic matrix (K)
        # Assume focal length = width (typical for cameras)
        fx = fy = float(self.image_width)
        cx = float(self.image_width) / 2.0
        cy = float(self.image_height) / 2.0
        
        camera_info.k = [
            fx, 0.0, cx,
            0.0, fy, cy,
            0.0, 0.0, 1.0
        ]
        
        # Set distortion coefficients (no distortion)
        camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        
        # Set projection matrix (P)
        camera_info.p = [
            fx, 0.0, cx, 0.0,
            0.0, fy, cy, 0.0,
            0.0, 0.0, 1.0, 0.0
        ]
        
        # Set rectification matrix (R) - identity
        camera_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        
        # Set distortion model
        camera_info.distortion_model = "plumb_bob"
        
        return camera_info
    
    def _generate_rgb_image(self) -> np.ndarray:
        """
        Generate a synthetic RGB image with random noise.
        
        Uses pre-allocated arrays and fast NumPy random generation.
        
        Returns:
            BGR image as numpy array (uint8)
        """
        # Generate random noise image (BGR format) - very fast
        np.random.seed(self.frame_counter)  # Different seed per frame for different noise
        self._rgb_img[:] = np.random.randint(0, 256, size=(self.image_height, self.image_width, 3), dtype=np.uint8)
        
        return self._rgb_img
    
    def _generate_depth_image(self) -> np.ndarray:
        """
        Generate a synthetic depth image with random noise.
        
        Uses pre-allocated arrays and fast NumPy random generation.
        
        Returns:
            Depth image as numpy array (uint16, in millimeters)
        """
        # Generate random depth values (500-1500mm range) - very fast
        np.random.seed(self.frame_counter + 1000)  # Different seed for depth
        self._depth_img[:] = np.random.randint(500, 1500, size=(self.image_height, self.image_width), dtype=np.uint16)
        
        return self._depth_img
    
    def _numpy_to_image_msg(self, img: np.ndarray, encoding: str = 'bgr8') -> Image:
        """
        Convert numpy array to ROS2 Image message.
        
        Args:
            img: Image as numpy array
            encoding: Image encoding (default: 'bgr8' for RGB, '16UC1' for depth)
            
        Returns:
            Image message
        """
        msg = Image()
        
        # Set header
        if self.use_sim_time:
            msg.header.stamp = self.get_clock().now().to_msg()
        else:
            # Use wall clock time
            now = self.get_clock().now()
            msg.header.stamp = now.to_msg()
        
        msg.header.frame_id = self.camera_frame
        
        # Set image properties
        msg.height = img.shape[0]
        msg.width = img.shape[1]
        msg.encoding = encoding
        msg.is_bigendian = False
        
        # Set step (bytes per row)
        if len(img.shape) == 3:
            # BGR image: width * channels * bytes_per_channel
            msg.step = img.shape[1] * img.shape[2] * img.dtype.itemsize
        else:
            # Depth image: width * bytes_per_pixel
            if encoding == '16UC1':
                msg.step = img.shape[1] * 2  # uint16 = 2 bytes
            else:
                msg.step = img.shape[1] * img.dtype.itemsize
        
        # Set data
        msg.data = img.tobytes()
        
        return msg
    
    def publish_images(self):
        """Publish RGB, depth, and camera_info messages."""
        try:
            # Track FPS
            current_time = time.time()
            if self._fps_start_time is None:
                self._fps_start_time = current_time
                self._fps_last_update_time = current_time
            
            # Generate images
            rgb_img = self._generate_rgb_image()
            depth_img = self._generate_depth_image()
            
            # Apply delay if specified
            if self.delay_ms > 0:
                time.sleep(self.delay_ms / 1000.0)
            
            # Convert to ROS2 messages
            rgb_msg = self._numpy_to_image_msg(rgb_img, encoding='bgr8')
            depth_msg = self._numpy_to_image_msg(depth_img, encoding='16UC1')
            
            # Get timestamp after all data processing is complete, just before publishing
            # This ensures RGB, depth, and camera_info from the same camera have identical timestamps
            # and the timestamp represents the latest time after all data preparation
            timestamp = self.get_clock().now()
            timestamp_msg = timestamp.to_msg()
            
            # Set same timestamp for all messages from this camera (for synchronization)
            # Different cameras will have different timestamps since each camera has its own timer
            rgb_msg.header.stamp = timestamp_msg
            depth_msg.header.stamp = timestamp_msg
            
            # Update camera info with current timestamp
            self.camera_info.header.stamp = timestamp_msg
            self.camera_info.header.frame_id = self.camera_frame
            
            # Publish messages
            self.rgb_pub.publish(rgb_msg)
            self.depth_pub.publish(depth_msg)
            self.camera_info_pub.publish(self.camera_info)
            
            # Increment frame counter
            self.frame_counter += 1
            self._fps_frame_count += 1
            
            # Calculate and update FPS (only log every second to reduce overhead)
            elapsed_since_last_update = current_time - self._fps_last_update_time
            if elapsed_since_last_update >= self._fps_update_interval:
                # Calculate FPS
                self._fps_current = self._fps_frame_count / elapsed_since_last_update
                
                # Log FPS and frame info (use print for faster output, or reduce logging level)
                print(
                    f"[{self.camera_id}] Frame #{self.frame_counter} | "
                    f"FPS: {self._fps_current:.2f} Hz (target: {self.publish_rate:.2f} Hz) | "
                    f"Delay: {self.delay_ms:.1f} ms"
                )
                
                # Reset counters
                self._fps_frame_count = 0
                self._fps_last_update_time = current_time
        
        except Exception as e:
            self.get_logger().error(f"Error publishing images: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())


def load_camera_config(config_file: Optional[str] = None):
    """
    Load camera configurations from YAML file.
    
    Args:
        config_file: Path to YAML configuration file (if None, uses config/multi_camera_config.yaml)
        
    Returns:
        tuple: (config_data dict, cameras list)
    """
    from role_ros2.misc.config_loader import load_yaml_config, get_package_config_path
    
    if config_file is None:
        # Use unified config loader to find config file in config/ directory
        config_data = load_yaml_config('multi_camera_reader_config.yaml')
    else:
        import yaml
        config_file = Path(config_file)
        if not config_file.exists():
            raise FileNotFoundError(f"Config file not found: {config_file}")
        with open(config_file, 'r') as f:
            config_data = yaml.safe_load(f)
    
    cameras = config_data.get("cameras", [])
    return config_data, cameras


def main():
    """Main function."""
    parser = argparse.ArgumentParser(
        description='Fake camera image publisher for testing ROS2 camera synchronization'
    )
    parser.add_argument('--config-file', type=str, default=None,
                       help='Path to camera configuration file (optional)')
    parser.add_argument('--camera-id', type=str, default=None,
                       help='Specific camera ID to publish (optional, if None publishes all cameras)')
    parser.add_argument('--publish-rate', type=float, default=30.0,
                       help='Publishing rate in Hz (default: 30.0)')
    parser.add_argument('--delay-ms', type=float, default=0.0,
                       help='Simulated delay in milliseconds (default: 0.0)')
    parser.add_argument('--image-width', type=int, default=640,
                       help='Image width in pixels (default: 640)')
    parser.add_argument('--image-height', type=int, default=480,
                       help='Image height in pixels (default: 480)')
    parser.add_argument('--use-sim-time', action='store_true',
                       help='Use ROS simulation time')
    parser.add_argument('--random-pose', action='store_true', default=True,
                       help='Use random pose for static transform (default: True)')
    parser.add_argument('--no-random-pose', dest='random_pose', action='store_false',
                       help='Disable random pose for static transform')
    
    args = parser.parse_args()
    
    rclpy.init()
    
    try:
        # Load camera configurations
        config_data, cameras = load_camera_config(args.config_file)
        
        if not cameras:
            print("❌ No cameras found in configuration file!")
            sys.exit(1)
        
        # Filter to specific camera if requested
        if args.camera_id:
            cameras = [c for c in cameras if c.get("camera_id") == args.camera_id]
            if not cameras:
                print(f"❌ Camera {args.camera_id} not found in configuration!")
                sys.exit(1)
        
        # Create publishers for each camera
        publishers = []
        for camera_config in cameras:
            camera_id = camera_config["camera_id"]
            rgb_topic = camera_config["rgb_topic"]
            depth_topic = camera_config["depth_topic"]
            camera_info_topic = camera_config.get("camera_info_topic")
            
            if camera_info_topic is None:
                print(f"⚠️  Camera {camera_id} has no camera_info_topic, skipping")
                continue
            
            # Get image dimensions and publish rate from config or use defaults
            image_width = camera_config.get("image_width", args.image_width)
            image_height = camera_config.get("image_height", args.image_height)
            # Allow per-camera publish_rate override from config, fallback to command line arg
            camera_publish_rate = camera_config.get("publish_rate", args.publish_rate)
            
            # Get base_frame and camera_frame from config
            base_frame = camera_config.get("base_frame", config_data.get("base_frame", "base_link"))
            camera_frame = camera_config.get("camera_frame")
            
            publisher = FakeCameraPublisher(
                camera_id=camera_id,
                rgb_topic=rgb_topic,
                depth_topic=depth_topic,
                camera_info_topic=camera_info_topic,
                publish_rate=camera_publish_rate,
                delay_ms=args.delay_ms,
                image_width=image_width,
                image_height=image_height,
                use_sim_time=args.use_sim_time,
                base_frame=base_frame,
                camera_frame=camera_frame,
                random_pose=args.random_pose
            )
            publishers.append(publisher)
        
        if not publishers:
            print("❌ No valid cameras to publish!")
            sys.exit(1)
        
        print(f"\n✅ Started {len(publishers)} fake camera publisher(s)")
        print(f"   Default publishing rate: {args.publish_rate} Hz (can be overridden per camera in config)")
        print(f"   Delay: {args.delay_ms} ms")
        print(f"   Default image size: {args.image_width}x{args.image_height}")
        print(f"   Use sim time: {args.use_sim_time}")
        print("\nCamera configurations:")
        for publisher in publishers:
            print(f"   - {publisher.camera_id}: {publisher.publish_rate} Hz")
        print("\nPress Ctrl+C to stop...\n")
        
        # Spin all publishers
        executor = rclpy.executors.MultiThreadedExecutor()
        for publisher in publishers:
            executor.add_node(publisher)
        
        try:
            executor.spin()
        except KeyboardInterrupt:
            print("\n⚠️  Shutting down...")
        finally:
            for publisher in publishers:
                publisher.destroy_node()
            executor.shutdown()
            
    except Exception as e:
        print(f"❌ Error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()

