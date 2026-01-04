"""ROS2 camera reader for single camera using TimeSynchronizer."""

import threading
from typing import Dict, Tuple, Optional
from collections import deque
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import message_filters
from tf2_ros import TransformListener, Buffer
import tf2_ros

from role_ros2.utils import ros_time_to_ns, transform_to_matrix


class ROS2CameraReader:
    """
    ROS2 camera reader for a single camera.
    
    Uses TimeSynchronizer to synchronize RGB, depth, and camera_info messages with exact timestamp matching.
    Requires all three messages to have identical timestamps for synchronization.
    Same implementation as test_topic_sync.py which successfully receives callbacks.
    Uses tf2 to get camera extrinsics with caching for static transforms.
    """
    
    def __init__(
        self,
        camera_id: str,
        rgb_topic: str,
        depth_topic: str,
        node: Node,
        camera_info_topic: Optional[str] = None,
        base_frame: str = "base_link",
        camera_frame: Optional[str] = None,
        queue_size: int = 10,
        slop: float = 0.005
    ):
        """
        Initialize ROS2 camera reader.
        
        Args:
            camera_id: Camera identifier (e.g., serial number or name)
            rgb_topic: RGB image topic (e.g., "/hand_camera/hand_camera/rgb/color/rect/image")
            depth_topic: Depth image topic (e.g., "/hand_camera/hand_camera/depth/depth_registered")
            node: ROS2 node to use for subscriptions
            camera_info_topic: Camera info topic (required for synchronization)
            base_frame: Base frame for tf lookup (default: "base_link")
            camera_frame: Camera frame for tf lookup (if None, extracted from image header)
            queue_size: Queue size for TimeSynchronizer
            slop: DEPRECATED - TimeSynchronizer requires exact timestamp matching (parameter kept for compatibility, not used)
        """
        self.camera_id = camera_id
        self.node = node
        self.base_frame = base_frame
        self.camera_frame = camera_frame
        self.cv_bridge = CvBridge()
        
        # Initialize callback count for logging
        self._callback_count = 0
        
        # Thread-safe storage for synchronized data using queue (dict keyed by timestamp)
        # Queue stores: {timestamp_ns: (data_dict, timestamp_dict)}
        # Intrinsics are stored separately since they don't change over time
        self._data_queue: Dict[int, Tuple[Dict, Dict]] = {}
        self._timestamp_order: deque = deque(maxlen=100)  # Keep order, limit to 100 entries
        self._data_lock = threading.Lock()
        self._queue_max_size = 100  # Maximum number of entries to keep
        
        # Event to notify when new data is available
        self._data_available_event = threading.Event()
        
        # Camera intrinsics (fixed, stored once)
        self._intrinsics: Optional[Dict] = None
        
        # TF buffer and listener
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, node)
        self._tf_warning_printed = False
        
        # TF transform cache for static transforms (camera extrinsics are typically static)
        self._tf_cache: Optional[np.ndarray] = None
        self._tf_cache_lock = threading.Lock()
        self._tf_cache_valid = False
        
        # Create subscribers (camera_info is required for synchronization)
        if camera_info_topic is None:
            raise ValueError(f"camera_info_topic is required for camera {camera_id}")
        
        # Create and store subscribers as instance variables to prevent garbage collection
        # This is critical: subscribers must be kept alive for the synchronizer to work
        self._rgb_sub = message_filters.Subscriber(node, Image, rgb_topic)
        self._depth_sub = message_filters.Subscriber(node, Image, depth_topic)
        self._camera_info_sub = message_filters.Subscriber(node, CameraInfo, camera_info_topic)
        
        # Log actual topic names for debugging
        node.get_logger().info(
            f"[CameraReader {camera_id}] Subscribing to topics: "
            f"RGB={rgb_topic}, Depth={depth_topic}, CameraInfo={camera_info_topic}"
        )
        
        # Synchronize camera_info, RGB, and depth (three-way synchronization)
        # Use TimeSynchronizer for exact timestamp matching (same as test_topic_sync.py)
        # Note: TimeSynchronizer requires identical timestamps, but test shows it works
        self._sync = message_filters.TimeSynchronizer(
            [self._camera_info_sub, self._rgb_sub, self._depth_sub],
            queue_size=queue_size
        )
        self._sync.registerCallback(self._sync_callback)
        
        node.get_logger().info(
            f"ROS2CameraReader initialized for {camera_id} with TimeSynchronizer "
            f"(exact timestamp matching): RGB={rgb_topic}, Depth={depth_topic}, CameraInfo={camera_info_topic}"
        )
    
    def _get_cached_tf_transform(self, camera_frame: str) -> Optional[np.ndarray]:
        """
        Get cached TF transform or query and cache it if not available.
        
        For static transforms (typical for camera extrinsics), this avoids repeated lookups.
        
        Args:
            camera_frame: Camera frame name
            
        Returns:
            4x4 transformation matrix, or None if lookup fails
        """
        with self._tf_cache_lock:
            # If cache is valid, return cached transform
            if self._tf_cache_valid and self._tf_cache is not None:
                return self._tf_cache.copy()
        
        # Cache miss or invalid - try to lookup transform
        try:
            transform = self._tf_buffer.lookup_transform(
                self.base_frame,
                camera_frame,
                rclpy.time.Time()
            )
            
            # Convert ROS2 transform to 4x4 matrix using utility function
            extrinsics = transform_to_matrix(transform)
            
            # Cache the transform (assume static for camera extrinsics)
            with self._tf_cache_lock:
                self._tf_cache = extrinsics.copy()
                self._tf_cache_valid = True
            
            return extrinsics
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            if not self._tf_warning_printed:
                self.node.get_logger().warning(
                    f"TF lookup failed for camera {self.camera_id} "
                    f"({self.base_frame} -> {camera_frame}): {e}. "
                    f"Extrinsics will be None."
                )
                self._tf_warning_printed = True
            return None
    
    def _sync_callback(self, camera_info_msg: CameraInfo, rgb_msg: Image, depth_msg: Image):
        """
        Callback for synchronized camera_info, RGB, and depth images.
        
        Args:
            camera_info_msg: Camera info message
            rgb_msg: RGB image message
            depth_msg: Depth image message
        """
        try:
            # Track callback count for logging (initialize if needed)
            if not hasattr(self, '_callback_count'):
                self._callback_count = 0
            # Get ROS time at start of callback (subscription time)
            ros_time_sub = self.node.get_clock().now()
            sub_t_ns = ros_time_to_ns(ros_time_sub)
            
            # Get published time from message header (ROS time, nanoseconds)
            # TimeSynchronizer ensures messages have identical timestamps (same as test_topic_sync.py)
            # Use camera_info as primary timestamp
            pub_t_ns = int(
                camera_info_msg.header.stamp.sec * 1_000_000_000 +
                camera_info_msg.header.stamp.nanosec
            )
            
            # Convert images
            rgb_cv = self.cv_bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
            depth_cv = self.cv_bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
            
            # Convert depth to uint16 if needed (ZED cameras typically use 16UC1)
            if depth_cv.dtype == np.float32:
                # Convert from meters to millimeters
                # Clip values to uint16 range to avoid overflow
                depth_cv_mm = depth_cv * 1000
                depth_cv = np.clip(depth_cv_mm, 0, 65535).astype(np.uint16)
            elif depth_cv.dtype != np.uint16:
                depth_cv = depth_cv.astype(np.uint16)
            
            # Get camera frame from image header if not provided
            camera_frame = self.camera_frame
            if camera_frame is None:
                camera_frame = rgb_msg.header.frame_id
            
            # Try to get extrinsics from tf (with caching)
            extrinsics = None
            if camera_frame:
                extrinsics = self._get_cached_tf_transform(camera_frame)
            
            # Extract intrinsics from camera_info
            # Check if k exists and has values (avoid array truth value ambiguity)
            # Use len() check first to avoid truth value ambiguity with arrays
            if len(camera_info_msg.k) == 9:
                K = np.array(camera_info_msg.k).reshape(3, 3)
            else:
                K = None
            
            # Check if d exists and has values (avoid array truth value ambiguity)
            # Use len() check first to avoid truth value ambiguity with arrays
            if len(camera_info_msg.d) > 0:
                distortion_coeffs = np.array(camera_info_msg.d)
            else:
                distortion_coeffs = None
            
            intrinsics = {
                "K": K,
                "cameraMatrix": K,
                "distortionCoefficients": distortion_coeffs,
                "width": camera_info_msg.width,
                "height": camera_info_msg.height
            }
            
            # Get ROS time at end of callback (after processing and transform calculation)
            ros_time_end = self.node.get_clock().now()
            end_t_ns = ros_time_to_ns(ros_time_end)
            
            # Build data_dict in droid format
            data_dict = {
                "image": {self.camera_id: rgb_cv},
                "depth": {self.camera_id: depth_cv}
            }
            
            # Build timestamp_dict in droid format (all in nanoseconds)
            # TimeSynchronizer ensures messages have identical timestamps (same as test_topic_sync.py)
            # We store one pub_t timestamp (from camera_info)
            timestamp_dict = {
                self.camera_id + "_read_start": sub_t_ns,
                self.camera_id + "_frame_received": pub_t_ns,
                self.camera_id + "_read_end": end_t_ns,
                self.camera_id + "_pub_t": pub_t_ns,
                self.camera_id + "_sub_t": sub_t_ns,
                self.camera_id + "_end_t": end_t_ns,
            }
            
            # Store data in queue keyed by timestamp (thread-safe)
            with self._data_lock:
                # Store intrinsics once (they don't change)
                if self._intrinsics is None:
                    self._intrinsics = intrinsics
                
                # Add new entry to queue (only data_dict and timestamp_dict)
                self._data_queue[pub_t_ns] = (data_dict, timestamp_dict)
                self._timestamp_order.append(pub_t_ns)
                
                # Remove oldest entries if queue exceeds max size
                while len(self._data_queue) > self._queue_max_size:
                    if self._timestamp_order:
                        oldest_timestamp = self._timestamp_order.popleft()
                        if oldest_timestamp in self._data_queue:
                            del self._data_queue[oldest_timestamp]
            
            # Notify waiting threads that new data is available
            self._data_available_event.set()
            self._data_available_event.clear()  # Auto-reset for next wait
            
            # Track callback count for logging
            self._callback_count += 1
            
            # Log first 10 callbacks and then every 30th callback for debugging
            if self._callback_count <= 10 or self._callback_count % 30 == 0:
                # Calculate timestamp differences for debugging
                rgb_t_ns = int(rgb_msg.header.stamp.sec * 1_000_000_000 + rgb_msg.header.stamp.nanosec)
                depth_t_ns = int(depth_msg.header.stamp.sec * 1_000_000_000 + depth_msg.header.stamp.nanosec)
                info_t_ns = pub_t_ns
                
                rgb_diff_ms = abs(rgb_t_ns - info_t_ns) / 1e6
                depth_diff_ms = abs(depth_t_ns - info_t_ns) / 1e6
                
                self.node.get_logger().info(
                    f"[CameraReader {self.camera_id}] ✅ Sync callback #{self._callback_count}: "
                    f"pub_t={pub_t_ns/1e9:.9f}s, "
                    f"RGB_diff={rgb_diff_ms:.2f}ms, Depth_diff={depth_diff_ms:.2f}ms, "
                    f"queue_size={len(self._data_queue)}"
                )
                        
        except Exception as e:
            self.node.get_logger().error(f"Error processing camera {self.camera_id} images: {e}")
            import traceback
            self.node.get_logger().error(traceback.format_exc())
    
    def read_camera(self) -> Tuple[Dict, Dict]:
        """
        Read latest camera data in droid format.
        
        Returns:
            Tuple[dict, dict]: (data_dict, timestamp_dict)
                - data_dict: Dictionary with "image" and "depth" keys, each containing
                  {camera_id: numpy_array}
                - timestamp_dict: Dictionary with timestamp information (all in nanoseconds):
                  - {camera_id}_read_start: Start time (nanoseconds, ROS time)
                  - {camera_id}_frame_received: Frame received time (nanoseconds, ROS time)
                  - {camera_id}_read_end: End time (nanoseconds, ROS time)
                  - {camera_id}_pub_t: Published time (nanoseconds, ROS time from message header)
                  - {camera_id}_sub_t: Subscription/receive time (nanoseconds, ROS time)
                  - {camera_id}_end_t: End processing time (nanoseconds, ROS time)
        """
        with self._data_lock:
            if not self._timestamp_order:
                return {}, {}
            # Get latest timestamp (most recent)
            latest_timestamp = self._timestamp_order[-1]
            if latest_timestamp in self._data_queue:
                data_dict, timestamp_dict = self._data_queue[latest_timestamp]
                return data_dict, timestamp_dict
            return {}, {}
    
    def get_intrinsics(self) -> Optional[Dict]:
        """
        Get camera intrinsics.
        
        Camera intrinsics are fixed and don't change over time, so they are stored once.
        
        Returns:
            Dictionary with intrinsics information, or None if not available:
                - K: 3x3 intrinsic matrix
                - cameraMatrix: Same as K (for compatibility)
                - distortionCoefficients: Distortion coefficients
                - width: Image width
                - height: Image height
        """
        with self._data_lock:
            return self._intrinsics
    
    def is_running(self) -> bool:
        """
        Check if camera is running (has received data).
        
        Returns:
            True if camera has received data, False otherwise
        """
        with self._data_lock:
            return len(self._data_queue) > 0
    
    def get_latest_pub_timestamp(self) -> Optional[int]:
        """
        Get the published timestamp of the latest synchronized data.
        
        Returns:
            Published timestamp in nanoseconds, or None if no data available
        """
        with self._data_lock:
            if not self._timestamp_order:
                return None
            return self._timestamp_order[-1]
    
    def get_data_for_timestamp(
        self, 
        target_timestamp_ns: int, 
        tolerance_ns: int = 100_000_000
    ) -> Optional[Tuple[Dict, Dict]]:
        """
        Get camera data for a specific timestamp key from the queue.
        
        Searches the queue for data with timestamp matching target_timestamp_ns within tolerance.
        If exact match is found, returns that data. Otherwise, finds the closest match within tolerance.
        
        Args:
            target_timestamp_ns: Target timestamp in nanoseconds (key to search for)
            tolerance_ns: Tolerance in nanoseconds (default: 100ms)
            
        Returns:
            Tuple[dict, dict]: (data_dict, timestamp_dict) if found, None otherwise
        """
        with self._data_lock:
            if not self._data_queue:
                return None
            
            # First, try exact match
            if target_timestamp_ns in self._data_queue:
                data_dict, timestamp_dict = self._data_queue[target_timestamp_ns]
                return data_dict, timestamp_dict
            
            # If no exact match, find closest timestamp within tolerance
            best_match = None
            min_diff = tolerance_ns + 1
            
            for timestamp_ns in self._data_queue.keys():
                diff = abs(timestamp_ns - target_timestamp_ns)
                if diff <= tolerance_ns and diff < min_diff:
                    min_diff = diff
                    best_match = timestamp_ns
            
            if best_match is not None:
                data_dict, timestamp_dict = self._data_queue[best_match]
                return data_dict, timestamp_dict
            
            return None
    
    def wait_for_data(
        self, 
        timeout_sec: float = 1.0
    ) -> bool:
        """
        Wait for new data to become available.
        
        Uses threading.Event to avoid blocking the executor.
        
        Args:
            timeout_sec: Maximum time to wait in seconds
            
        Returns:
            True if data became available, False if timeout
        """
        return self._data_available_event.wait(timeout=timeout_sec)
