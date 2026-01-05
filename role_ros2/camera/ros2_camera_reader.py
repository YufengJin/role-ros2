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

from role_ros2.misc.ros2_utils import ros_time_to_ns


class ROS2CameraReader:
    """
    ROS2 camera reader for a single camera.
    
    Uses TimeSynchronizer to synchronize camera_info, RGB, and depth messages with exact timestamp matching.
    Requires all three messages to have identical timestamps for synchronization.
    """
    
    def __init__(
        self,
        camera_id: str,
        rgb_topic: str,
        depth_topic: str,
        node: Node,
        camera_info_topic: Optional[str] = None,
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
            queue_size: Queue size for TimeSynchronizer
            slop: DEPRECATED - TimeSynchronizer requires exact timestamp matching (parameter kept for compatibility, not used)
        """
        self.camera_id = camera_id
        self.node = node
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
        
        # Camera intrinsics (stored from synchronized camera_info messages)
        self._intrinsics: Optional[Dict] = None
        
        # Create subscribers (camera_info is required for synchronization)
        if camera_info_topic is None:
            raise ValueError(f"camera_info_topic is required for camera {camera_id}")
        
        # Create and store subscribers for camera_info, RGB, and depth (for synchronization)
        # This is critical: subscribers must be kept alive for the synchronizer to work
        self._camera_info_sub = message_filters.Subscriber(node, CameraInfo, camera_info_topic)
        self._rgb_sub = message_filters.Subscriber(node, Image, rgb_topic)
        self._depth_sub = message_filters.Subscriber(node, Image, depth_topic)
        
        # Synchronize camera_info, RGB, and depth (three-way synchronization)
        # Use TimeSynchronizer for exact timestamp matching
        self._sync = message_filters.TimeSynchronizer(
            [self._camera_info_sub, self._rgb_sub, self._depth_sub],
            queue_size=queue_size
        )
        self._sync.registerCallback(self._sync_callback)
    
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
            # TimeSynchronizer ensures messages have identical timestamps
            # Use camera_info timestamp as primary timestamp
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
            
            # Extract intrinsics from camera_info
            if len(camera_info_msg.k) == 9:
                K = np.array(camera_info_msg.k).reshape(3, 3)
            else:
                K = None
            
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
            
            # Get ROS time at end of callback (after processing)
            ros_time_end = self.node.get_clock().now()
            end_t_ns = ros_time_to_ns(ros_time_end)
            
            # Build data_dict in droid format
            data_dict = {
                "image": {self.camera_id: rgb_cv},
                "depth": {self.camera_id: depth_cv}
            }
            
            # Build timestamp_dict in droid format (all in nanoseconds)
            # TimeSynchronizer ensures messages have identical timestamps
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
                # Store intrinsics (they may change, but typically don't)
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
            
            # Track callback count (for internal use)
            self._callback_count += 1
                        
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
        target_timestamp_ns: int
    ) -> Optional[Tuple[Dict, Dict]]:
        """
        Get camera data for a specific timestamp key from the queue.

        Only returns data if there is an exact timestamp match.
        No tolerance is used.

        Args:
            target_timestamp_ns: Target timestamp in nanoseconds (key to search for)
        
        Returns:
            Tuple[dict, dict]: (data_dict, timestamp_dict) if found, None otherwise
        """
        with self._data_lock:
            if not self._data_queue:
                return None

            if target_timestamp_ns in self._data_queue:
                data_dict, timestamp_dict = self._data_queue[target_timestamp_ns]
                return data_dict, timestamp_dict
            else:
                self.node.get_logger().warning(
                    f"[CameraReader {self.camera_id}] No data found for timestamp {target_timestamp_ns}."
                )

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
