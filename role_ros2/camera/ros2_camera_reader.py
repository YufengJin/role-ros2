"""ROS2 camera reader for single camera using ApproximateTimeSynchronizer with event-driven architecture."""

import threading
from typing import Dict, Tuple, Optional, Callable
from collections import deque
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import message_filters
from tf2_ros import Buffer, TransformListener
import tf2_ros

from role_ros2.misc.ros2_utils import get_ros_time_ns, transform_to_matrix


# Sensor Data QoS profile for reliable sensor data reception
SENSOR_DATA_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10
)


class ROS2CameraReader:
    """
    ROS2 camera reader for a single camera with event-driven architecture.
    
    Uses ApproximateTimeSynchronizer with SensorDataQoS for robust synchronization.
    Implements lazy decoding - stores raw messages and decodes only when data is read.
    Supports callback mechanism to notify when new data arrives.
    """
    
    def __init__(
        self,
        camera_id: str,
        rgb_topic: str,
        depth_topic: str,
        node: Node,
        camera_info_topic: Optional[str] = None,
        queue_size: int = 10,
        slop: float = 0.05,
        base_frame: Optional[str] = None,
        camera_frame: Optional[str] = None,
        on_data_received_callback: Optional[Callable[[str, int], None]] = None
    ):
        """
        Initialize ROS2 camera reader with event-driven architecture.
        
        Args:
            camera_id: Camera identifier (e.g., serial number or name)
            rgb_topic: RGB image topic
            depth_topic: Depth image topic
            node: ROS2 node to use for subscriptions
            camera_info_topic: Camera info topic (required for synchronization)
            queue_size: Queue size for ApproximateTimeSynchronizer
            slop: Time tolerance for ApproximateTimeSynchronizer (default: 0.05s = 50ms)
            base_frame: Base frame for TF lookup (e.g., "base_link")
            camera_frame: Camera frame for TF lookup
            on_data_received_callback: Callback function called when new data arrives.
                Signature: callback(camera_id: str, timestamp_ns: int) -> None
        """
        self.camera_id = camera_id
        self.node = node
        self.cv_bridge = CvBridge()
        self.base_frame = base_frame
        self.camera_frame = camera_frame
        self.slop = slop
        
        # Event-driven callback
        self.on_data_received_callback = on_data_received_callback
        
        # Initialize callback count for logging
        self._callback_count = 0
        
        # Thread-safe storage for raw messages (lazy decoding)
        # Queue stores: {timestamp_ns: (camera_info_msg, rgb_msg, depth_msg, sub_t_ns)}
        self._raw_data_queue: Dict[int, Tuple[CameraInfo, Image, Image, int]] = {}
        self._timestamp_order: deque = deque(maxlen=100)
        self._data_lock = threading.Lock()
        self._queue_max_size = 100
        
        # Cache for decoded data to avoid re-decoding
        self._decoded_cache: Dict[int, Tuple[Dict, Dict]] = {}
        self._cache_lock = threading.Lock()
        
        # Event to notify when new data is available
        self._data_available_event = threading.Event()
        
        # Camera intrinsics (stored from synchronized camera_info messages)
        self._intrinsics: Optional[Dict] = None
        
        # Track last message reception time for is_running() check
        self._last_message_time_ns: Optional[int] = None
        self._message_timeout_ns = 2_000_000_000  # 2 seconds timeout
        
        # Create ReentrantCallbackGroup for parallel callback execution
        # This allows camera callbacks to run concurrently with robot state callbacks
        # eliminating serial execution bottleneck
        self._cb_group = ReentrantCallbackGroup()
        
        # Initialize TF2 buffer and listener for extrinsic lookup
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, node)
        
        # Create subscribers (camera_info is required for synchronization)
        if camera_info_topic is None:
            raise ValueError(f"camera_info_topic is required for camera {camera_id}")
        
        # Create subscribers with SensorDataQoS for reliable sensor data reception
        # Use ReentrantCallbackGroup for parallel execution
        self._camera_info_sub = message_filters.Subscriber(
            node, CameraInfo, camera_info_topic, 
            callback_group=self._cb_group, qos_profile=SENSOR_DATA_QOS
        )
        self._rgb_sub = message_filters.Subscriber(
            node, Image, rgb_topic, 
            callback_group=self._cb_group, qos_profile=SENSOR_DATA_QOS
        )
        self._depth_sub = message_filters.Subscriber(
            node, Image, depth_topic, 
            callback_group=self._cb_group, qos_profile=SENSOR_DATA_QOS
        )
        
        # Use ApproximateTimeSynchronizer with configurable slop for robustness
        self._sync = message_filters.ApproximateTimeSynchronizer(
            [self._camera_info_sub, self._rgb_sub, self._depth_sub],
            queue_size=queue_size,
            slop=slop
        )
        self._sync.registerCallback(self._sync_callback)
    
    def set_on_data_received_callback(
        self,
        callback: Optional[Callable[[str, int], None]]
    ) -> None:
        """
        Set or update the callback for when new data is received.
        
        Args:
            callback: Callback function with signature (camera_id, timestamp_ns) -> None
        """
        self.on_data_received_callback = callback
    
    def _sync_callback(
        self,
        camera_info_msg: CameraInfo,
        rgb_msg: Image,
        depth_msg: Image
    ) -> None:
        """
        Callback for synchronized camera_info, RGB, and depth images.
        
        Implements lazy decoding - stores raw messages without processing.
        Triggers the on_data_received_callback for event-driven processing.
        
        Args:
            camera_info_msg: Camera info message
            rgb_msg: RGB image message
            depth_msg: Depth image message
        """
        try:
            # Get ROS time at subscription (for latency tracking)
            sub_t_ns = get_ros_time_ns(self.node)
            
            # Get published timestamp from message header
            pub_t_ns = int(
                camera_info_msg.header.stamp.sec * 1_000_000_000 +
                camera_info_msg.header.stamp.nanosec
            )
            
            # Store raw messages (lazy decoding - don't process images yet)
            with self._data_lock:
                # Update last message reception time
                self._last_message_time_ns = sub_t_ns
                
                # Store raw messages
                self._raw_data_queue[pub_t_ns] = (
                    camera_info_msg,
                    rgb_msg,
                    depth_msg,
                    sub_t_ns
                )
                self._timestamp_order.append(pub_t_ns)
                
                # Remove oldest entries if queue exceeds max size
                while len(self._raw_data_queue) > self._queue_max_size:
                    if self._timestamp_order:
                        oldest_timestamp = self._timestamp_order.popleft()
                        if oldest_timestamp in self._raw_data_queue:
                            del self._raw_data_queue[oldest_timestamp]
                        # Also clear decoded cache for this timestamp
                        with self._cache_lock:
                            if oldest_timestamp in self._decoded_cache:
                                del self._decoded_cache[oldest_timestamp]
            
            # Notify waiting threads
            self._data_available_event.set()
            self._data_available_event.clear()
            
            # Track callback count
            self._callback_count += 1
            
            # Trigger event-driven callback (AFTER storing data)
            if self.on_data_received_callback is not None:
                try:
                    self.on_data_received_callback(self.camera_id, pub_t_ns)
                except Exception as e:
                    self.node.get_logger().error(
                        f"Error in on_data_received_callback for camera {self.camera_id}: {e}"
                    )
                        
        except Exception as e:
            self.node.get_logger().error(
                f"Error in sync_callback for camera {self.camera_id}: {e}"
            )
            import traceback
            self.node.get_logger().error(traceback.format_exc())
    
    def _decode_data(
        self,
        pub_t_ns: int,
        camera_info_msg: CameraInfo,
        rgb_msg: Image,
        depth_msg: Image,
        sub_t_ns: int
    ) -> Tuple[Dict, Dict]:
        """
        Decode raw messages into data_dict and timestamp_dict.
        
        This is called lazily when data is actually requested.
        
        Args:
            pub_t_ns: Published timestamp in nanoseconds
            camera_info_msg: Camera info message
            rgb_msg: RGB image message
            depth_msg: Depth image message
            sub_t_ns: Subscription timestamp in nanoseconds
            
        Returns:
            Tuple of (data_dict, timestamp_dict)
        """
        # Get ROS time at decode (end processing time)
        end_t_ns = get_ros_time_ns(self.node)
        
        # Convert images
        rgb_cv = self.cv_bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
        depth_cv = self.cv_bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        
        # Convert depth to uint16 if needed
        if depth_cv.dtype == np.float32:
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
        
        # Update intrinsics (thread-safe)
        with self._data_lock:
            self._intrinsics = intrinsics
        
        # Build data_dict in droid format
        data_dict = {
            "image": {self.camera_id: rgb_cv},
            "depth": {self.camera_id: depth_cv}
        }
        
        # Build timestamp_dict
        timestamp_dict = {
            self.camera_id + "_pub_t": pub_t_ns,
            self.camera_id + "_sub_t": sub_t_ns,
            self.camera_id + "_end_t": end_t_ns,
        }
        
        return data_dict, timestamp_dict
    
    def _get_decoded_data(self, pub_t_ns: int) -> Optional[Tuple[Dict, Dict]]:
        """
        Get decoded data for a timestamp, using cache if available.
        
        Args:
            pub_t_ns: Published timestamp in nanoseconds
            
        Returns:
            Tuple of (data_dict, timestamp_dict) or None if not found
        """
        # Check cache first
        with self._cache_lock:
            if pub_t_ns in self._decoded_cache:
                return self._decoded_cache[pub_t_ns]
        
        # Get raw data
        with self._data_lock:
            if pub_t_ns not in self._raw_data_queue:
                return None
            raw_data = self._raw_data_queue[pub_t_ns]
        
        # Decode (outside lock to avoid blocking other operations)
        camera_info_msg, rgb_msg, depth_msg, sub_t_ns = raw_data
        decoded = self._decode_data(
            pub_t_ns, camera_info_msg, rgb_msg, depth_msg, sub_t_ns
        )
        
        # Cache the decoded data
        with self._cache_lock:
            self._decoded_cache[pub_t_ns] = decoded
        
        return decoded
    
    def read_camera(self) -> Tuple[Dict, Dict]:
        """
        Read latest camera data in droid format.
        
        Performs lazy decoding - only decodes when this method is called.
        
        Returns:
            Tuple[dict, dict]: (data_dict, timestamp_dict)
        """
        with self._data_lock:
            if not self._timestamp_order:
                return {}, {}
            latest_timestamp = self._timestamp_order[-1]
        
        decoded = self._get_decoded_data(latest_timestamp)
        if decoded is None:
            return {}, {}
        return decoded
    
    def get_intrinsics(self) -> Optional[Dict]:
        """
        Get camera intrinsics.
        
        Returns:
            Dictionary with intrinsics information, or None if not available
        """
        with self._data_lock:
            return self._intrinsics
    
    def is_running(self) -> bool:
        """
        Check if camera is running (actively receiving messages).
        
        Returns:
            True if camera has received data recently (within timeout period)
        """
        with self._data_lock:
            if self._last_message_time_ns is None:
                return False
            
            current_time_ns = get_ros_time_ns(self.node)
            time_since_last_msg = current_time_ns - self._last_message_time_ns
            
            return time_since_last_msg < self._message_timeout_ns
    
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
    
    def has_data_for_timestamp(self, target_timestamp_ns: int) -> bool:
        """
        Check if data exists for a specific timestamp (without decoding).
        
        This is a fast check used by the event-driven sync mechanism.
        
        Args:
            target_timestamp_ns: Target timestamp in nanoseconds
            
        Returns:
            True if data exists for this timestamp
        """
        with self._data_lock:
            return target_timestamp_ns in self._raw_data_queue
    
    def get_data_for_timestamp(
        self,
        target_timestamp_ns: int
    ) -> Optional[Tuple[Dict, Dict]]:
        """
        Get camera data for a specific timestamp (with lazy decoding).
        
        Args:
            target_timestamp_ns: Target timestamp in nanoseconds
        
        Returns:
            Tuple[dict, dict]: (data_dict, timestamp_dict) if found, None otherwise
        """
        # Check if data exists
        with self._data_lock:
            if target_timestamp_ns not in self._raw_data_queue:
                self.node.get_logger().debug(
                    f"[CameraReader {self.camera_id}] No data for timestamp {target_timestamp_ns}. "
                    f"Available: {len(self._timestamp_order)} entries."
                )
                return None
        
        # Decode and return
        return self._get_decoded_data(target_timestamp_ns)
    
    def wait_for_data(self, timeout_sec: float = 1.0) -> bool:
        """
        Wait for new data to become available.
        
        Args:
            timeout_sec: Maximum time to wait in seconds
            
        Returns:
            True if data became available, False if timeout
        """
        return self._data_available_event.wait(timeout=timeout_sec)
    
    def get_extrinsic(
        self,
        timestamp_dict: Optional[Dict] = None
    ) -> Optional[np.ndarray]:
        """
        Get camera extrinsic (transformation matrix from base_frame to camera_frame).
        
        Args:
            timestamp_dict: Optional timestamp dictionary for TF lookup
        
        Returns:
            4x4 transformation matrix or None if not available
        """
        if self.base_frame is None or self.camera_frame is None:
            self.node.get_logger().warning(
                f"[CameraReader {self.camera_id}] Cannot get extrinsic: "
                f"base_frame={self.base_frame}, camera_frame={self.camera_frame} not set"
            )
            return None
        
        # Determine timestamp to use
        timestamp_ns = None
        if timestamp_dict is not None:
            pub_t_key = self.camera_id + "_pub_t"
            if pub_t_key in timestamp_dict:
                timestamp_ns = timestamp_dict[pub_t_key]
            elif "pub_t" in timestamp_dict:
                timestamp_ns = timestamp_dict["pub_t"]
        
        if timestamp_ns is None:
            latest_timestamp = self.get_latest_pub_timestamp()
            if latest_timestamp is None:
                self.node.get_logger().warning(
                    f"[CameraReader {self.camera_id}] Cannot get extrinsic: no timestamp available"
                )
                return None
            timestamp_ns = latest_timestamp
        
        # Convert to ROS Time
        try:
            ros_time = Time(nanoseconds=timestamp_ns)
        except Exception as e:
            self.node.get_logger().warning(
                f"[CameraReader {self.camera_id}] Failed to convert timestamp: {e}"
            )
            return None
        
        # Lookup transform
        try:
            transform = self._tf_buffer.lookup_transform(
                self.base_frame,
                self.camera_frame,
                ros_time,
                timeout=Duration(seconds=0.1)
            )
        except tf2_ros.TransformException as e:
            self.node.get_logger().warning(
                f"[CameraReader {self.camera_id}] TF lookup failed: {e}"
            )
            return None
        except Exception as e:
            self.node.get_logger().warning(
                f"[CameraReader {self.camera_id}] Unexpected TF error: {e}"
            )
            return None
        
        # Convert to matrix
        try:
            return transform_to_matrix(transform)
        except Exception as e:
            self.node.get_logger().warning(
                f"[CameraReader {self.camera_id}] Failed to convert transform: {e}"
            )
            return None
