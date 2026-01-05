"""Multi-camera wrapper for role-ros2 using ROS2."""

import random
import yaml
from pathlib import Path
from typing import Dict, Tuple, Optional, List
from collections import defaultdict
import threading

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import message_filters

from role_ros2.camera.ros2_camera_reader import ROS2CameraReader
from role_ros2.misc.ros2_utils import ros_time_to_ns
from role_ros2.misc.config_loader import get_package_config_path, load_yaml_config


class MultiCameraWrapper:
    """
    ROS2-based multi-camera wrapper.
    
    Uses ROS2 topics and layered synchronization to read camera data.
    Supports ZED cameras via ROS2 topics.
    
    Synchronization Architecture (layered, decoupled):
    - Layer 1: Each camera internally uses TimeSynchronizer (in ROS2CameraReader)
      to synchronize CameraInfo, RGB, and Depth with exact timestamp matching
    - Layer 2: MultiCameraWrapper uses ApproximateTimeSynchronizer on camera_info
      messages from all cameras to synchronize across cameras
    - Decoupled Processing: Layer 2 callback only records sync requests, does not wait.
      A separate timer callback (_check_pending_sync_requests) periodically checks if
      data is ready and processes requests when all cameras have data available.
      This avoids blocking the executor and prevents deadlocks.
    
    For synchronized mode (use_sync=True):
    - Uses layered synchronization with decoupled processing
    - Layer 2 callback records sync requests without blocking
    - Timer callback (10ms period) checks and processes requests when data is ready
    - Records data ready time for latency analysis
    - Ensures all cameras are time-aligned within ApproximateTimeSynchronizer slop
    
    For non-synchronized mode (use_sync=False):
    - Uses ROS2CameraReader for each camera (maintains backward compatibility)
    - Returns latest data from each camera (may not be time-synchronized)
    """
    
    def __init__(
        self,
        node: Optional[Node] = None,
        config_file: Optional[str] = None
    ):
        """
        Initialize multi-camera wrapper.
        
        Args:
            node: Optional ROS2 node (if None, creates a new node)
            config_file: Path to YAML configuration file. If None, uses default config path.
        """
        
        # Use provided node or create a new one
        self._own_node = node is None
        if node is None:
            if not rclpy.ok():
                rclpy.init()
            self._node = Node('multi_camera_wrapper')
        else:
            self._node = node
        
        # Initialize camera dictionary (for non-sync mode)
        self.camera_dict: Dict[str, ROS2CameraReader] = {}
        
        # Camera configurations (for sync mode)
        self.camera_configs: List[Dict] = []
        
        # CV bridge for image conversion
        self.cv_bridge = CvBridge()
        
        # Thread-safe storage for synchronized multi-camera data
        self._latest_sync_data_dict: Optional[Dict] = None
        self._latest_sync_timestamp_dict: Optional[Dict] = None
        self._sync_data_lock = threading.Lock()
        
        # Pending sync requests queue (for decoupled processing)
        # Stores: (request_id, camera_id_list, camera_info_timestamps, callback_start_time_ns)
        self._pending_sync_requests: List[Tuple[int, List[str], List[int], int]] = []
        self._pending_requests_lock = threading.Lock()
        self._sync_request_counter = 0
        self._max_pending_requests = 10  # Limit queue size
        
        # Load camera configurations from config/ directory
        if config_file is None:
            # Use unified config loader to find config file in config/ directory
            config_file = get_package_config_path('multi_camera_reader_config.yaml')
        else:
            config_file = Path(config_file)
        
        camera_configs = self._load_config(config_file)
        self.camera_configs = camera_configs
        
        if not camera_configs:
            self._node.get_logger().warning("No cameras configured!")
            return
        
        # Create camera readers for non-sync mode (backward compatibility)
        for config in camera_configs:
            camera_id = config["camera_id"]
            rgb_topic = config["rgb_topic"]
            depth_topic = config["depth_topic"]
            camera_info_topic = config.get("camera_info_topic", None)
            
            try:
                # Get slop from config or use default (0.005s for RealSense compatibility)
                slop = config.get("slop", 0.005)
                queue_size = config.get("queue_size", 10)
                
                camera_reader = ROS2CameraReader(
                    camera_id=camera_id,
                    rgb_topic=rgb_topic,
                    depth_topic=depth_topic,
                    node=self._node,
                    camera_info_topic=camera_info_topic,
                    queue_size=queue_size,
                    slop=slop
                )
                self.camera_dict[camera_id] = camera_reader
            except Exception as e:
                self._node.get_logger().error(
                    f"Failed to initialize camera {camera_id}: {e}"
                )
        
        # Setup layered multi-camera synchronization using ApproximateTimeSynchronizer
        # Layer 1: Each camera uses TimeSynchronizer internally (in ROS2CameraReader)
        # Layer 2: MultiCameraWrapper uses ApproximateTimeSynchronizer on camera_info messages
        self._setup_layered_multi_camera_sync(camera_configs)
    
    def _load_config(self, config_file: Path) -> List[Dict]:
        """
        Load camera configurations from YAML file.
        
        Args:
            config_file: Path to YAML configuration file
            
        Returns:
            List of camera configuration dictionaries
        """
        if not config_file.exists():
            self._node.get_logger().error(
                f"Camera config file not found: {config_file}. "
                f"Using empty camera list."
            )
            return []
        
        try:
            with open(config_file, 'r') as f:
                config_data = yaml.safe_load(f)
            
            # Get global base_frame if specified
            global_base_frame = config_data.get("base_frame", "base_link")
            
            # Get global sync settings
            global_sync = config_data.get("global_sync", {})
            global_slop = global_sync.get("slop", 0.2)  # Default 0.2s for bag replay
            global_queue_size = global_sync.get("queue_size", 10)
            
            # Get camera configurations
            cameras = config_data.get("cameras", [])
            camera_configs = []
            
            for camera in cameras:
                camera_config = {
                    "camera_id": camera["camera_id"],
                    "rgb_topic": camera["rgb_topic"],
                    "depth_topic": camera["depth_topic"],
                    "camera_info_topic": camera.get("camera_info_topic", None),
                    "base_frame": camera.get("base_frame", global_base_frame),
                    "camera_frame": camera.get("camera_frame", None),
                    # Use camera-specific slop if provided, otherwise use global slop, 
                    # otherwise default to 0.1s (100ms) for ZED camera compatibility
                    "slop": camera.get("slop", global_slop if global_slop else 0.1),
                    "queue_size": camera.get("queue_size", global_queue_size)
                }
                camera_configs.append(camera_config)
            
            return camera_configs
            
        except Exception as e:
            self._node.get_logger().error(
                f"Error loading camera config file {config_file}: {e}"
            )
            return []
    
    def _setup_layered_multi_camera_sync(self, camera_configs: List[Dict]) -> None:
        """
        Setup layered ApproximateTimeSynchronizer for multiple cameras.
        
        Layer 1: Each camera internally uses TimeSynchronizer (in ROS2CameraReader)
        Layer 2: MultiCameraWrapper uses ApproximateTimeSynchronizer on camera_info messages
        
        Uses event-based waiting mechanism to handle race conditions between layers.
        No rclpy.spin_once in callbacks to avoid deadlocks.
        
        Args:
            camera_configs: List of camera configuration dictionaries
        """
        # Get global sync settings from config file
        global_sync = {}
        try:
            config_data = load_yaml_config('multi_camera_config.yaml')
            global_sync = config_data.get("global_sync", {})
        except Exception as e:
            self._node.get_logger().warning(f"Failed to load global sync settings: {e}")
        
        # Use global sync settings for multi-camera synchronization
        # Increased slop for better tolerance (default: 0.2s = 200ms)
        multi_camera_slop = global_sync.get("multi_camera_slop", global_sync.get("slop", 0.2))
        multi_camera_queue_size = global_sync.get("multi_camera_queue_size", global_sync.get("queue_size", 10))
        
        # Store sync settings for use in callback
        self._multi_camera_slop = multi_camera_slop
        self._multi_camera_slop_ns = int(multi_camera_slop * 1_000_000_000)
        
        # Store max wait time (increased for better reliability)
        self._max_wait_time = global_sync.get("max_wait_time", 0.5)  # Default: 500ms
        
        # Use ReentrantCallbackGroup to allow concurrent callback execution
        callback_group = ReentrantCallbackGroup()
        
        # Create subscribers for each camera's camera_info
        camera_info_subs = []
        camera_id_list = []
        
        for config in camera_configs:
            camera_id = config["camera_id"]
            camera_info_topic = config.get("camera_info_topic", None)
            
            if camera_info_topic is None:
                self._node.get_logger().warning(
                    f"Camera {camera_id} has no camera_info_topic, skipping multi-camera sync"
                )
                continue
            
            # Create subscriber for this camera's camera_info with ReentrantCallbackGroup
            camera_info_sub = message_filters.Subscriber(
                self._node, CameraInfo, camera_info_topic, callback_group=callback_group
            )
            camera_info_subs.append(camera_info_sub)
            camera_id_list.append(camera_id)
        
        if len(camera_info_subs) < 2:
            self._node.get_logger().warning(
                f"Only {len(camera_info_subs)} camera(s) available, "
                f"multi-camera synchronization requires at least 2 cameras"
            )
            return
        
        # Create ApproximateTimeSynchronizer for multiple cameras
        self._multi_camera_sync = message_filters.ApproximateTimeSynchronizer(
            camera_info_subs,
            queue_size=multi_camera_queue_size,
            slop=multi_camera_slop
        )
        self._multi_camera_sync.registerCallback(
            lambda *args: self._layered_sync_callback(camera_id_list, *args)
        )
        
        # Create timer to periodically check pending sync requests (decoupled processing)
        # Check every 10ms to ensure timely processing
        self._sync_check_timer = self._node.create_timer(
            0.01,  # 10ms period
            self._check_pending_sync_requests,
            callback_group=callback_group
        )
    
    def _layered_sync_callback(self, camera_id_list: List[str], *camera_info_msgs: CameraInfo) -> None:
        """
        Callback when multiple camera_info messages are synchronized (Layer 2).
        
        Decoupled approach: Only records the sync request, does not wait for data.
        A separate timer callback (_check_pending_sync_requests) will process requests
        when data becomes available. This avoids blocking the executor.
        
        Args:
            camera_id_list: List of camera IDs in the same order as camera_info_msgs
            *camera_info_msgs: Synchronized CameraInfo messages from all cameras
        """
        try:
            # Get ROS time at start of callback (using ROS clock for sim time support)
            ros_time_start = self._node.get_clock().now()
            start_t_ns = ros_time_to_ns(ros_time_start)
            
            # Extract timestamps from camera_info messages
            camera_info_timestamps = []
            for camera_info_msg in camera_info_msgs:
                ts_ns = int(
                    camera_info_msg.header.stamp.sec * 1_000_000_000 +
                    camera_info_msg.header.stamp.nanosec
                )
                camera_info_timestamps.append(ts_ns)
            
            # Add sync request to pending queue (decoupled processing)
            with self._pending_requests_lock:
                self._sync_request_counter += 1
                request_id = self._sync_request_counter
                
                # Limit queue size to prevent memory issues
                if len(self._pending_sync_requests) >= self._max_pending_requests:
                    # Remove oldest request
                    self._pending_sync_requests.pop(0)
                    self._node.get_logger().warning(
                        f"[MultiCamera] Pending sync request queue full, removing oldest request"
                    )
                
                # Add new request
                self._pending_sync_requests.append((
                    request_id,
                    camera_id_list.copy(),
                    camera_info_timestamps.copy(),
                    start_t_ns
                ))
            
        except Exception as e:
            self._node.get_logger().error(f"Error in layered sync callback: {e}")
            import traceback
            self._node.get_logger().error(traceback.format_exc())
    
    def _check_pending_sync_requests(self) -> None:
        """
        Timer callback to check pending sync requests and process them when data is ready.
        
        This runs periodically (every 10ms) and checks if data for pending sync requests
        has become available. When data is ready, it processes the request and stores
        the synchronized data. Records data ready time for latency analysis.
        """
        with self._pending_requests_lock:
            if not self._pending_sync_requests:
                return
            
            # Process requests (from oldest to newest)
            requests_to_remove = []
            
            for idx, (request_id, camera_id_list, camera_info_timestamps, callback_start_t_ns) in enumerate(self._pending_sync_requests):
                # Get ROS time when checking (data ready time)
                ros_time_check = self._node.get_clock().now()
                check_t_ns = ros_time_to_ns(ros_time_check)
                
                # Try to collect data for all cameras
                sync_data_dict = defaultdict(dict)
                sync_timestamp_dict = {}
                cameras_with_data = 0
                
                # Check each camera's data
                all_cameras_ready = True
                for camera_id, camera_info_t_ns in zip(camera_id_list, camera_info_timestamps):
                    camera_reader = self.camera_dict.get(camera_id)
                    if camera_reader is None:
                        all_cameras_ready = False
                        break
                    
                    # Try to get data for this timestamp (exact match only, no tolerance)
                    # get_data_for_timestamp will log a warning if data is not found
                    camera_data = camera_reader.get_data_for_timestamp(camera_info_t_ns)
                    
                    if camera_data is None:
                        # Data not ready yet, skip this request for now
                        all_cameras_ready = False
                        break
                    
                    data_dict, timestamp_dict = camera_data
                    
                    # Validate data
                    if not data_dict or "image" not in data_dict:
                        all_cameras_ready = False
                        break
                    
                    if camera_id not in data_dict.get("image", {}):
                        all_cameras_ready = False
                        break
                    
                    # Merge data_dict into sync_data_dict
                    for key in data_dict:
                        sync_data_dict[key].update(data_dict[key])
                    
                    # Merge timestamp_dict
                    sync_timestamp_dict.update(timestamp_dict)
                    
                    cameras_with_data += 1
                
                # Only process if all cameras have data ready
                if not all_cameras_ready:
                    # Check if request is too old (timeout)
                    request_age_ms = (check_t_ns - callback_start_t_ns) / 1e6
                    if request_age_ms > self._max_wait_time * 1000:
                        # Request timed out, remove it
                        self._node.get_logger().warning(
                            f"[MultiCamera] ⚠️ Sync request #{request_id} timed out "
                            f"(age={request_age_ms:.2f}ms > {self._max_wait_time*1000:.2f}ms)"
                        )
                        requests_to_remove.append(idx)
                    # Otherwise, keep request for next check
                    continue
                
                # Process request if all cameras have data ready
                # Get ROS time when data is ready (data ready time)
                ros_time_ready = self._node.get_clock().now()
                ready_t_ns = ros_time_to_ns(ros_time_ready)
                
                # Calculate latencies
                callback_to_check_ms = (check_t_ns - callback_start_t_ns) / 1e6
                callback_to_ready_ms = (ready_t_ns - callback_start_t_ns) / 1e6
                
                # Get ROS time at end of processing
                ros_time_end = self._node.get_clock().now()
                end_t_ns = ros_time_to_ns(ros_time_end)
                
                # Store synchronized data (thread-safe)
                with self._sync_data_lock:
                    self._latest_sync_data_dict = sync_data_dict
                    self._latest_sync_timestamp_dict = sync_timestamp_dict
                    
                    # Add multi-camera sync metadata
                    self._latest_sync_timestamp_dict["multi_camera_sync_start"] = callback_start_t_ns
                    self._latest_sync_timestamp_dict["multi_camera_sync_check"] = check_t_ns  # When we started checking
                    self._latest_sync_timestamp_dict["multi_camera_sync_ready"] = ready_t_ns  # When data became ready
                    self._latest_sync_timestamp_dict["multi_camera_sync_end"] = end_t_ns
                    self._latest_sync_timestamp_dict["multi_camera_sync_cameras_count"] = cameras_with_data
                
                # Mark this request for removal
                requests_to_remove.append(idx)
            
            # Remove processed/timed-out requests (in reverse order to maintain indices)
            for idx in reversed(requests_to_remove):
                self._pending_sync_requests.pop(idx)
    
    def _wait_for_camera_data(
        self,
        camera_reader: ROS2CameraReader,
        camera_id: str,
        target_timestamp_ns: int,
        tolerance_ns: int,
        max_wait_time: float = 0.5
    ) -> Optional[Tuple[Dict, Dict]]:
        """
        DEPRECATED: This method is no longer used in the decoupled architecture.
        Kept for backward compatibility but should not be called.
        
        In the new decoupled architecture, sync requests are processed by
        _check_pending_sync_requests timer callback.
        """
        """
        Wait for camera data to be available with matching timestamp.
        
        Uses threading.Event and ROS clock (not rclpy.spin_once) to avoid deadlocks.
        Supports ROS simulation time.
        
        Args:
            camera_reader: ROS2CameraReader instance
            camera_id: Camera identifier
            target_timestamp_ns: Target timestamp in nanoseconds
            tolerance_ns: Timestamp tolerance in nanoseconds
            max_wait_time: Maximum wait time in seconds (default: 500ms)
        
        Returns:
            Tuple[dict, dict]: (data_dict, timestamp_dict) if found, None otherwise
        """
        # Get ROS time at start (using ROS clock for sim time support)
        ros_time_start = self._node.get_clock().now()
        wait_start_ns = ros_time_to_ns(ros_time_start)
        max_wait_ns = int(max_wait_time * 1_000_000_000)
        
        check_count = 0
        check_interval = 10  # Log every N checks
        
        # Check initial state
        latest_timestamp = camera_reader.get_latest_pub_timestamp()
        if latest_timestamp is not None:
            initial_diff_ns = abs(latest_timestamp - target_timestamp_ns)
            initial_diff_ms = initial_diff_ns / 1e6
            self._node.get_logger().debug(
                f"[Wait {camera_id}] Initial state: "
                f"latest_pub_t={latest_timestamp} ({latest_timestamp/1e9:.9f}s), "
                f"target_t={target_timestamp_ns} ({target_timestamp_ns/1e9:.9f}s), "
                f"diff={initial_diff_ms:.2f}ms, tolerance={tolerance_ns/1e6:.2f}ms"
            )
        else:
            self._node.get_logger().debug(
                f"[Wait {camera_id}] Initial state: No data available yet"
            )
        
        # Wait loop using ROS clock and threading.Event (no rclpy.spin_once)
        while True:
            # Check elapsed time using ROS clock (supports sim time)
            ros_time_now = self._node.get_clock().now()
            wait_now_ns = ros_time_to_ns(ros_time_now)
            elapsed_ns = wait_now_ns - wait_start_ns
            
            if elapsed_ns >= max_wait_ns:
                break
            
            # Try to get data for this timestamp
            camera_data = camera_reader.get_data_for_timestamp(
                target_timestamp_ns, tolerance_ns=tolerance_ns
            )
            
            if camera_data is not None:
                data_dict, timestamp_dict = camera_data
                # Validate that data actually exists
                if data_dict and "image" in data_dict and camera_id in data_dict.get("image", {}):
                    elapsed_ms = elapsed_ns / 1e6
                    found_timestamp = camera_reader.get_latest_pub_timestamp()
                    if found_timestamp is not None:
                        latency_ns = abs(found_timestamp - target_timestamp_ns)
                        latency_ms = latency_ns / 1e6
                        self._node.get_logger().info(
                            f"[Wait {camera_id}] ✅ Data found: "
                            f"checks={check_count}, wait_time={elapsed_ms:.2f}ms, "
                            f"found_timestamp={found_timestamp} ({found_timestamp/1e9:.9f}s), "
                            f"latency={latency_ms:.2f}ms (target={target_timestamp_ns/1e9:.9f}s)"
                        )
                    else:
                        self._node.get_logger().info(
                            f"[Wait {camera_id}] ✅ Data found: "
                            f"checks={check_count}, wait_time={elapsed_ms:.2f}ms"
                        )
                    return camera_data
            
            # Log progress every N checks
            if check_count % check_interval == 0 and check_count > 0:
                elapsed_ms = elapsed_ns / 1e6
                latest_timestamp = camera_reader.get_latest_pub_timestamp()
                if latest_timestamp is not None:
                    diff_ns = abs(latest_timestamp - target_timestamp_ns)
                    diff_ms = diff_ns / 1e6
                    self._node.get_logger().debug(
                        f"[Wait {camera_id}] Still waiting: "
                        f"checks={check_count}, elapsed={elapsed_ms:.2f}ms, "
                        f"latest_t={latest_timestamp} ({latest_timestamp/1e9:.9f}s), "
                        f"diff={diff_ms:.2f}ms"
                    )
                else:
                    self._node.get_logger().debug(
                        f"[Wait {camera_id}] Still waiting: "
                        f"checks={check_count}, elapsed={elapsed_ms:.2f}ms, no data yet"
                    )
            
            # Wait for data using Event (non-blocking, allows executor to process other callbacks)
            # Use short timeout to periodically check elapsed time
            remaining_time = (max_wait_ns - elapsed_ns) / 1e9
            if remaining_time > 0:
                # Wait for data event with short timeout (allows periodic checks)
                wait_timeout = min(0.01, remaining_time)  # 10ms or remaining time
                camera_reader.wait_for_data(timeout_sec=wait_timeout)
            
            check_count += 1
        
        # If still no data, try one more time with latest data
        # (in case timestamp doesn't match exactly but is close)
        elapsed_ms = (ros_time_to_ns(self._node.get_clock().now()) - wait_start_ns) / 1e6
        self._node.get_logger().warning(
            f"[Wait {camera_id}] ⚠️ Max wait time reached: "
            f"checks={check_count}, elapsed={elapsed_ms:.2f}ms, trying latest data"
        )
        
        camera_data = camera_reader.read_camera()
        if camera_data and camera_data[0] and "image" in camera_data[0]:
            if camera_id in camera_data[0].get("image", {}):
                latest_timestamp = camera_reader.get_latest_pub_timestamp()
                if latest_timestamp is not None:
                    diff_ns = abs(latest_timestamp - target_timestamp_ns)
                    diff_ms = diff_ns / 1e6
                    if diff_ns <= tolerance_ns:
                        self._node.get_logger().info(
                            f"[Wait {camera_id}] ✅ Using latest data: "
                            f"timestamp_diff={diff_ms:.2f}ms (within tolerance {tolerance_ns/1e6:.2f}ms), "
                            f"latest_t={latest_timestamp} ({latest_timestamp/1e9:.9f}s)"
                        )
                        return camera_data
                    else:
                        self._node.get_logger().warning(
                            f"[Wait {camera_id}] ❌ Latest data timestamp diff too large: "
                            f"{diff_ms:.2f}ms > {tolerance_ns/1e6:.2f}ms"
                        )
                else:
                    self._node.get_logger().warning(
                        f"[Wait {camera_id}] ❌ Latest data has no timestamp"
                    )
            else:
                self._node.get_logger().warning(
                    f"[Wait {camera_id}] ❌ Latest data missing camera_id in image dict"
                )
        else:
            self._node.get_logger().warning(
                f"[Wait {camera_id}] ❌ No latest data available"
            )
        
        return None
    
    def read_cameras(self, use_sync: bool = True) -> Tuple[Dict, Dict]:
        """
        Read camera data from all cameras in droid format.
        
        Each camera's camera_info, color, and depth are already synchronized.
        If use_sync=True, returns time-synchronized data from all cameras using
        ApproximateTimeSynchronizer. If use_sync=False, returns latest data from
        each camera (may not be time-synchronized).
        
        Args:
            use_sync: If True, use multi-camera synchronization. If False, read latest data from each camera.
        
        Returns:
            Tuple[dict, dict]: (full_obs_dict, full_timestamp_dict)
                - full_obs_dict: Dictionary with "image" and "depth" keys, each containing
                  {camera_id: numpy_array, ...}
                - full_timestamp_dict: Dictionary with all camera timestamps merged (all in nanoseconds):
                  - {camera_id}_read_start: Start time (nanoseconds, ROS time)
                  - {camera_id}_frame_received: Frame received time (nanoseconds, ROS time)
                  - {camera_id}_read_end: End time (nanoseconds, ROS time)
                  - {camera_id}_pub_t: Published time (nanoseconds, ROS time from message header)
                  - {camera_id}_sub_t: Subscription/receive time (nanoseconds, ROS time)
                  - {camera_id}_end_t: End processing time (nanoseconds, ROS time)
                  - multi_camera_sync_start: Multi-camera sync start time (if use_sync=True)
                  - multi_camera_sync_end: Multi-camera sync end time (if use_sync=True)
        """
        # Try to use synchronized data if available
        if use_sync:
            with self._sync_data_lock:
                if self._latest_sync_data_dict is not None and self._latest_sync_timestamp_dict is not None:
                    # Return synchronized data
                    return self._latest_sync_data_dict, self._latest_sync_timestamp_dict
        
        # Fallback to reading latest data from each camera (non-synchronized)
        # Initialize dictionaries in droid format
        full_obs_dict = defaultdict(dict)
        full_timestamp_dict = {}
        
        # Read cameras in randomized order (matching droid behavior)
        all_cam_ids = list(self.camera_dict.keys())
        random.shuffle(all_cam_ids)
        
        # Read data from each camera
        for cam_id in all_cam_ids:
            camera = self.camera_dict[cam_id]
            if not camera.is_running():
                continue
            
            try:
                data_dict, timestamp_dict = camera.read_camera()
                
                # Merge data_dict into full_obs_dict (matching droid format)
                for key in data_dict:
                    full_obs_dict[key].update(data_dict[key])
                
                # Merge timestamp_dict
                full_timestamp_dict.update(timestamp_dict)
                
            except Exception as e:
                # Log error but continue with other cameras
                self._node.get_logger().warning(f"Failed to read camera {cam_id}: {e}")
        
        return full_obs_dict, full_timestamp_dict
    
    def get_camera(self, camera_id: str) -> Optional[ROS2CameraReader]:
        """
        Get camera object by ID.
        
        Args:
            camera_id: Camera identifier
            
        Returns:
            Camera object, or None if not found
        """
        return self.camera_dict.get(camera_id)
