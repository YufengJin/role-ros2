"""Multi-camera wrapper for role-ros2 using ROS2 with event-driven architecture."""

import random
import yaml
from pathlib import Path
from typing import Dict, Tuple, Optional, List, Set
from collections import defaultdict
from dataclasses import dataclass, field
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import CameraInfo
import message_filters

from role_ros2.camera.ros2_camera_reader import ROS2CameraReader, SENSOR_DATA_QOS
from role_ros2.misc.ros2_utils import get_ros_time_ns
from role_ros2.misc.config_loader import get_package_config_path, load_yaml_config


@dataclass
class SyncRequest:
    """
    Represents a multi-camera synchronization request.
    
    Tracks which cameras have reported data for a specific set of timestamps.
    """
    request_id: int
    camera_id_list: List[str]
    camera_timestamps: Dict[str, int]  # camera_id -> timestamp_ns
    created_time_ns: int
    cameras_ready: Set[str] = field(default_factory=set)
    
    def is_complete(self) -> bool:
        """Check if all cameras have reported data."""
        return len(self.cameras_ready) == len(self.camera_id_list)
    
    def mark_camera_ready(self, camera_id: str) -> bool:
        """
        Mark a camera as ready and return True if this completes the request.
        
        Args:
            camera_id: The camera that reported data
            
        Returns:
            True if all cameras are now ready
        """
        if camera_id in self.camera_id_list:
            self.cameras_ready.add(camera_id)
        return self.is_complete()


class MultiCameraWrapper:
    """
    ROS2-based multi-camera wrapper with event-driven architecture.
    
    Uses layered synchronization:
    - Layer 1: Each camera uses ApproximateTimeSynchronizer internally
    - Layer 2: MultiCameraWrapper uses ApproximateTimeSynchronizer on camera_info messages
    
    Event-driven processing:
    - No polling timers - everything is callback-based
    - When Layer 2 matches camera_info messages, creates a SyncRequest
    - Each ROS2CameraReader triggers a callback when data arrives
    - When all cameras report data for a SyncRequest, immediately process and store
    
    This architecture minimizes latency by eliminating polling delays.
    """
    
    def __init__(
        self,
        node: Optional[Node] = None,
        config_file: Optional[str] = None
    ):
        """
        Initialize multi-camera wrapper with event-driven architecture.
        
        Args:
            node: Optional ROS2 node (if None, creates a new node)
            config_file: Path to YAML configuration file
        """
        # Use provided node or create a new one
        self._own_node = node is None
        if node is None:
            if not rclpy.ok():
                rclpy.init()
            self._node = Node('multi_camera_wrapper')
        else:
            self._node = node
        
        # Initialize camera dictionary
        self.camera_dict: Dict[str, ROS2CameraReader] = {}
        
        # Camera configurations
        self.camera_configs: List[Dict] = []
        
        # Thread-safe storage for synchronized multi-camera data
        self._latest_sync_data_dict: Optional[Dict] = None
        self._latest_sync_timestamp_dict: Optional[Dict] = None
        self._sync_data_lock = threading.Lock()
        
        # Pending sync requests (event-driven)
        # Maps request_id -> SyncRequest
        self._pending_sync_requests: Dict[int, SyncRequest] = {}
        # Maps (camera_id, timestamp_ns) -> request_id for fast lookup
        self._timestamp_to_request: Dict[Tuple[str, int], int] = {}
        self._sync_requests_lock = threading.Lock()
        self._sync_request_counter = 0
        self._max_pending_requests = 20
        self._max_request_age_ns = 500_000_000  # 500ms timeout
        
        # Load camera configurations
        if config_file is None:
            config_file = get_package_config_path('multi_camera_reader_config.yaml')
        else:
            config_file = Path(config_file)
        
        camera_configs = self._load_config(config_file)
        self.camera_configs = camera_configs
        
        if not camera_configs:
            self._node.get_logger().warning("No cameras configured!")
            return
        
        # Create camera readers with event-driven callbacks
        for config in camera_configs:
            camera_id = config["camera_id"]
            rgb_topic = config["rgb_topic"]
            depth_topic = config["depth_topic"]
            camera_info_topic = config.get("camera_info_topic", None)
            
            try:
                slop = config.get("slop", 0.05)
                queue_size = config.get("queue_size", 10)
                
                camera_reader = ROS2CameraReader(
                    camera_id=camera_id,
                    rgb_topic=rgb_topic,
                    depth_topic=depth_topic,
                    node=self._node,
                    camera_info_topic=camera_info_topic,
                    queue_size=queue_size,
                    slop=slop,
                    base_frame=config.get("base_frame", None),
                    camera_frame=config.get("camera_frame", None),
                    on_data_received_callback=self._on_camera_data_received
                )
                self.camera_dict[camera_id] = camera_reader
            except Exception as e:
                self._node.get_logger().error(
                    f"Failed to initialize camera {camera_id}: {e}"
                )
        
        # Setup layered multi-camera synchronization
        self._setup_layered_multi_camera_sync(camera_configs)
        
        # Spin thread management based on node ownership
        self._executor = None
        self._spin_thread = None
        
        if self._own_node:
            # Own node: create executor and spin thread
            from rclpy.executors import MultiThreadedExecutor
            self._executor = MultiThreadedExecutor()
            self._executor.add_node(self._node)
            self._spin_thread = threading.Thread(target=self._spin_executor, daemon=True)
            self._spin_thread.start()
            self._node.get_logger().info("MultiCameraWrapper: Own node created with spin thread")
        else:
            # Shared node: external executor handles spinning (e.g., RobotEnv's MultiThreadedExecutor)
            self._node.get_logger().debug("MultiCameraWrapper: Using shared node - external executor handles spinning")
    
    def _load_config(self, config_file: Path) -> List[Dict]:
        """Load camera configurations from YAML file."""
        if not config_file.exists():
            self._node.get_logger().error(
                f"Camera config file not found: {config_file}"
            )
            return []
        
        try:
            with open(config_file, 'r') as f:
                config_data = yaml.safe_load(f)
            
            global_base_frame = config_data.get("base_frame", "base_link")
            global_sync = config_data.get("global_sync", {})
            global_slop = global_sync.get("slop", 0.05)
            global_queue_size = global_sync.get("queue_size", 10)
            
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
                    "slop": camera.get("slop", global_slop if global_slop else 0.05),
                    "queue_size": camera.get("queue_size", global_queue_size)
                }
                camera_configs.append(camera_config)
            
            return camera_configs
            
        except Exception as e:
            self._node.get_logger().error(
                f"Error loading camera config: {e}"
            )
            return []
    
    def _setup_layered_multi_camera_sync(self, camera_configs: List[Dict]) -> None:
        """
        Setup ApproximateTimeSynchronizer for multi-camera synchronization (Layer 2).
        
        Event-driven: Creates SyncRequest objects when camera_info messages are matched.
        """
        # Get global sync settings
        global_sync = {}
        try:
            config_data = load_yaml_config('multi_camera_config.yaml')
            global_sync = config_data.get("global_sync", {})
        except Exception:
            pass
        
        multi_camera_slop = global_sync.get("multi_camera_slop", global_sync.get("slop", 0.2))
        multi_camera_queue_size = global_sync.get("multi_camera_queue_size", 10)
        
        self._multi_camera_slop = multi_camera_slop
        self._multi_camera_slop_ns = int(multi_camera_slop * 1_000_000_000)
        
        # Use ReentrantCallbackGroup for concurrent callbacks
        callback_group = ReentrantCallbackGroup()
        
        # Create subscribers for each camera's camera_info
        camera_info_subs = []
        camera_id_list = []
        
        for config in camera_configs:
            camera_id = config["camera_id"]
            camera_info_topic = config.get("camera_info_topic", None)
            
            if camera_info_topic is None:
                self._node.get_logger().warning(
                    f"Camera {camera_id} has no camera_info_topic, skipping sync"
                )
                continue
            
            camera_info_sub = message_filters.Subscriber(
                self._node, CameraInfo, camera_info_topic,
                callback_group=callback_group, qos_profile=SENSOR_DATA_QOS
            )
            camera_info_subs.append(camera_info_sub)
            camera_id_list.append(camera_id)
        
        if len(camera_info_subs) < 2:
            self._node.get_logger().warning(
                f"Only {len(camera_info_subs)} camera(s), need at least 2 for multi-camera sync"
            )
            return
        
        # Store camera_id_list for later use
        self._sync_camera_ids = camera_id_list
        
        # Create ApproximateTimeSynchronizer
        self._multi_camera_sync = message_filters.ApproximateTimeSynchronizer(
            camera_info_subs,
            queue_size=multi_camera_queue_size,
            slop=multi_camera_slop
        )
        self._multi_camera_sync.registerCallback(
            lambda *args: self._on_multi_camera_sync(camera_id_list, *args)
        )
    
    def _on_multi_camera_sync(
        self,
        camera_id_list: List[str],
        *camera_info_msgs: CameraInfo
    ) -> None:
        """
        Callback when multi-camera CameraInfo messages are synchronized (Layer 2).
        
        Creates a SyncRequest and checks if data is already available.
        This is event-driven - no polling needed.
        
        Args:
            camera_id_list: List of camera IDs
            *camera_info_msgs: Synchronized CameraInfo messages
        """
        try:
            start_t_ns = get_ros_time_ns(self._node)
            
            # Extract timestamps from each camera_info message
            camera_timestamps = {}
            for camera_id, camera_info_msg in zip(camera_id_list, camera_info_msgs):
                ts_ns = int(
                    camera_info_msg.header.stamp.sec * 1_000_000_000 +
                    camera_info_msg.header.stamp.nanosec
                )
                camera_timestamps[camera_id] = ts_ns
            
            with self._sync_requests_lock:
                # Create new sync request
                self._sync_request_counter += 1
                request_id = self._sync_request_counter
                
                # Clean up old requests
                self._cleanup_old_requests(start_t_ns)
                
                # Limit queue size
                if len(self._pending_sync_requests) >= self._max_pending_requests:
                    oldest_id = min(self._pending_sync_requests.keys())
                    self._remove_sync_request(oldest_id)
                    self._node.get_logger().debug(
                        f"[MultiCamera] Request queue full, removed oldest request #{oldest_id}"
                    )
                
                # Create SyncRequest
                sync_request = SyncRequest(
                    request_id=request_id,
                    camera_id_list=camera_id_list.copy(),
                    camera_timestamps=camera_timestamps.copy(),
                    created_time_ns=start_t_ns
                )
                
                # Register the request
                self._pending_sync_requests[request_id] = sync_request
                
                # Map timestamps to request for fast lookup
                for camera_id, ts_ns in camera_timestamps.items():
                    self._timestamp_to_request[(camera_id, ts_ns)] = request_id
                
                # Check if data is already available for any cameras
                for camera_id, ts_ns in camera_timestamps.items():
                    camera_reader = self.camera_dict.get(camera_id)
                    if camera_reader and camera_reader.has_data_for_timestamp(ts_ns):
                        sync_request.mark_camera_ready(camera_id)
                
                # If all cameras already have data, process immediately
                if sync_request.is_complete():
                    self._process_complete_request(sync_request)
                    
        except Exception as e:
            self._node.get_logger().error(f"Error in multi-camera sync callback: {e}")
            import traceback
            self._node.get_logger().error(traceback.format_exc())
    
    def _on_camera_data_received(self, camera_id: str, timestamp_ns: int) -> None:
        """
        Callback when a camera reports new data (event-driven).
        
        This is called by ROS2CameraReader when data is stored.
        Checks if this completes any pending sync requests.
        
        Args:
            camera_id: Camera that received data
            timestamp_ns: Timestamp of the received data
        """
        try:
            with self._sync_requests_lock:
                # Look up request by (camera_id, timestamp)
                request_id = self._timestamp_to_request.get((camera_id, timestamp_ns))
                
                if request_id is None:
                    return  # No pending request for this timestamp
                
                sync_request = self._pending_sync_requests.get(request_id)
                if sync_request is None:
                    return  # Request was already processed or cleaned up
                
                # Mark this camera as ready
                if sync_request.mark_camera_ready(camera_id):
                    # All cameras ready - process immediately!
                    self._process_complete_request(sync_request)
                    
        except Exception as e:
            self._node.get_logger().error(
                f"Error in camera data callback for {camera_id}: {e}"
            )
    
    def _process_complete_request(self, sync_request: SyncRequest) -> None:
        """
        Process a completed sync request and update synchronized data.
        
        Must be called with _sync_requests_lock held.
        
        Args:
            sync_request: The completed SyncRequest
        """
        try:
            process_start_ns = get_ros_time_ns(self._node)
            
            # Collect data from all cameras
            sync_data_dict = defaultdict(dict)
            sync_timestamp_dict = {}
            
            for camera_id in sync_request.camera_id_list:
                camera_reader = self.camera_dict.get(camera_id)
                if camera_reader is None:
                    continue
                
                target_ts = sync_request.camera_timestamps.get(camera_id)
                if target_ts is None:
                    continue
                
                # Get decoded data (lazy decoding happens here)
                camera_data = camera_reader.get_data_for_timestamp(target_ts)
                if camera_data is None:
                    # This shouldn't happen since we checked has_data_for_timestamp
                    self._node.get_logger().warning(
                        f"[MultiCamera] Data missing for camera {camera_id} at {target_ts}"
                    )
                    continue
                
                data_dict, timestamp_dict = camera_data
                
                # Merge data
                for key in data_dict:
                    sync_data_dict[key].update(data_dict[key])
                sync_timestamp_dict.update(timestamp_dict)
            
            # Get end timestamp
            end_t_ns = get_ros_time_ns(self._node)
            
            # Store synchronized data
            with self._sync_data_lock:
                self._latest_sync_data_dict = sync_data_dict
                self._latest_sync_timestamp_dict = sync_timestamp_dict
                
                # Add multi-camera sync metadata
                self._latest_sync_timestamp_dict["multi_camera_sync_start"] = sync_request.created_time_ns
                self._latest_sync_timestamp_dict["multi_camera_sync_ready"] = process_start_ns
                self._latest_sync_timestamp_dict["multi_camera_sync_end"] = end_t_ns
                self._latest_sync_timestamp_dict["multi_camera_sync_cameras_count"] = len(sync_request.cameras_ready)
                self._latest_sync_timestamp_dict["multi_camera_sync_fallback"] = False  # Normal sync, not fallback
            
            # Clean up this request
            self._remove_sync_request(sync_request.request_id)
            
        except Exception as e:
            self._node.get_logger().error(f"Error processing sync request: {e}")
            import traceback
            self._node.get_logger().error(traceback.format_exc())
    
    def _remove_sync_request(self, request_id: int) -> None:
        """
        Remove a sync request and its timestamp mappings.
        
        Must be called with _sync_requests_lock held.
        
        Args:
            request_id: Request ID to remove
        """
        sync_request = self._pending_sync_requests.pop(request_id, None)
        if sync_request:
            # Remove timestamp mappings
            for camera_id, ts_ns in sync_request.camera_timestamps.items():
                self._timestamp_to_request.pop((camera_id, ts_ns), None)
    
    def _cleanup_old_requests(self, current_time_ns: int) -> None:
        """
        Clean up sync requests that have timed out.
        
        Must be called with _sync_requests_lock held.
        
        Args:
            current_time_ns: Current time in nanoseconds
        """
        expired_ids = []
        for request_id, sync_request in self._pending_sync_requests.items():
            age_ns = current_time_ns - sync_request.created_time_ns
            if age_ns > self._max_request_age_ns:
                expired_ids.append(request_id)
        
        for request_id in expired_ids:
            sync_request = self._pending_sync_requests.get(request_id)
            if sync_request:
                # Try fallback: use latest data from each camera
                self._try_fallback_for_request(sync_request, current_time_ns)
            self._remove_sync_request(request_id)
    
    def _try_fallback_for_request(
        self,
        sync_request: SyncRequest,
        current_time_ns: int
    ) -> None:
        """
        Try to use latest available data when a sync request times out.
        
        Must be called with _sync_requests_lock held.
        
        Args:
            sync_request: The timed-out SyncRequest
            current_time_ns: Current time in nanoseconds
        """
        try:
            fallback_data_dict = defaultdict(dict)
            fallback_timestamp_dict = {}
            cameras_with_data = 0
            
            for camera_id in sync_request.camera_id_list:
                camera_reader = self.camera_dict.get(camera_id)
                if camera_reader is None:
                    return  # Can't fallback without all cameras
                
                # Get latest available data
                latest_ts = camera_reader.get_latest_pub_timestamp()
                if latest_ts is None:
                    return  # No data available
                
                camera_data = camera_reader.get_data_for_timestamp(latest_ts)
                if camera_data is None:
                    return
                
                data_dict, timestamp_dict = camera_data
                
                for key in data_dict:
                    fallback_data_dict[key].update(data_dict[key])
                fallback_timestamp_dict.update(timestamp_dict)
                cameras_with_data += 1
            
            # If we got data from all cameras, use it
            if cameras_with_data == len(sync_request.camera_id_list):
                end_t_ns = get_ros_time_ns(self._node)
                
                with self._sync_data_lock:
                    self._latest_sync_data_dict = fallback_data_dict
                    self._latest_sync_timestamp_dict = fallback_timestamp_dict
                    
                    self._latest_sync_timestamp_dict["multi_camera_sync_start"] = sync_request.created_time_ns
                    self._latest_sync_timestamp_dict["multi_camera_sync_ready"] = current_time_ns
                    self._latest_sync_timestamp_dict["multi_camera_sync_end"] = end_t_ns
                    self._latest_sync_timestamp_dict["multi_camera_sync_cameras_count"] = cameras_with_data
                    self._latest_sync_timestamp_dict["multi_camera_sync_fallback"] = True
                
                self._node.get_logger().debug(
                    f"[MultiCamera] Using fallback data for request #{sync_request.request_id}"
                )
                
        except Exception as e:
            self._node.get_logger().debug(f"Fallback failed: {e}")
    
    def read_cameras(self, use_sync: bool = True) -> Tuple[Dict, Dict]:
        """
        Read camera data from all cameras in droid format.
        
        For use_sync=True:
        - Returns the latest synchronized data from all cameras
        - If no sync data available yet, returns empty dicts
        - This is non-blocking and returns immediately
        
        For use_sync=False:
        - Returns latest data from each camera (may not be time-synchronized)
        
        Args:
            use_sync: If True, use multi-camera synchronization
        
        Returns:
            Tuple[dict, dict]: (full_obs_dict, full_timestamp_dict)
        """
        if use_sync:
            with self._sync_data_lock:
                if self._latest_sync_data_dict is not None and self._latest_sync_timestamp_dict is not None:
                    # Return copies to avoid reference issues
                    data_copy = defaultdict(dict)
                    for key, value in self._latest_sync_data_dict.items():
                        data_copy[key] = dict(value)
                    timestamp_copy = dict(self._latest_sync_timestamp_dict)
                    return data_copy, timestamp_copy
                else:
                    return defaultdict(dict), {}
        
        # Non-synchronized mode
        full_obs_dict = defaultdict(dict)
        full_timestamp_dict = {}
        
        all_cam_ids = list(self.camera_dict.keys())
        random.shuffle(all_cam_ids)
        
        for cam_id in all_cam_ids:
            camera = self.camera_dict[cam_id]
            if not camera.is_running():
                continue
            
            try:
                data_dict, timestamp_dict = camera.read_camera()
                
                for key in data_dict:
                    full_obs_dict[key].update(data_dict[key])
                full_timestamp_dict.update(timestamp_dict)
                
            except Exception as e:
                self._node.get_logger().warning(f"Failed to read camera {cam_id}: {e}")
        
        return full_obs_dict, full_timestamp_dict
    
    def get_camera_intrinsics(self) -> Optional[Dict]:
        """
        Get camera intrinsics for all cameras.
        
        Returns:
            Dictionary mapping camera_id to 3x3 intrinsic matrix, or None
        """
        if not self.camera_dict:
            return None
        
        intrinsics = {}
        try:
            for cam in self.camera_dict.values():
                cam_intr_info = cam.get_intrinsics()
                if cam_intr_info is not None and "cameraMatrix" in cam_intr_info:
                    intrinsics[cam.camera_id] = cam_intr_info["cameraMatrix"]
        except Exception as e:
            self._node.get_logger().warning(f"Failed to get intrinsics: {e}")
            return None
        return intrinsics if intrinsics else None
    
    def get_cameras_extrinsics(
        self,
        timestamp_dict: Optional[Dict] = None
    ) -> Optional[Dict]:
        """
        Get camera extrinsics (transformation matrices) for all cameras.
        
        Args:
            timestamp_dict: Optional timestamp dictionary for TF lookup
        
        Returns:
            Dictionary mapping camera_id to 4x4 transformation matrix, or None
        """
        if not self.camera_dict:
            return None
        
        extrinsics = {}
        for cam in self.camera_dict.values():
            try:
                extrinsic_matrix = cam.get_extrinsic(timestamp_dict)
                if extrinsic_matrix is not None:
                    extrinsics[cam.camera_id] = extrinsic_matrix
            except Exception as e:
                self._node.get_logger().debug(
                    f"Failed to get extrinsic for camera {cam.camera_id}: {e}"
                )
        
        return extrinsics if extrinsics else None
    
    def reset_tf_states(self):
        """
        Reset TF lookup state for all cameras.
        
        Useful when TF tree is reconfigured at runtime.
        """
        for cam in self.camera_dict.values():
            cam.reset_tf_state()
        self._node.get_logger().info("Reset TF states for all cameras")
    
    def get_tf_status(self) -> Dict:
        """
        Get TF lookup status for all cameras.
        
        Returns:
            Dictionary mapping camera_id to TF status
        """
        return {
            cam_id: cam.get_tf_status()
            for cam_id, cam in self.camera_dict.items()
        }
    
    def _spin_executor(self):
        """
        Background thread method that spins the executor.
        Only used when own_node=True.
        """
        try:
            self._executor.spin()
        except Exception as e:
            if rclpy.ok():
                self._node.get_logger().error(f"Error in MultiCameraWrapper spin thread: {e}")
    
    def shutdown(self):
        """
        Clean up resources.
        
        If own_node=True: stops executor, spin thread, clears resources, and destroys node.
        If own_node=False: only clears internal resources (node is managed by external executor).
        """
        if self._node is not None:
            self._node.get_logger().info("MultiCameraWrapper: Shutting down...")
        
        # Step 1: Stop multi-camera synchronizer
        if hasattr(self, '_multi_camera_sync') and self._multi_camera_sync is not None:
            try:
                del self._multi_camera_sync
                self._multi_camera_sync = None
            except Exception as e:
                if self._node is not None:
                    self._node.get_logger().warn(f"Error stopping multi-camera sync: {e}")
        
        # Step 2: Clear camera dictionary
        if hasattr(self, 'camera_dict') and self.camera_dict:
            try:
                self.camera_dict.clear()
            except Exception as e:
                if self._node is not None:
                    self._node.get_logger().warn(f"Error clearing camera dict: {e}")
        
        # Step 3: Clear sync request queues
        if hasattr(self, '_pending_sync_requests') and hasattr(self, '_sync_requests_lock'):
            try:
                with self._sync_requests_lock:
                    self._pending_sync_requests.clear()
                    if hasattr(self, '_timestamp_to_request'):
                        self._timestamp_to_request.clear()
            except Exception as e:
                if self._node is not None:
                    self._node.get_logger().warn(f"Error clearing sync request queues: {e}")
        
        # Step 4: Clear synchronized data
        if hasattr(self, '_sync_data_lock'):
            with self._sync_data_lock:
                self._latest_sync_data_dict = None
                self._latest_sync_timestamp_dict = None
        
        # Step 5: If own_node, shutdown executor, spin thread, and destroy node
        if self._own_node:
            # Shutdown executor (stops spin thread)
            if self._executor is not None:
                try:
                    self._executor.shutdown(timeout_sec=2.0)
                except Exception as e:
                    if self._node is not None:
                        self._node.get_logger().warn(f"Error shutting down executor: {e}")
            
            # Join spin thread
            if self._spin_thread is not None and self._spin_thread.is_alive():
                try:
                    self._spin_thread.join(timeout=3.0)
                except Exception as e:
                    if self._node is not None:
                        self._node.get_logger().warn(f"Error joining spin thread: {e}")
            
            # Destroy node
            if self._node is not None:
                try:
                    self._node.destroy_node()
                except Exception as e:
                    self._node.get_logger().warn(f"Error destroying node: {e}")
            
            # Shutdown rclpy if we initialized it
            if rclpy.ok():
                try:
                    rclpy.shutdown()
                except Exception as e:
                    pass  # May already be shutdown
        else:
            # Shared node: external executor manages node lifecycle
            if self._node is not None:
                self._node.get_logger().debug("MultiCameraWrapper: Using shared node - not destroying")
        
        if self._node is not None:
            self._node.get_logger().info("MultiCameraWrapper: Shutdown complete")