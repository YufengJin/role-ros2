from copy import deepcopy
from typing import Optional
import threading

import gym
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from role_ros2.camera.multi_camera_wrapper import MultiCameraWrapper
from role_ros2.robot.base_robot import BaseRobot
from role_ros2.robot.franka.robot import FrankaRobot
from role_ros2.misc.ros2_utils import get_ros_time_ns


class RobotEnv(gym.Env):
    """
    General robot environment for role-ros2.
    
    This class provides a Gym-like interface for robot control with camera support.
    It uses BaseRobot and MultiCameraWrapper for camera support.
    """
    
    def __init__(
        self,
        action_space: str = "cartesian_velocity",
        gripper_action_space: Optional[str] = None,
        do_reset: bool = True,
        node=None,
        robot: Optional[BaseRobot] = None,
        camera_reader: Optional[MultiCameraWrapper] = None,
        control_hz: float = 15.0
    ):
        """
        Initialize RobotEnv.
        
        Args:
            action_space: Action space type ("cartesian_position", "joint_position",
                         "cartesian_velocity", "joint_velocity")
            gripper_action_space: Gripper action space ("position" or "velocity")
            do_reset: Whether to reset robot during initialization
            node: Optional ROS2 node (used for robot initialization if robot is None, and for camera_reader if camera_reader is None)
            robot: Optional robot interface (if None, creates FrankaRobot)
            camera_reader: Optional MultiCameraWrapper instance (if None, creates MultiCameraWrapper automatically)
            control_hz: Control frequency in Hz (default: 15.0)
        """
        # Initialize Gym Environment
        super().__init__()

        # Define Action Space #
        assert action_space in ["cartesian_position", "joint_position", "cartesian_velocity", "joint_velocity"]
        self.action_space = action_space
        self.gripper_action_space = gripper_action_space
        self.check_action_range = "velocity" in action_space
        self.control_hz = control_hz

        # Ensure a node exists (create one if node is None)
        if node is None:
            if not rclpy.ok():
                rclpy.init()
            self._node = Node('robot_env')
            self._own_node = True
        else:
            self._node = node
            self._own_node = False

        # Initialize robot (before executor setup, so subscribers are registered)
        if robot is None:
            self._robot: BaseRobot = FrankaRobot(node=self._node)
        else:
            self._robot: BaseRobot = robot

        # Get DoF from robot (supports single-arm and bimanual)
        if hasattr(self._robot, "DOF_CARTESIAN") and hasattr(self._robot, "DOF_JOINT"):
            self.DoF = (
                self._robot.DOF_CARTESIAN
                if "cartesian" in action_space
                else self._robot.DOF_JOINT
            )
        else:
            self.DoF = 7 if ("cartesian" in action_space) else 8

        # Initialize camera reader (before executor setup, so subscribers are registered)
        if camera_reader is None:
            # Create MultiCameraWrapper if no camera_reader provided
            try:
                self.camera_reader: Optional[MultiCameraWrapper] = MultiCameraWrapper(
                    node=self._node
                )
            except Exception as e:
                self._node.get_logger().warning(
                    f"Failed to initialize MultiCameraWrapper: {e}. "
                    f"Continuing without camera support."
                )
                self.camera_reader: Optional[MultiCameraWrapper] = None
        else:
            self.camera_reader: Optional[MultiCameraWrapper] = camera_reader
        
        # Setup MultiThreadedExecutor for background spinning
        # This ensures all ROS2 callbacks are processed continuously
        self._executor = MultiThreadedExecutor()
        self._executor.add_node(self._node)
        
        # Start background thread for executor spinning
        # This runs after robot and camera initialization so all subscribers are registered
        self._spin_thread = threading.Thread(target=self._spin_executor, daemon=True)
        self._spin_thread.start()
        
        self._node.get_logger().info("RobotEnv: Background executor thread started")

        # Reset Robot
        if do_reset:
            self._node.get_logger().info("🔄 RobotEnv: Calling reset() during initialization...")
            self.reset()
            self._node.get_logger().info("✅ RobotEnv: Reset completed during initialization")

    def step(self, action):
        # Check Action
        assert len(action) == self.DoF
        if self.check_action_range:
            assert (action.max() <= 1) and (action.min() >= -1)

        # Update Robot
        action_info = self.update_robot(
            action,
            action_space=self.action_space,
            gripper_action_space=self.gripper_action_space,
        )

        # Return Action Info
        return action_info

    def reset(self, randomize: bool = False):
        """
        Reset robot to home position.
        
        Args:
            randomize: If True, add random cartesian noise to reset position.
        """
        self._robot.reset(randomize=randomize, wait_for_completion=True, wait_time_sec=10.0)

    def update_robot(self, action, action_space="cartesian_velocity", gripper_action_space=None, blocking=False):
        action_info = self._robot.update_command(
            action,
            action_space=action_space,
            gripper_action_space=gripper_action_space,
            blocking=blocking
        )
        return action_info

    def create_action_dict(self, action):
        return self._robot.create_action_dict(action)

    def read_cameras(self, use_sync: bool = True):
        """
        Read camera data from all cameras, including intrinsics and extrinsics.
        
        Args:
            use_sync: If True, use multi-camera synchronization. If False, read latest data from each camera.
                     Default is False for better reliability (synchronization may not always be available).
        
        Returns:
            Tuple[dict, dict]: (camera_obs_dict, camera_timestamp_dict)
                camera_obs_dict contains:
                - "image": {camera_id: numpy.ndarray, ...}
                - "depth": {camera_id: numpy.ndarray, ...}
                - "camera_intrinsics": {camera_id: numpy.ndarray (3x3), ...} or None
                - "camera_extrinsics": {camera_id: numpy.ndarray (4x4), ...} or None
        """
        if self.camera_reader is None:
            return {}, {}
        
        # Read camera images and depth
        # Use use_sync=False by default for better reliability
        # Synchronization may not always be available, especially during startup
        read_start = get_ros_time_ns(self._node)
        camera_obs, camera_timestamp = self.camera_reader.read_cameras(use_sync=use_sync)
        
        # Get camera intrinsics
        camera_intrinsics = self.camera_reader.get_camera_intrinsics()
        camera_obs["camera_intrinsics"] = camera_intrinsics
        
        # Get camera extrinsics using timestamp_dict for synchronized lookup
        camera_extrinsics = self.camera_reader.get_cameras_extrinsics(camera_timestamp)
        camera_obs["camera_extrinsics"] = camera_extrinsics

        camera_timestamp["read_start"] = read_start
        camera_timestamp["read_end"] = get_ros_time_ns(self._node)
        
        return camera_obs, camera_timestamp

    def get_state(self, use_sync: bool = False, timestamp_ns: Optional[int] = None):
        """
        Get robot state.
        
        Args:
            use_sync: If True and timestamp_ns is provided, get state closest to timestamp_ns.
                     If False, get latest state.
            timestamp_ns: Target timestamp in nanoseconds for synchronized lookup.
                         Only used when use_sync=True.
        
        Returns:
            Tuple[dict, dict]: (state_dict, timestamp_dict)
        
        No manual spin needed - background executor thread keeps data fresh.
        """
        read_start = get_ros_time_ns(self._node)
        
        if use_sync and timestamp_ns is not None:
            # Get state closest to the given timestamp (for camera-robot sync)
            state_dict, timestamp_dict, _ = self._robot.get_robot_state_for_timestamp(timestamp_ns)
        else:
            # Get latest state
            state_dict, timestamp_dict = self._robot.get_robot_state()
        
        timestamp_dict["read_start"] = read_start
        timestamp_dict["read_end"] = get_ros_time_ns(self._node)
        return state_dict, timestamp_dict
    
    def _spin_executor(self):
        """
        Background thread method that spins the MultiThreadedExecutor.
        
        This continuously processes ROS2 callbacks in the background,
        ensuring robot state and camera data are always up-to-date.
        """
        try:
            self._executor.spin()
        except Exception as e:
            # Handle shutdown gracefully
            if rclpy.ok():
                self._node.get_logger().error(f"Error in executor spin thread: {e}")
            else:
                # Normal shutdown
                self._node.get_logger().debug("Executor spin thread shutting down")
    
    def shutdown(self):
        """
        Cleanly shut down RobotEnv and all resources.
        
        Shutdown order (important for proper cleanup):
        1. Clean up camera resources (clear internal data, don't destroy node - it's shared)
        2. Clean up robot resources (clear internal data, don't destroy node - it's shared)
        3. Shutdown executor (stops spin thread)
        4. Join spin thread
        5. Destroy node if we own it
        
        Note: FrankaRobot and MultiCameraWrapper use shared node (_own_node=False),
        so their shutdown() only clears internal resources, not the node itself.
        """
        self._node.get_logger().info("RobotEnv: Shutting down...")
        
        # Step 1: Clean up camera resources (clears internal data only, node is shared)
        if self.camera_reader is not None and hasattr(self.camera_reader, 'shutdown'):
            try:
                self.camera_reader.shutdown()
            except Exception as e:
                self._node.get_logger().warn(f"Error shutting down camera reader: {e}")
        
        # Step 2: Clean up robot resources (clears internal data only, node is shared)
        if hasattr(self, '_robot') and self._robot is not None and hasattr(self._robot, 'shutdown'):
            try:
                self._robot.shutdown()
            except Exception as e:
                self._node.get_logger().warn(f"Error shutting down robot: {e}")
        
        # Step 3: Shutdown executor (stops spin thread)
        if hasattr(self, '_executor') and self._executor is not None:
            try:
                self._executor.shutdown(timeout_sec=2.0)
            except Exception as e:
                self._node.get_logger().warn(f"Error shutting down executor: {e}")
        
        # Step 4: Join spin thread
        if hasattr(self, '_spin_thread') and self._spin_thread is not None:
            if self._spin_thread.is_alive():
                try:
                    self._spin_thread.join(timeout=3.0)
                    if self._spin_thread.is_alive():
                        self._node.get_logger().warn("Spin thread did not finish within timeout")
                except Exception as e:
                    self._node.get_logger().warn(f"Error joining spin thread: {e}")
        
        # Step 5: Destroy node if we own it
        if self._own_node:
            try:
                self._node.destroy_node()
                self._node.get_logger().info("RobotEnv: Node destroyed")
            except Exception as e:
                self._node.get_logger().warn(f"Error destroying node: {e}")
        else:
            self._node.get_logger().debug("RobotEnv: Using shared node - not destroying")
        
        self._node.get_logger().info("RobotEnv: Shutdown complete")


    def get_observation(self, use_sync: bool = True):
        """
        Get comprehensive observation dictionary.
        
        Args:
            use_sync: If True, synchronize camera and robot state timestamps.
                     Camera sync timestamp (multi_camera_sync_start) is used to 
                     lookup the closest robot state from cache.
                     If False, get latest camera and robot data independently.
        
        Returns:
            dict: Observation dictionary with the following structure:
            - Single arm (FrankaRobot): robot_state has cartesian_position, gripper_position, etc.
            - Bimanual (BimanualFrankaRobot): robot_state has left_arm, right_arm,
              left_gripper_position, right_gripper_position.
            {
                "robot_state": {
                    # Single arm: cartesian_position, gripper_position, joint_positions, ...
                    # Bimanual: left_arm, right_arm, left_gripper_position, right_gripper_position
                    "joint_velocities": [qd1, qd2, ..., qd7],  # 7 floats (rad/s)
                    "joint_torques_computed": [tau1, ..., tau7],  # 7 floats
                    "prev_joint_torques_computed": [tau1, ..., tau7],  # 7 floats
                    "prev_joint_torques_computed_safened": [tau1, ..., tau7],  # 7 floats
                    "motor_torques_measured": [tau1, ..., tau7],  # 7 floats
                    "prev_controller_latency_ms": float,
                    "prev_command_successful": bool,
                },
                "image": {
                    camera_id: numpy.ndarray,  # BGR image (H, W, 3), uint8
                    ...
                },
                "depth": {
                    camera_id: numpy.ndarray,  # Depth image (H, W), uint16 (millimeters)
                    ...
                },
                "camera_extrinsics": {
                    camera_id: numpy.ndarray,  # 4x4 transformation matrix (base_link to camera)
                    ...
                } or None,  # None if no cameras available or TF lookup failed
                "camera_intrinsics": {
                    camera_id: numpy.ndarray,  # 3x3 intrinsic matrix (K)
                    ...
                } or None,  # None if no cameras available
                "timestamp": {
                    "robot_state": {
                        "robot_polymetis_t": int,  # Polymetis timestamp (nanoseconds)
                        "robot_pub_t": int,  # Message header timestamp (nanoseconds)
                        "robot_sub_t": int,  # Message received/subscription time (nanoseconds, ROS time)
                        "robot_end_t": int,  # Processing end time (nanoseconds, ROS time)
                        "read_start": int,  # get_state() start time (nanoseconds, ROS time)
                        "read_end": int,  # get_state() end time (nanoseconds, ROS time)
                        "sync_time_diff_ns": int,  # (only when use_sync=True) Time diff to camera sync
                    },
                    "cameras": {
                        "{camera_id}_pub_t": int,  # Published time (nanoseconds, ROS time)
                        "{camera_id}_sub_t": int,  # Subscription time (nanoseconds, ROS time)
                        "{camera_id}_end_t": int,  # Processing end time (nanoseconds, ROS time)
                        "multi_camera_sync_start": int,  # Multi-camera sync start time (nanoseconds)
                        "read_start": int,  # read_cameras() start time (nanoseconds, ROS time)
                        "read_end": int,  # read_cameras() end time (nanoseconds, ROS time)
                        ...
                    },
                },
            }
        """
        obs_dict = {"timestamp": {}}
        
        # Camera Readings (includes image, depth, intrinsics, and extrinsics)
        camera_obs, camera_timestamp = self.read_cameras(use_sync=use_sync)
        obs_dict.update(camera_obs)
        obs_dict["timestamp"]["cameras"] = camera_timestamp

        # Robot State
        # When use_sync=True, use mean of all camera pub_t timestamps to get time-synchronized robot state
        if use_sync:
            # Calculate mean of all camera pub_t timestamps
            camera_pub_timestamps = []
            for key, value in camera_timestamp.items():
                if key.endswith("_pub_t") and isinstance(value, (int, np.integer)):
                    camera_pub_timestamps.append(int(value))
            
            if camera_pub_timestamps:
                # Use mean of camera pub_t timestamps (more accurate for robot state lookup)
                sync_timestamp_ns = int(np.mean(camera_pub_timestamps))
            else:
                # Fallback to multi_camera_sync_start if no pub_t found
                sync_timestamp_ns = camera_timestamp.get("multi_camera_sync_start")
            
            state_dict, timestamp_dict = self.get_state(
                use_sync=True, 
                timestamp_ns=sync_timestamp_ns
            )
        else:
            state_dict, timestamp_dict = self.get_state(use_sync=False)
        
        obs_dict["robot_state"] = state_dict
        obs_dict["timestamp"]["robot_state"] = timestamp_dict

        return obs_dict
