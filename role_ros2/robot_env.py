from copy import deepcopy
from typing import Optional
import threading

import gym
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from role_ros2.calibration.calibration_utils import load_calibration_info
from role_ros2.camera.multi_camera_wrapper import MultiCameraWrapper
from role_ros2.robot.base_robot import BaseRobot
from role_ros2.robot.franka.robot import FrankaRobot
from role_ros2.misc.parameters import hand_camera_id
from role_ros2.misc.time import time_ms
from role_ros2.misc.transformations import change_pose_frame


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
        self.DoF = 7 if ("cartesian" in action_space) else 8
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
        
        # Load calibration and camera info
        self.calibration_dict = load_calibration_info()

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
        self._robot.reset(randomize=randomize, wait_for_completion=True, wait_time_sec=20.0)

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
        camera_obs, camera_timestamp = self.camera_reader.read_cameras(use_sync=use_sync)
        
        # Get camera intrinsics
        camera_intrinsics = self.camera_reader.get_camera_intrinsics()
        camera_obs["camera_intrinsics"] = camera_intrinsics
        
        # Get camera extrinsics using timestamp_dict for synchronized lookup
        camera_extrinsics = self.camera_reader.get_cameras_extrinsics(camera_timestamp)
        camera_obs["camera_extrinsics"] = camera_extrinsics
        
        return camera_obs, camera_timestamp

    def get_state(self):
        """
        Get robot state.
        
        No manual spin needed - background executor thread keeps data fresh.
        """
        read_start = time_ms()
        state_dict, timestamp_dict = self._robot.get_robot_state()
        timestamp_dict["read_start"] = read_start
        timestamp_dict["read_end"] = time_ms()
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

    def get_camera_extrinsics(self, state_dict):
        # Return None if calibration_dict is empty
        if not self.calibration_dict:
            return None
        
        # Adjust gripper camere by current pose
        extrinsics = deepcopy(self.calibration_dict)
        for cam_id in self.calibration_dict:
            if hand_camera_id not in cam_id:
                continue
            gripper_pose = state_dict["cartesian_position"]
            extrinsics[cam_id + "_gripper_offset"] = extrinsics[cam_id]
            extrinsics[cam_id] = change_pose_frame(extrinsics[cam_id], gripper_pose)
        return extrinsics

    def get_observation(self):
        """
        Get comprehensive observation dictionary.
        
        Returns:
            dict: Observation dictionary with the following structure:
            {
                "robot_state": {
                    "cartesian_position": [x, y, z, roll, pitch, yaw],  # 6 floats
                    "gripper_position": float,  # normalized (0=closed, 1=open)
                    "joint_positions": [q1, q2, ..., q7],  # 7 floats (radians)
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
                        "read_start": int,  # get_state() start time (milliseconds, system time)
                        "read_end": int,  # get_state() end time (milliseconds, system time)
                    },
                    "cameras": {
                        "{camera_id}_pub_t": int,  # Published time (nanoseconds, ROS time)
                        "{camera_id}_sub_t": int,  # Subscription time (nanoseconds, ROS time)
                        "{camera_id}_end_t": int,  # Processing end time (nanoseconds, ROS time)
                        ...
                    },
                },
            }
        """
        obs_dict = {"timestamp": {}}

        # Robot State #
        state_dict, timestamp_dict = self.get_state()
        obs_dict["robot_state"] = state_dict
        obs_dict["timestamp"]["robot_state"] = timestamp_dict

        # Camera Readings (includes image, depth, intrinsics, and extrinsics) #
        camera_obs, camera_timestamp = self.read_cameras()
        obs_dict.update(camera_obs)
        obs_dict["timestamp"]["cameras"] = camera_timestamp

        return obs_dict
