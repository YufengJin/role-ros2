from copy import deepcopy
from typing import Optional

import gym
import numpy as np

from role_ros2.calibration.calibration_utils import load_calibration_info
from role_ros2.camera_utils.info import camera_type_dict
from role_ros2.camera_utils.base_camera_reader import BaseCameraReader
from role_ros2.franka.base_robot import BaseRobot
from role_ros2.franka.robot import FrankaRobot
from role_ros2.misc.parameters import hand_camera_id
from role_ros2.misc.time import time_ms
from role_ros2.misc.transformations import change_pose_frame


class RobotEnv(gym.Env):
    """
    General robot environment for role-ros2.
    
    This class provides a Gym-like interface for robot control with camera support.
    It uses BaseRobot and BaseCameraReader interfaces for flexibility.
    """
    
    def __init__(
        self,
        action_space: str = "cartesian_velocity",
        gripper_action_space: Optional[str] = None,
        camera_kwargs: dict = None,
        do_reset: bool = True,
        node=None,
        robot: Optional[BaseRobot] = None,
        camera_reader: Optional[BaseCameraReader] = None,
        control_hz: float = 15.0
    ):
        """
        Initialize RobotEnv.
        
        Args:
            action_space: Action space type ("cartesian_position", "joint_position",
                         "cartesian_velocity", "joint_velocity")
            gripper_action_space: Gripper action space ("position" or "velocity")
            camera_kwargs: Camera configuration parameters (passed to camera_reader if provided)
            do_reset: Whether to reset robot during initialization
            node: Optional ROS2 node (used for robot initialization if robot is None)
            robot: Optional robot interface (if None, creates FrankaRobot)
            camera_reader: Optional camera reader interface (if None, camera_reader will be None)
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

        # Initialize robot
        if robot is None:
            self._robot: BaseRobot = FrankaRobot(node=node)
        else:
            self._robot: BaseRobot = robot

        # Initialize camera reader
        self.camera_reader: Optional[BaseCameraReader] = camera_reader
        
        # Load calibration and camera info
        self.calibration_dict = load_calibration_info()
        self.camera_type_dict = camera_type_dict

        # Reset Robot
        if do_reset:
            if node is not None:
                node.get_logger().info("🔄 RobotEnv: Calling reset() during initialization...")
            self.reset()
            if node is not None:
                node.get_logger().info("✅ RobotEnv: Reset completed during initialization")

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

    def read_cameras(self):
        # Note: Camera functionality not yet implemented in role_ros2
        # Return empty dict for now
        if self.camera_reader is None:
            return {}, {}
        return self.camera_reader.read_cameras()

    def get_state(self):
        read_start = time_ms()
        state_dict, timestamp_dict = self._robot.get_robot_state()
        timestamp_dict["read_start"] = read_start
        timestamp_dict["read_end"] = time_ms()
        return state_dict, timestamp_dict

    def get_camera_extrinsics(self, state_dict):
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
                "camera_type": {
                    camera_id: str,  # Camera type identifier
                    ...
                },
                "camera_extrinsics": {
                    camera_id: numpy.ndarray,  # 4x4 transformation matrix (base_link to camera)
                    camera_id + "_gripper_offset": numpy.ndarray,  # Original extrinsics for hand camera
                    ...
                },
                "camera_intrinsics": {
                    camera_id: numpy.ndarray,  # 3x3 intrinsic matrix (K)
                    ...
                } or None,  # None if no cameras available
                "timestamp": {
                    "robot_state": {
                        "robot_polymetis_t": int,  # Polymetis timestamp (nanoseconds)
                        "robot_pub_t": int,  # Message header timestamp (nanoseconds)
                        "robot_received_t": int,  # Message received time (nanoseconds, ROS time)
                        "robot_end_t": int,  # Processing end time (nanoseconds, ROS time)
                        "read_start": int,  # get_state() start time (milliseconds, system time)
                        "read_end": int,  # get_state() end time (milliseconds, system time)
                    },
                    "cameras": {
                        "{camera_id}_read_start": int,  # Start time (nanoseconds, ROS time)
                        "{camera_id}_frame_received": int,  # Frame received time (nanoseconds, ROS time)
                        "{camera_id}_read_end": int,  # End time (nanoseconds, ROS time)
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

        # Camera Readings #
        camera_obs, camera_timestamp = self.read_cameras()
        obs_dict.update(camera_obs)
        obs_dict["timestamp"]["cameras"] = camera_timestamp

        # Camera Info #
        obs_dict["camera_type"] = deepcopy(self.camera_type_dict)
        extrinsics = self.get_camera_extrinsics(state_dict)
        obs_dict["camera_extrinsics"] = extrinsics

        # Handle camera intrinsics - return None if no cameras available
        intrinsics = {}
        if self.camera_reader is not None and hasattr(self.camera_reader, 'camera_dict'):
            try:
                for cam in self.camera_reader.camera_dict.values():
                    cam_intr_info = cam.get_intrinsics()
                    for (full_cam_id, info) in cam_intr_info.items():
                        intrinsics[full_cam_id] = info["cameraMatrix"]
            except Exception:
                intrinsics = None
        else:
            intrinsics = None
        obs_dict["camera_intrinsics"] = intrinsics

        return obs_dict
