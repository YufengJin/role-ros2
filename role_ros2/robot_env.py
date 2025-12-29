from copy import deepcopy

import gym
import numpy as np

from role_ros2.calibration.calibration_utils import load_calibration_info
from role_ros2.camera_utils.info import camera_type_dict
from role_ros2.misc.parameters import hand_camera_id, nuc_ip
from role_ros2.misc.time import time_ms
from role_ros2.misc.transformations import change_pose_frame
from role_ros2.franka.robot import FrankaRobot

# Note: MultiCameraWrapper and ServerInterface are not available in role_ros2
# Camera functionality should be implemented via ROS2CameraSubscriber or similar
# ServerInterface functionality should be replaced with ROS2 service calls


class RobotEnv(gym.Env):
    def __init__(self, action_space="cartesian_velocity", gripper_action_space=None, camera_kwargs={}, do_reset=True, node=None):
        # Initialize Gym Environment
        super().__init__()

        # Define Action Space #
        assert action_space in ["cartesian_position", "joint_position", "cartesian_velocity", "joint_velocity"]
        self.action_space = action_space
        self.gripper_action_space = gripper_action_space
        self.check_action_range = "velocity" in action_space

        # Robot Configuration
        self.reset_joints = np.array([0, -1 / 5 * np.pi, 0, -4 / 5 * np.pi, 0, 3 / 5 * np.pi, 0.0])
        self.randomize_low = np.array([-0.1, -0.2, -0.1, -0.3, -0.3, -0.3])
        self.randomize_high = np.array([0.1, 0.2, 0.1, 0.3, 0.3, 0.3])
        self.DoF = 7 if ("cartesian" in action_space) else 8
        self.control_hz = 15

        # Initialize robot (always use ROS2-based FrankaRobot)
        # Note: launch_controller and launch_robot are no longer needed
        # Robot is launched via polymetis_bridge_node
        self._robot = FrankaRobot(node=node)

        # Create Cameras
        # Note: MultiCameraWrapper is not available in role_ros2
        # For now, camera functionality is disabled
        # TODO: Implement camera support using ROS2CameraSubscriber or similar
        self.camera_reader = None  # MultiCameraWrapper(camera_kwargs)  # Not available
        self.calibration_dict = load_calibration_info()
        self.camera_type_dict = camera_type_dict

        # Reset Robot
        if do_reset:
            self.reset()

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

    def reset(self, randomize=False):
        """
        Reset robot to home position.
        
        Args:
            randomize: If True, add random cartesian noise to reset position.
                      The randomization is handled by polymetis_bridge_node's reset service,
                      which applies noise using the same randomize_low/randomize_high range.
        
        Note: The reset service executes asynchronously in the background.
              This method returns immediately after the command is accepted.
        """
        # Use reset service - randomization is handled internally by polymetis_bridge_node
        # The service applies cartesian noise via _add_cartesian_noise_to_joints()
        # using the same noise range as this class (randomize_low, randomize_high)
        self._robot.reset(randomize=randomize)

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

        intrinsics = {}
        for cam in self.camera_reader.camera_dict.values():
            cam_intr_info = cam.get_intrinsics()
            for (full_cam_id, info) in cam_intr_info.items():
                intrinsics[full_cam_id] = info["cameraMatrix"]
        obs_dict["camera_intrinsics"] = intrinsics

        return obs_dict
