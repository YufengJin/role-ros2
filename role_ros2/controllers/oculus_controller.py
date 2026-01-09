"""
Oculus VR Controller for role_ros2.

This module provides VRPolicy class for VR-based robot control,
copied from droid.controllers.oculus_controller and adapted for role_ros2.
"""

import time

import numpy as np
from oculus_reader.reader import OculusReader

from role_ros2.misc.subprocess_utils import run_threaded_command
from role_ros2.misc.transformations import (
    add_angles,
    euler_to_quat,
    quat_diff,
    quat_to_euler,
    rmat_to_quat,
)


def vec_to_reorder_mat(vec):
    """
    Convert reorder vector to transformation matrix.
    
    Args:
        vec: Reorder vector (e.g., [-2, -1, -3, 4])
    
    Returns:
        Transformation matrix
    """
    X = np.zeros((len(vec), len(vec)))
    for i in range(X.shape[0]):
        ind = int(abs(vec[i])) - 1
        X[i, ind] = np.sign(vec[i])
    return X


class VRPolicy:
    """
    VR Policy for robot control using Oculus Quest controllers.
    
    This class provides relative position control:
    - Robot movement is relative to the origin set when grip is first pressed
    - VR controller movement is relative to its origin (set at same time)
    - Action = (VR_offset - Robot_offset) * gain → velocity command
    """
    
    def __init__(
        self,
        right_controller: bool = True,
        max_lin_vel: float = 1,
        max_rot_vel: float = 1,
        max_gripper_vel: float = 1,
        spatial_coeff: float = 1,
        pos_action_gain: float = 5,
        rot_action_gain: float = 2,
        gripper_action_gain: float = 3,
        rmat_reorder: list = [-2, -1, -3, 4],
    ):
        """
        Initialize VR Policy.
        
        Args:
            right_controller: If True, use right controller; else use left controller
            max_lin_vel: Maximum linear velocity
            max_rot_vel: Maximum rotational velocity
            max_gripper_vel: Maximum gripper velocity
            spatial_coeff: Spatial coefficient for VR position scaling
            pos_action_gain: Position action gain
            rot_action_gain: Rotation action gain
            gripper_action_gain: Gripper action gain
            rmat_reorder: Reorder matrix vector
        """
        self.oculus_reader = OculusReader()
        self.vr_to_global_mat = np.eye(4)
        self.max_lin_vel = max_lin_vel
        self.max_rot_vel = max_rot_vel
        self.max_gripper_vel = max_gripper_vel
        self.spatial_coeff = spatial_coeff
        self.pos_action_gain = pos_action_gain
        self.rot_action_gain = rot_action_gain
        self.gripper_action_gain = gripper_action_gain
        self.global_to_env_mat = vec_to_reorder_mat(rmat_reorder)
        self.controller_id = "r" if right_controller else "l"
        self.reset_orientation = True
        self.reset_state()

        # Start State Listening Thread
        run_threaded_command(self._update_internal_state)

    def reset_state(self):
        """Reset internal state."""
        self._state = {
            "poses": {},
            "buttons": {"A": False, "B": False, "X": False, "Y": False},
            "movement_enabled": False,
            "controller_on": True,
        }
        self.update_sensor = True
        self.reset_origin = True
        self.robot_origin = None
        self.vr_origin = None
        self.vr_state = None

    def _update_internal_state(self, num_wait_sec=5, hz=50):
        """
        Update internal state from Oculus controller.
        
        This runs in a separate thread and continuously reads controller data.
        
        Args:
            num_wait_sec: Timeout for controller connection (seconds)
            hz: Update frequency (Hz)
        """
        last_read_time = time.time()
        while True:
            # Regulate Read Frequency
            time.sleep(1 / hz)

            # Read Controller
            time_since_read = time.time() - last_read_time
            poses, buttons = self.oculus_reader.get_transformations_and_buttons()
            self._state["controller_on"] = time_since_read < num_wait_sec
            if poses == {}:
                continue

            # Determine Control Pipeline
            toggled = self._state["movement_enabled"] != buttons[self.controller_id.upper() + "G"]
            self.update_sensor = self.update_sensor or buttons[self.controller_id.upper() + "G"]
            self.reset_orientation = self.reset_orientation or buttons[self.controller_id.upper() + "J"]
            self.reset_origin = self.reset_origin or toggled

            # Save Info
            self._state["poses"] = poses
            self._state["buttons"] = buttons
            self._state["movement_enabled"] = buttons[self.controller_id.upper() + "G"]
            self._state["controller_on"] = True
            last_read_time = time.time()

            # Update Definition Of "Forward"
            stop_updating = self._state["buttons"][self.controller_id.upper() + "J"] or self._state["movement_enabled"]
            if self.reset_orientation:
                rot_mat = np.asarray(self._state["poses"][self.controller_id])
                if stop_updating:
                    self.reset_orientation = False
                # Try to invert the rotation matrix, if not possible, then just use the identity matrix
                try:
                    rot_mat = np.linalg.inv(rot_mat)
                except:
                    print(f"exception for rot mat: {rot_mat}")
                    rot_mat = np.eye(4)
                    self.reset_orientation = True
                self.vr_to_global_mat = rot_mat

    def _process_reading(self):
        """Process latest reading to compute VR state."""
        rot_mat = np.asarray(self._state["poses"][self.controller_id])
        rot_mat = self.global_to_env_mat @ self.vr_to_global_mat @ rot_mat
        vr_pos = self.spatial_coeff * rot_mat[:3, 3]
        vr_quat = rmat_to_quat(rot_mat[:3, :3])
        
        # VR trigger value matches droid gripper convention:
        # - Trigger pressed (value ≈ 1) -> gripper close (position = 1)
        # - Trigger released (value ≈ 0) -> gripper open (position = 0)
        vr_gripper = self._state["buttons"]["rightTrig" if self.controller_id == "r" else "leftTrig"][0]

        self.vr_state = {"pos": vr_pos, "quat": vr_quat, "gripper": vr_gripper}

    def _limit_velocity(self, lin_vel, rot_vel, gripper_vel):
        """
        Scale down the linear and angular magnitudes of the action.
        
        Args:
            lin_vel: Linear velocity
            rot_vel: Rotational velocity
            gripper_vel: Gripper velocity
        
        Returns:
            Limited velocities
        """
        lin_vel_norm = np.linalg.norm(lin_vel)
        rot_vel_norm = np.linalg.norm(rot_vel)
        gripper_vel_norm = np.linalg.norm(gripper_vel)
        if lin_vel_norm > self.max_lin_vel:
            lin_vel = lin_vel * self.max_lin_vel / lin_vel_norm
        if rot_vel_norm > self.max_rot_vel:
            rot_vel = rot_vel * self.max_rot_vel / rot_vel_norm
        if gripper_vel_norm > self.max_gripper_vel:
            gripper_vel = gripper_vel * self.max_gripper_vel / gripper_vel_norm
        return lin_vel, rot_vel, gripper_vel

    def _calculate_action(self, state_dict, include_info=False):
        """
        Calculate action from robot state and VR state.
        
        Args:
            state_dict: Robot state dictionary
            include_info: If True, return additional info dict
        
        Returns:
            Action array (7D: [vx, vy, vz, wx, wy, wz, gripper_vel])
            If include_info=True, also returns info dict
        """
        # Read Sensor
        if self.update_sensor:
            self._process_reading()
            self.update_sensor = False

        # Read Observation
        robot_pos = np.array(state_dict["cartesian_position"][:3])
        robot_euler = state_dict["cartesian_position"][3:]
        robot_quat = euler_to_quat(robot_euler)
        robot_gripper = state_dict["gripper_position"]

        # Reset Origin On Release
        if self.reset_origin:
            self.robot_origin = {"pos": robot_pos, "quat": robot_quat}
            self.vr_origin = {"pos": self.vr_state["pos"], "quat": self.vr_state["quat"]}
            self.reset_origin = False

        # Calculate Positional Action
        robot_pos_offset = robot_pos - self.robot_origin["pos"]
        target_pos_offset = self.vr_state["pos"] - self.vr_origin["pos"]
        pos_action = target_pos_offset - robot_pos_offset

        # Calculate Euler Action
        robot_quat_offset = quat_diff(robot_quat, self.robot_origin["quat"])
        target_quat_offset = quat_diff(self.vr_state["quat"], self.vr_origin["quat"])
        quat_action = quat_diff(target_quat_offset, robot_quat_offset)
        euler_action = quat_to_euler(quat_action)

        # Calculate Gripper Action
        gripper_action = (self.vr_state["gripper"] * 1.5) - robot_gripper

        # Calculate Desired Pose
        target_pos = pos_action + robot_pos
        target_euler = add_angles(euler_action, robot_euler)
        target_cartesian = np.concatenate([target_pos, target_euler])
        target_gripper = self.vr_state["gripper"]

        # Scale Appropriately
        pos_action *= self.pos_action_gain
        euler_action *= self.rot_action_gain
        gripper_action *= self.gripper_action_gain
        lin_vel, rot_vel, gripper_vel = self._limit_velocity(pos_action, euler_action, gripper_action)

        # Prepare Return Values
        info_dict = {"target_cartesian_position": target_cartesian, "target_gripper_position": target_gripper}
        action = np.concatenate([lin_vel, rot_vel, [gripper_vel]])
        action = action.clip(-1, 1)

        # Return
        if include_info:
            return action, info_dict
        else:
            return action

    def get_info(self):
        """
        Get controller info.
        
        Returns:
            Dictionary with controller status information
        """
        return {
            "success": self._state["buttons"]["A"] if self.controller_id == 'r' else self._state["buttons"]["X"],
            "failure": self._state["buttons"]["B"] if self.controller_id == 'r' else self._state["buttons"]["Y"],
            "movement_enabled": self._state["movement_enabled"],
            "controller_on": self._state["controller_on"],
        }

    def forward(self, obs_dict, include_info=False):
        """
        Forward pass: compute action from observation.
        
        Args:
            obs_dict: Observation dictionary (must contain "robot_state")
            include_info: If True, return additional info dict
        
        Returns:
            Action array (7D: [vx, vy, vz, wx, wy, wz, gripper_vel])
            If include_info=True, also returns info dict
        """
        if self._state["poses"] == {}:
            action = np.zeros(7)
            if include_info:
                return action, {}
            else:
                return action
        return self._calculate_action(obs_dict["robot_state"], include_info=include_info)

