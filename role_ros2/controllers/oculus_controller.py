"""
Oculus VR Controller for role_ros2.

This module provides VRPolicy class for VR-based robot control,
copied from droid.controllers.oculus_controller and adapted for role_ros2.
"""

import time
from typing import Dict, Tuple, Union

import numpy as np
from oculus_reader.reader import OculusReader

from role_ros2.controllers.base_controller import BaseController
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


class VRPolicy(BaseController):
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
        pos_vel_scale: float = 1.0,
        rot_vel_scale: float = 1.0,
        rmat_reorder: list = [-2, -1, -3, 4],
        mirror_xy: bool = False,
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
            pos_vel_scale: Scale factor for position velocity (0.0-1.0, reduces linear movement)
            rot_vel_scale: Scale factor for rotation velocity (0.0-1.0, reduces rotational movement)
            rmat_reorder: Reorder matrix vector
            mirror_xy: If True, mirror X and Y axes (forward<->backward, left<->right, Z unchanged)
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
        self.pos_vel_scale = pos_vel_scale
        self.rot_vel_scale = rot_vel_scale
        self.global_to_env_mat = vec_to_reorder_mat(rmat_reorder)
        self.controller_id = "r" if right_controller else "l"
        self.mirror_xy = mirror_xy  # Mirror X and Y axes (forward<->backward, left<->right)
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
        
        # Mirror X and Y axes if enabled (Z axis unchanged)
        # This makes: forward<->backward, left<->right, but up/down stays the same
        if self.mirror_xy:
            vr_pos[0] *= -1  # Mirror X axis
            vr_pos[1] *= -1  # Mirror Y axis
            # Z axis (vr_pos[2]) remains unchanged
        
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
        # Always read latest VR state (especially trigger) for immediate response
        # Don't wait for update_sensor flag - gripper needs real-time response
        # This ensures trigger release is detected immediately, not delayed
        if self._state["poses"] != {}:
            self._process_reading()
        
        # Mark sensor as updated to prevent duplicate processing
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

        # Calculate Gripper Action - IMPROVED LOGIC
        vr_target_gripper = self.vr_state["gripper"] * 1.5  # Target position (0 to 1.5, clamped to 1.0)
        vr_target_gripper = min(vr_target_gripper, 1.0)
        
        # Direct position error
        gripper_action = vr_target_gripper - robot_gripper
        
        # CRITICAL FIX: If trigger is released (vr_target_gripper < 0.1),
        # always try to open regardless of delayed robot state
        # This ensures immediate response when user releases trigger
        if vr_target_gripper < 0.1:
            # Trigger released - force open command (use direct velocity, not position error)
            # This ensures immediate response even with delayed robot state
            gripper_action = -0.3  # Direct open command (negative = open)
        # Add deadband to prevent small oscillations when gripper is at target
        elif abs(gripper_action) < 0.05:  # 5% deadband
            gripper_action = 0.0

        # Calculate Desired Pose
        target_pos = pos_action + robot_pos
        target_euler = add_angles(euler_action, robot_euler)
        target_cartesian = np.concatenate([target_pos, target_euler])
        target_gripper = vr_target_gripper

        # Scale Appropriately
        pos_action *= self.pos_action_gain
        euler_action *= self.rot_action_gain
        gripper_action *= self.gripper_action_gain
        lin_vel, rot_vel, gripper_vel = self._limit_velocity(pos_action, euler_action, gripper_action)

        # Apply velocity scales to reduce movement magnitude (especially rotation)
        lin_vel *= self.pos_vel_scale
        rot_vel *= self.rot_vel_scale

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


# ---------------------------------------------------------------------------
# VRBimanPolicy - Bimanual VR control using both Oculus controllers
# ---------------------------------------------------------------------------

# Bimanual action dimension (cartesian_velocity): 14D
DOF_BIMANUAL_CARTESIAN = 14  # left_lin(3)+left_rot(3)+right_lin(3)+right_rot(3)+left_grip+right_grip


class VRBimanPolicy(BaseController):
    """
    Bimanual VR Policy using both Oculus Quest controllers.

    - Left controller -> left arm (pose) + left gripper (trigger)
    - Right controller -> right arm (pose) + right gripper (trigger)
    - When mirror_arms=True: left controller -> right arm, right controller -> left arm
    - Each arm has independent origin reset when its grip is toggled
    - Output: 14D cartesian_velocity action for BimanualFrankaRobot

    Observation format (from BimanualFrankaRobot.get_robot_state()[0]):
        state_dict = {
            "left_arm": {
                "cartesian_position_local": [x,y,z,r,p,y],  # arm base frame (for IK)
                "cartesian_position": [x,y,z,r,p,y],        # world/base_link frame
                "joint_positions": [...], ...
            },
            "right_arm": { ... },
            "left_gripper_position": float,
            "right_gripper_position": float,
        }

    Uses cartesian_position_local (arm base frame) for VR-robot pose comparison
    so that the output velocity is in the same frame the IK solver expects.
    NOTE: VR offset is in world/env frame.  This works when arm-local and world
    frames share the same orientation (just translated).  If the arms are
    rotated relative to world an additional rotation transform is needed.
    """

    def __init__(
        self,
        max_lin_vel: float = 1,
        max_rot_vel: float = 1,
        max_gripper_vel: float = 1,
        spatial_coeff: float = 1,
        pos_action_gain: float = 5,
        rot_action_gain: float = 2,
        gripper_action_gain: float = 3,
        pos_vel_scale: float = 1.0,
        rot_vel_scale: float = 1.0,
        rmat_reorder: list = [-2, -1, -3, 4],
        mirror_xy: bool = False,
        mirror_arms: bool = False,
    ):
        """
        Initialize VR Bimanual Policy.

        Args:
            max_lin_vel: Maximum linear velocity per arm
            max_rot_vel: Maximum rotational velocity per arm
            max_gripper_vel: Maximum gripper velocity
            spatial_coeff: Spatial coefficient for VR position scaling
            pos_action_gain: Position action gain
            rot_action_gain: Rotation action gain
            gripper_action_gain: Gripper action gain
            pos_vel_scale: Scale factor for position velocity (0.0-1.0)
            rot_vel_scale: Scale factor for rotation velocity (0.0-1.0)
            rmat_reorder: Reorder matrix vector for VR frame alignment
            mirror_xy: If True, mirror X and Y axes for both controllers
            mirror_arms: If True, left joystick controls right arm, right joystick controls left arm
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
        self.pos_vel_scale = pos_vel_scale
        self.rot_vel_scale = rot_vel_scale
        self.global_to_env_mat = vec_to_reorder_mat(rmat_reorder)
        self.mirror_xy = mirror_xy
        self.mirror_arms = mirror_arms
        self.reset_orientation = True
        self.reset_state()

        run_threaded_command(self._update_internal_state)

    def reset_state(self):
        """Reset internal state for both arms."""
        self._state = {
            "poses": {},
            "buttons": {"A": False, "B": False, "X": False, "Y": False},
            "movement_enabled_left": False,
            "movement_enabled_right": False,
            "controller_on": True,
        }
        self.update_sensor = True
        self.reset_origin_left = True
        self.reset_origin_right = True
        self.robot_origin_left = None
        self.robot_origin_right = None
        self.vr_origin_left = None
        self.vr_origin_right = None
        self.vr_state_left = None
        self.vr_state_right = None

    def _update_internal_state(self, num_wait_sec=5, hz=50):
        """Update internal state from both Oculus controllers."""
        last_read_time = time.time()
        while True:
            time.sleep(1 / hz)
            time_since_read = time.time() - last_read_time
            poses, buttons = self.oculus_reader.get_transformations_and_buttons()
            self._state["controller_on"] = time_since_read < num_wait_sec
            if poses == {}:
                continue

            toggled_left = self._state["movement_enabled_left"] != buttons["LG"]
            toggled_right = self._state["movement_enabled_right"] != buttons["RG"]
            self.update_sensor = self.update_sensor or buttons["LG"] or buttons["RG"]
            self.reset_orientation = self.reset_orientation or buttons["LJ"] or buttons["RJ"]
            self.reset_origin_left = self.reset_origin_left or toggled_left
            self.reset_origin_right = self.reset_origin_right or toggled_right

            self._state["poses"] = poses
            self._state["buttons"] = buttons
            self._state["movement_enabled_left"] = buttons["LG"]
            self._state["movement_enabled_right"] = buttons["RG"]
            self._state["controller_on"] = True
            last_read_time = time.time()

            stop_updating = (
                self._state["buttons"]["LJ"]
                or self._state["buttons"]["RJ"]
                or self._state["movement_enabled_left"]
                or self._state["movement_enabled_right"]
            )
            if self.reset_orientation:
                rot_mat = np.eye(4)
                if "l" in self._state["poses"]:
                    rot_mat = np.asarray(self._state["poses"]["l"])
                elif "r" in self._state["poses"]:
                    rot_mat = np.asarray(self._state["poses"]["r"])
                if stop_updating:
                    self.reset_orientation = False
                try:
                    rot_mat = np.linalg.inv(rot_mat)
                except Exception:
                    rot_mat = np.eye(4)
                    self.reset_orientation = True
                self.vr_to_global_mat = rot_mat

    def _process_reading_side(self, controller_id: str):
        """Process one controller and return vr_state dict."""
        if controller_id not in self._state["poses"]:
            return None
        rot_mat = np.asarray(self._state["poses"][controller_id])
        rot_mat = self.global_to_env_mat @ self.vr_to_global_mat @ rot_mat
        vr_pos = self.spatial_coeff * rot_mat[:3, 3].copy()
        if self.mirror_xy:
            vr_pos[0] *= -1
            vr_pos[1] *= -1
        vr_quat = rmat_to_quat(rot_mat[:3, :3])
        trig_key = "rightTrig" if controller_id == "r" else "leftTrig"
        vr_gripper = self._state["buttons"].get(trig_key, [0.0])[0]
        return {"pos": vr_pos, "quat": vr_quat, "gripper": vr_gripper}

    def _process_reading(self):
        """Process both controllers."""
        if self._state["poses"] != {}:
            self.vr_state_left = self._process_reading_side("l")
            self.vr_state_right = self._process_reading_side("r")
        self.update_sensor = False

    def _limit_velocity(self, lin_vel, rot_vel, gripper_vel):
        """Limit velocity magnitudes."""
        lin_vel_norm = np.linalg.norm(lin_vel)
        rot_vel_norm = np.linalg.norm(rot_vel)
        gripper_vel_norm = np.abs(gripper_vel)
        if lin_vel_norm > self.max_lin_vel:
            lin_vel = lin_vel * self.max_lin_vel / lin_vel_norm
        if rot_vel_norm > self.max_rot_vel:
            rot_vel = rot_vel * self.max_rot_vel / rot_vel_norm
        if gripper_vel_norm > self.max_gripper_vel:
            gripper_vel = gripper_vel * self.max_gripper_vel / gripper_vel_norm
        return lin_vel, rot_vel, gripper_vel

    def _compute_arm_action(
        self,
        robot_pos: np.ndarray,
        robot_euler: np.ndarray,
        robot_gripper: float,
        vr_state: dict,
        robot_origin: dict,
        vr_origin: dict,
        reset_origin: bool,
    ) -> tuple:
        """
        Compute 7D action (lin_vel, rot_vel, gripper_vel) for one arm.

        robot_pos / robot_euler should be in arm-local frame so that the
        resulting velocity matches the IK solver's expected frame.

        Returns:
            (lin_vel, rot_vel, gripper_vel, new_robot_origin, new_vr_origin, reset_origin_consumed)
        """
        robot_quat = euler_to_quat(robot_euler)

        if reset_origin:
            robot_origin = {"pos": robot_pos.copy(), "quat": robot_quat.copy()}
            vr_origin = {"pos": vr_state["pos"].copy(), "quat": vr_state["quat"].copy()}
            reset_origin = False

        robot_pos_offset = robot_pos - robot_origin["pos"]
        target_pos_offset = vr_state["pos"] - vr_origin["pos"]
        pos_action = target_pos_offset - robot_pos_offset

        robot_quat_offset = quat_diff(robot_quat, robot_origin["quat"])
        target_quat_offset = quat_diff(vr_state["quat"], vr_origin["quat"])
        quat_action = quat_diff(target_quat_offset, robot_quat_offset)
        euler_action = quat_to_euler(quat_action)

        vr_target_gripper = min(vr_state["gripper"] * 1.5, 1.0)
        gripper_action = vr_target_gripper - robot_gripper
        if vr_target_gripper < 0.1:
            gripper_action = -0.3
        elif abs(gripper_action) < 0.05:
            gripper_action = 0.0

        pos_action *= self.pos_action_gain
        euler_action *= self.rot_action_gain
        gripper_action *= self.gripper_action_gain
        lin_vel, rot_vel, gripper_vel = self._limit_velocity(pos_action, euler_action, gripper_action)
        lin_vel *= self.pos_vel_scale
        rot_vel *= self.rot_vel_scale

        return lin_vel, rot_vel, gripper_vel, robot_origin, vr_origin, reset_origin

    def _calculate_action(self, state_dict: dict, include_info: bool = False):
        """
        Calculate 14D bimanual action from robot state and VR state.

        Robot positions are read from cartesian_position_local (arm base frame)
        so the output velocity is in the arm-local frame expected by the IK solver.

        Args:
            state_dict: BimanualFrankaRobot observation format with left_arm,
                        right_arm, left_gripper_position, right_gripper_position.
            include_info: If True, also return a diagnostics dict.
        """
        if self._state["poses"] != {}:
            self._process_reading()

        left_arm = state_dict.get("left_arm", {})
        right_arm = state_dict.get("right_arm", {})
        left_gripper = state_dict.get("left_gripper_position", 0.0)
        right_gripper = state_dict.get("right_gripper_position", 0.0)

        # Use cartesian_position_local (arm base frame) so that the resulting
        # velocity is in the same frame the IK solver expects.  For single-arm
        # setups local == world, but for bimanual the two frames differ.
        robot_pos_left = np.array(left_arm.get("cartesian_position_local", [0] * 6)[:3])
        robot_euler_left = np.array(left_arm.get("cartesian_position_local", [0] * 6)[3:])
        robot_pos_right = np.array(right_arm.get("cartesian_position_local", [0] * 6)[:3])
        robot_euler_right = np.array(right_arm.get("cartesian_position_local", [0] * 6)[3:])

        # Default VR state if controller missing
        vl = self.vr_state_left or {"pos": robot_pos_left, "quat": euler_to_quat(robot_euler_left), "gripper": 0.0}
        vr = self.vr_state_right or {"pos": robot_pos_right, "quat": euler_to_quat(robot_euler_right), "gripper": 0.0}
        # In mirror_arms mode: left joystick -> right arm, right joystick -> left arm
        if self.mirror_arms:
            vl, vr = vr, vl

        if self.robot_origin_left is None:
            self.robot_origin_left = {"pos": robot_pos_left.copy(), "quat": euler_to_quat(robot_euler_left)}
            self.vr_origin_left = {"pos": vl["pos"].copy(), "quat": vl["quat"].copy()}
        if self.robot_origin_right is None:
            self.robot_origin_right = {"pos": robot_pos_right.copy(), "quat": euler_to_quat(robot_euler_right)}
            self.vr_origin_right = {"pos": vr["pos"].copy(), "quat": vr["quat"].copy()}

        # In mirror_arms mode: left arm reset = right grip, right arm reset = left grip
        reset_left = self.reset_origin_right if self.mirror_arms else self.reset_origin_left
        reset_right = self.reset_origin_left if self.mirror_arms else self.reset_origin_right

        (
            lin_left,
            rot_left,
            grip_left,
            self.robot_origin_left,
            self.vr_origin_left,
            reset_left_done,
        ) = self._compute_arm_action(
            robot_pos_left,
            robot_euler_left,
            left_gripper,
            vl,
            self.robot_origin_left,
            self.vr_origin_left,
            reset_left,
        )

        (
            lin_right,
            rot_right,
            grip_right,
            self.robot_origin_right,
            self.vr_origin_right,
            reset_right_done,
        ) = self._compute_arm_action(
            robot_pos_right,
            robot_euler_right,
            right_gripper,
            vr,
            self.robot_origin_right,
            self.vr_origin_right,
            reset_right,
        )

        # Write back consumed reset flags (mirror_arms swaps which grip triggers which)
        if self.mirror_arms:
            self.reset_origin_right = reset_left_done
            self.reset_origin_left = reset_right_done
        else:
            self.reset_origin_left = reset_left_done
            self.reset_origin_right = reset_right_done

        # Zero out arm and gripper action when grip not pressed (movement disabled for that arm)
        # When only one controller is held, the other side must not receive any velocity command
        # In mirror_arms mode: left arm uses right grip, right arm uses left grip
        enabled_left = self._state["movement_enabled_right"] if self.mirror_arms else self._state["movement_enabled_left"]
        enabled_right = self._state["movement_enabled_left"] if self.mirror_arms else self._state["movement_enabled_right"]
        if not enabled_left:
            lin_left = np.zeros(3)
            rot_left = np.zeros(3)
            grip_left = 0.0
        if not enabled_right:
            lin_right = np.zeros(3)
            rot_right = np.zeros(3)
            grip_right = 0.0

        action = np.concatenate([lin_left, rot_left, lin_right, rot_right, [grip_left, grip_right]])
        action = action.clip(-1, 1)

        if include_info:
            info = {
                "target_cartesian_left": np.concatenate([vl["pos"], quat_to_euler(vl["quat"])]),
                "target_cartesian_right": np.concatenate([vr["pos"], quat_to_euler(vr["quat"])]),
                "target_gripper_left": min(vl["gripper"] * 1.5, 1.0),
                "target_gripper_right": min(vr["gripper"] * 1.5, 1.0),
            }
            return action, info
        return action

    def get_info(self) -> dict:
        """Get controller info for both hands."""
        return {
            "success": self._state["buttons"]["A"],
            "failure": self._state["buttons"]["B"],
            "movement_enabled_left": self._state["movement_enabled_left"],
            "movement_enabled_right": self._state["movement_enabled_right"],
            "controller_on": self._state["controller_on"],
        }

    def forward(self, obs_dict: dict, include_info: bool = False):
        """
        Forward pass: compute 14D bimanual action from observation.

        Args:
            obs_dict: Must contain "robot_state" with BimanualFrankaRobot format:
                - left_arm, right_arm (each with cartesian_position, cartesian_position_local, ...)
                - left_gripper_position, right_gripper_position
            include_info: If True, return additional info dict

        Returns:
            Action array (14D: [left_lin(3), left_rot(3), right_lin(3), right_rot(3), left_grip, right_grip])
        """
        if self._state["poses"] == {}:
            action = np.zeros(DOF_BIMANUAL_CARTESIAN)
            if include_info:
                return action, {}
            return action

        state_dict = obs_dict.get("robot_state", obs_dict)
        return self._calculate_action(state_dict, include_info=include_info)

