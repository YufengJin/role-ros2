#!/usr/bin/env python3
"""
ROS2 VR Policy Node

This node subscribes to Oculus controller data from oculus_reader_node,
computes actions based on VR input and robot state, and publishes actions
for teleoperation_node to execute.

Architecture:
    Oculus Quest Hardware
        ↓
    oculus_reader_node (publishes /oculus/* topics)
        ↓
    vr_policy_node (subscribes to /oculus/*, publishes /vr_policy/action)
        ↓
    teleoperation_node (subscribes to /vr_policy/action, executes on robot)

Usage:
    ros2 run role_ros2 vr_policy_node --ros-args -p right_controller:=true

Control Logic (matches droid VRPolicy):
    - Control Type: RELATIVE POSITION CONTROL
      * Robot movement is relative to the origin set when grip is first pressed
      * VR controller movement is relative to its origin (set at same time)
      * Action = (VR_offset - Robot_offset) * gain → velocity command
    
    - Movement Enable: Hold GRIP button (deadman switch)
      * movement_enabled=True when grip is pressed
    
    - Reset VR Orientation: Long press JOYSTICK (≥0.5 seconds)
      * Resets the "forward" direction mapping between VR and robot
      * Must hold joystick for 0.5s to trigger (prevents accidental resets)
    
    - Reset Origin: Toggle GRIP button
      * When grip is first pressed, sets robot_origin and vr_origin
      * This establishes the relative position mapping
    
    - Gripper Control: TRIGGER value (0-1) → velocity command
      * gripper_action = (trigger * 1.5) - current_gripper_position
      * Scaled by gripper_action_gain and limited by max_gripper_vel
"""

import numpy as np
import threading
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from role_ros2.msg import OculusButtons, VRPolicyAction, PolymetisRobotState
from role_ros2.misc.transformations import (
    add_angles,
    euler_to_quat,
    quat_diff,
    quat_to_euler,
    rmat_to_quat,
)
from scipy.spatial.transform import Rotation as R


def vec_to_reorder_mat(vec):
    """Convert reorder vector to transformation matrix."""
    X = np.zeros((len(vec), len(vec)))
    for i in range(X.shape[0]):
        ind = int(abs(vec[i])) - 1
        X[i, ind] = np.sign(vec[i])
    return X


def _pose_to_mat(pose_msg: PoseStamped) -> np.ndarray:
    """Convert PoseStamped to 4x4 transformation matrix."""
    q = pose_msg.pose.orientation
    t = pose_msg.pose.position
    rot = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    T = np.eye(4)
    T[:3, :3] = rot
    T[:3, 3] = [t.x, t.y, t.z]
    return T


class VRPolicyNode(Node):
    """
    ROS2 Node that computes VR policy actions from Oculus controller input.
    
    This node:
    1. Subscribes to Oculus controller data (pose and buttons)
    2. Subscribes to robot state (to compute relative actions)
    3. Computes actions based on VR input
    4. Publishes actions for teleoperation_node to execute
    """
    
    def __init__(self):
        super().__init__("vr_policy_node")
        
        # Parameters
        self.declare_parameter("right_controller", True)
        self.declare_parameter("publish_rate", 15.0)
        
        # VR Policy parameters
        self.declare_parameter("max_lin_vel", 1.0)
        self.declare_parameter("max_rot_vel", 1.0)
        self.declare_parameter("max_gripper_vel", 1.0)
        self.declare_parameter("spatial_coeff", 1.0)
        self.declare_parameter("pos_action_gain", 5.0)
        self.declare_parameter("rot_action_gain", 2.0)
        self.declare_parameter("gripper_action_gain", 3.0)
        self.declare_parameter("rmat_reorder", [-2, -1, -3, 4])
        
        # Topic parameters
        self.declare_parameter("oculus_right_pose_topic", "oculus/right_controller/pose")
        self.declare_parameter("oculus_left_pose_topic", "oculus/left_controller/pose")
        self.declare_parameter("oculus_buttons_topic", "oculus/buttons")
        self.declare_parameter("robot_state_topic", "polymetis/robot_state")
        self.declare_parameter("action_topic", "vr_policy/action")
        
        # Get parameters
        right_controller = bool(self.get_parameter("right_controller").value)
        publish_rate = float(self.get_parameter("publish_rate").value)
        
        max_lin_vel = float(self.get_parameter("max_lin_vel").value)
        max_rot_vel = float(self.get_parameter("max_rot_vel").value)
        max_gripper_vel = float(self.get_parameter("max_gripper_vel").value)
        spatial_coeff = float(self.get_parameter("spatial_coeff").value)
        pos_action_gain = float(self.get_parameter("pos_action_gain").value)
        rot_action_gain = float(self.get_parameter("rot_action_gain").value)
        gripper_action_gain = float(self.get_parameter("gripper_action_gain").value)
        rmat_reorder = list(self.get_parameter("rmat_reorder").value)
        
        oculus_right_pose_topic = str(self.get_parameter("oculus_right_pose_topic").value)
        oculus_left_pose_topic = str(self.get_parameter("oculus_left_pose_topic").value)
        oculus_buttons_topic = str(self.get_parameter("oculus_buttons_topic").value)
        robot_state_topic = str(self.get_parameter("robot_state_topic").value)
        action_topic = str(self.get_parameter("action_topic").value)
        
        # Store parameters
        self.controller_id = "r" if right_controller else "l"
        self.right_controller = right_controller
        
        # Control parameters
        self.max_lin_vel = max_lin_vel
        self.max_rot_vel = max_rot_vel
        self.max_gripper_vel = max_gripper_vel
        self.spatial_coeff = spatial_coeff
        self.pos_action_gain = pos_action_gain
        self.rot_action_gain = rot_action_gain
        self.gripper_action_gain = gripper_action_gain
        self.global_to_env_mat = vec_to_reorder_mat(rmat_reorder)
        
        # State
        self.vr_to_global_mat = np.eye(4)
        self.reset_orientation = True
        self.reset_state()
        
        # Long press detection for joystick reset
        self._joystick_press_start_time = None
        self._joystick_hold_duration = 0.5  # seconds - must hold for 0.5s to reset
        self._joystick_reset_triggered = False
        
        # Latest messages (thread-safe access)
        self._pose_lock = threading.Lock()
        self._buttons_lock = threading.Lock()
        self._robot_state_lock = threading.Lock()
        self._last_pose_msg: Optional[PoseStamped] = None
        self._last_buttons_msg: Optional[OculusButtons] = None
        self._last_robot_state_msg: Optional[PolymetisRobotState] = None
        self._last_pose_time = 0.0
        self._last_buttons_time = 0.0
        self._last_robot_state_time = 0.0
        self._controller_timeout = 5.0  # seconds
        
        # QoS profile for subscriptions (RELIABLE to match oculus_reader_node)
        qos_profile_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # Subscribe to Oculus topics (RELIABLE QoS to match publisher)
        pose_topic = oculus_right_pose_topic if right_controller else oculus_left_pose_topic
        self._pose_sub = self.create_subscription(
            PoseStamped, pose_topic, self._pose_callback, qos_profile_reliable
        )
        self._buttons_sub = self.create_subscription(
            OculusButtons, oculus_buttons_topic, self._buttons_callback, qos_profile_reliable
        )
        
        # Subscribe to robot state (RELIABLE QoS)
        self._robot_state_sub = self.create_subscription(
            PolymetisRobotState, robot_state_topic, self._robot_state_callback, qos_profile_reliable
        )
        
        # Publisher for actions (RELIABLE QoS)
        self._action_pub = self.create_publisher(
            VRPolicyAction, action_topic, qos_profile_reliable
        )
        
        # Timer to compute and publish actions
        self._timer = self.create_timer(1.0 / max(publish_rate, 1e-6), self._compute_and_publish_action)
        
        self.get_logger().info("=" * 70)
        self.get_logger().info("VRPolicyNode Initialization")
        self.get_logger().info("=" * 70)
        self.get_logger().info(f"  Controller: {self.controller_id} ({'right' if right_controller else 'left'})")
        self.get_logger().info(f"  Pose topic: {pose_topic}")
        self.get_logger().info(f"  Buttons topic: {oculus_buttons_topic}")
        self.get_logger().info(f"  Robot state topic: {robot_state_topic}")
        self.get_logger().info(f"  Action topic: {action_topic}")
        self.get_logger().info(f"  Publish rate: {publish_rate} Hz")
        self.get_logger().info("=" * 70)
        self.get_logger().info("📋 Control Logic:")
        self.get_logger().info("   • Control Type: RELATIVE POSITION CONTROL")
        self.get_logger().info("   • Movement Enable: Hold GRIP button (deadman switch)")
        self.get_logger().info("   • Reset VR Orientation: Long press JOYSTICK (≥0.5s)")
        self.get_logger().info("   • Reset Origin: Toggle GRIP button (when starting movement)")
        self.get_logger().info("   • Gripper Control: TRIGGER value (0-1) → velocity")
        self.get_logger().info("=" * 70)
        self.get_logger().info("✅ VRPolicyNode ready!")
    
    def reset_state(self):
        """Reset internal state."""
        self.update_sensor = True
        self.reset_origin = True
        self.robot_origin = None
        self.vr_origin = None
        self.vr_state = None
        self._state = {
            "poses": {},
            "buttons": {"A": False, "B": False, "X": False, "Y": False},
            "movement_enabled": False,
            "controller_on": False,
        }
    
    def _pose_callback(self, msg: PoseStamped):
        """Callback for pose messages."""
        with self._pose_lock:
            self._last_pose_msg = msg
            self._last_pose_time = time.time()
    
    def _buttons_callback(self, msg: OculusButtons):
        """Callback for button messages."""
        with self._buttons_lock:
            self._last_buttons_msg = msg
            self._last_buttons_time = time.time()
    
    def _robot_state_callback(self, msg: PolymetisRobotState):
        """Callback for robot state messages."""
        with self._robot_state_lock:
            self._last_robot_state_msg = msg
            self._last_robot_state_time = time.time()
    
    def _get_deadman_and_reset_flags(self, buttons: OculusButtons):
        """Extract deadman and reset flags from button message."""
        if self.right_controller:
            grip_pressed = bool(buttons.right_grip_pressed)
            joystick_pressed = bool(buttons.right_joystick_pressed)
            trigger_value = float(buttons.right_trigger_value)
        else:
            grip_pressed = bool(buttons.left_grip_pressed)
            joystick_pressed = bool(buttons.left_joystick_pressed)
            trigger_value = float(buttons.left_trigger_value)
        return grip_pressed, joystick_pressed, trigger_value
    
    def _update_internal_state(self):
        """Update internal state from latest messages."""
        current_time = time.time()
        
        # Check if controller is still active
        with self._pose_lock:
            pose_time = self._last_pose_time
            pose_msg = self._last_pose_msg
        
        with self._buttons_lock:
            buttons_time = self._last_buttons_time
            buttons_msg = self._last_buttons_msg
        
        # Check timeout
        time_since_pose = current_time - pose_time
        time_since_buttons = current_time - buttons_time
        controller_on = (time_since_pose < self._controller_timeout and 
                        time_since_buttons < self._controller_timeout)
        
        if not controller_on or pose_msg is None or buttons_msg is None:
            self._state["controller_on"] = False
            self._state["poses"] = {}
            return
        
        self._state["controller_on"] = True
        
        # Get button states
        grip_pressed, joystick_pressed, trigger_value = self._get_deadman_and_reset_flags(buttons_msg)
        
        # Long press detection for joystick reset (orientation reset)
        current_time = time.time()
        if joystick_pressed:
            if self._joystick_press_start_time is None:
                # Just started pressing
                self._joystick_press_start_time = current_time
                self._joystick_reset_triggered = False
            else:
                # Check if held long enough
                hold_duration = current_time - self._joystick_press_start_time
                if hold_duration >= self._joystick_hold_duration and not self._joystick_reset_triggered:
                    # Long press detected - trigger orientation reset
                    self.reset_orientation = True
                    self._joystick_reset_triggered = True
                    self.get_logger().info(
                        f"🔄 Joystick long press detected ({hold_duration:.2f}s) - Resetting VR orientation"
                    )
        else:
            # Joystick released
            if self._joystick_press_start_time is not None:
                hold_duration = current_time - self._joystick_press_start_time
                if hold_duration < self._joystick_hold_duration:
                    self.get_logger().debug(
                        f"Joystick pressed but released too soon ({hold_duration:.2f}s < {self._joystick_hold_duration}s)"
                    )
            self._joystick_press_start_time = None
            self._joystick_reset_triggered = False
        
        # Update flags
        toggled = (self._state["movement_enabled"] != grip_pressed)
        self.update_sensor = self.update_sensor or grip_pressed
        # Note: reset_orientation is now set by long press detection above
        self.reset_origin = self.reset_origin or toggled
        self._state["movement_enabled"] = grip_pressed
        
        # Update button states
        if self.right_controller:
            self._state["buttons"]["A"] = bool(buttons_msg.a)
            self._state["buttons"]["B"] = bool(buttons_msg.b)
        else:
            self._state["buttons"]["X"] = bool(buttons_msg.x)
            self._state["buttons"]["Y"] = bool(buttons_msg.y)
        
        # Update pose
        rot_mat = _pose_to_mat(pose_msg)
        self._state["poses"][self.controller_id] = rot_mat
        
        # Update orientation reset (when long press is detected)
        # Stop updating when joystick is released OR when movement is enabled
        stop_updating = (not joystick_pressed) or self._state["movement_enabled"]
        if self.reset_orientation:
            if stop_updating:
                # Reset completed - update the transformation matrix
                try:
                    self.vr_to_global_mat = np.linalg.inv(rot_mat)
                    self.get_logger().info(
                        f"✅ VR orientation reset complete. "
                        f"New forward direction set based on current controller pose."
                    )
                except Exception as e:
                    self.get_logger().warn(
                        f"⚠️  Failed to invert rotation matrix for orientation reset: {e}. "
                        f"Using identity matrix."
                    )
                    self.vr_to_global_mat = np.eye(4)
                    self.reset_orientation = True  # Retry on next update
                else:
                    self.reset_orientation = False
            else:
                # Still holding joystick - keep updating
                try:
                    # Continuously update while holding (for smooth reset)
                    self.vr_to_global_mat = np.linalg.inv(rot_mat)
                except Exception:
                    pass  # Ignore errors during continuous update
    
    def _process_reading(self):
        """Process latest reading to compute VR state."""
        if self.controller_id not in self._state["poses"]:
            return
        
        rot_mat = np.asarray(self._state["poses"][self.controller_id])
        rot_mat = self.global_to_env_mat @ self.vr_to_global_mat @ rot_mat
        vr_pos = self.spatial_coeff * rot_mat[:3, 3]
        vr_quat = rmat_to_quat(rot_mat[:3, :3])
        
        # Get gripper value from trigger
        with self._buttons_lock:
            if self._last_buttons_msg is not None:
                if self.right_controller:
                    trigger_value = float(self._last_buttons_msg.right_trigger_value)
                else:
                    trigger_value = float(self._last_buttons_msg.left_trigger_value)
            else:
                trigger_value = 0.0
        
        vr_gripper = float(np.clip(trigger_value, 0.0, 1.0))
        self.vr_state = {"pos": vr_pos, "quat": vr_quat, "gripper": vr_gripper}
    
    def _limit_velocity(self, lin_vel, rot_vel, gripper_vel):
        """Limit velocities to maximum values."""
        lin_vel_norm = np.linalg.norm(lin_vel)
        rot_vel_norm = np.linalg.norm(rot_vel)
        gripper_vel_norm = np.linalg.norm([gripper_vel])
        
        if lin_vel_norm > self.max_lin_vel and lin_vel_norm > 1e-9:
            lin_vel = lin_vel * (self.max_lin_vel / lin_vel_norm)
        if rot_vel_norm > self.max_rot_vel and rot_vel_norm > 1e-9:
            rot_vel = rot_vel * (self.max_rot_vel / rot_vel_norm)
        if gripper_vel_norm > self.max_gripper_vel and gripper_vel_norm > 1e-9:
            gripper_vel = gripper_vel * (self.max_gripper_vel / gripper_vel_norm)
        
        return lin_vel, rot_vel, gripper_vel
    
    def _calculate_action(self, robot_state_msg: PolymetisRobotState):
        """Calculate action from robot state and VR state."""
        # Update internal state
        self._update_internal_state()
        
        # Read sensor if needed (update VR state from latest pose)
        # Note: In droid VRPolicy, _process_reading is called when update_sensor=True
        # But we should also call it periodically to keep vr_state updated
        if self.update_sensor or self.vr_state is None:
            self._process_reading()
            self.update_sensor = False
        
        # Check if controller is on and movement is enabled
        if not self._state["controller_on"] or not self._state["movement_enabled"] or self.vr_state is None:
            return None, None
        
        # Validate VR state is available
        if self.vr_state is None or "pos" not in self.vr_state:
            self.get_logger().warn("VR state not available, cannot calculate action")
            return None, None
        
        # Read robot state from message
        # PolymetisRobotState has ee_position [x,y,z] and ee_euler [roll,pitch,yaw]
        if len(robot_state_msg.ee_position) >= 3 and len(robot_state_msg.ee_euler) >= 3:
            robot_pos = np.array([
                robot_state_msg.ee_position[0],
                robot_state_msg.ee_position[1],
                robot_state_msg.ee_position[2]
            ])
            robot_euler = np.array([
                robot_state_msg.ee_euler[0],
                robot_state_msg.ee_euler[1],
                robot_state_msg.ee_euler[2]
            ])
        else:
            # Fallback: use default values
            robot_pos = np.array([0.3, 0.0, 0.5])
            robot_euler = np.array([0.0, 0.0, 0.0])
        
        robot_quat = euler_to_quat(robot_euler)
        robot_gripper = float(robot_state_msg.gripper_position)
        
        # Reset origin on grip toggle (when grip button is pressed/released)
        # This resets the relative position mapping between VR and robot
        if self.reset_origin:
            self.robot_origin = {"pos": robot_pos.copy(), "quat": robot_quat.copy()}
            self.vr_origin = {"pos": self.vr_state["pos"].copy(), "quat": self.vr_state["quat"].copy()}
            self.get_logger().info(
                f"🔄 Origin reset: "
                f"robot_pos={robot_pos}, "
                f"vr_pos={self.vr_origin['pos']}"
            )
            self.reset_origin = False
        
        # Calculate positional action (relative control)
        # This follows the same logic as droid VRPolicy:
        # - robot_pos_offset: how much robot has moved from origin
        # - target_pos_offset: how much VR controller has moved from origin
        # - pos_action: difference = how much robot needs to move to match VR
        robot_pos_offset = robot_pos - self.robot_origin["pos"]
        target_pos_offset = self.vr_state["pos"] - self.vr_origin["pos"]
        pos_action = target_pos_offset - robot_pos_offset
        
        # Calculate rotational action (relative control)
        # Same relative control logic for rotation
        robot_quat_offset = quat_diff(robot_quat, self.robot_origin["quat"])
        target_quat_offset = quat_diff(self.vr_state["quat"], self.vr_origin["quat"])
        quat_action = quat_diff(target_quat_offset, robot_quat_offset)
        euler_action = quat_to_euler(quat_action)
        
        # Calculate gripper action (relative control)
        # VR gripper value (0-1) scaled by 1.5, relative to current robot gripper
        gripper_action = (self.vr_state["gripper"] * 1.5) - robot_gripper
        
        # Calculate target positions for info (BEFORE scaling, as per droid VRPolicy)
        # These are the absolute target positions the robot should reach
        # Note: Use unscaled pos_action and euler_action for target calculation
        target_pos = pos_action + robot_pos
        target_euler = add_angles(euler_action, robot_euler)
        target_cartesian = np.concatenate([target_pos, target_euler])
        
        # Verify control logic: if controller not moving, action should be small
        # Check if VR controller is stationary (within threshold)
        vr_movement = np.linalg.norm(target_pos_offset)
        robot_movement = np.linalg.norm(robot_pos_offset)
        action_magnitude = np.linalg.norm(pos_action)
        
        # Log control logic periodically for verification
        if hasattr(self, '_last_control_log_time'):
            if time.time() - self._last_control_log_time > 2.0:  # Log every 2 seconds
                self.get_logger().info(
                    f"📊 Control Logic Verification:\n"
                    f"   Robot pos: {robot_pos}\n"
                    f"   Robot origin: {self.robot_origin['pos']}\n"
                    f"   Robot pos offset: {robot_pos_offset} (norm: {robot_movement:.4f})\n"
                    f"   VR pos: {self.vr_state['pos']}\n"
                    f"   VR origin: {self.vr_origin['pos']}\n"
                    f"   VR pos offset: {target_pos_offset} (norm: {vr_movement:.4f})\n"
                    f"   Position action (unscaled): {pos_action} (norm: {action_magnitude:.4f})\n"
                    f"   Target position: {target_pos}\n"
                    f"   Rotation action (euler, unscaled): {euler_action}\n"
                    f"   Gripper action (unscaled): {gripper_action:.3f}\n"
                    f"   ✅ Expected: If VR not moving, VR offset ≈ 0, action ≈ -robot_offset"
                )
                
                # Validation check
                if vr_movement < 0.001:  # VR controller essentially stationary
                    if action_magnitude > 0.01:  # But action is significant
                        self.get_logger().warn(
                            f"⚠️  WARNING: VR controller stationary but action is large!\n"
                            f"   This may indicate origin mismatch or control logic error."
                        )
                    else:
                        self.get_logger().debug(
                            f"✅ Control logic correct: VR stationary → action ≈ 0"
                        )
                
                self._last_control_log_time = time.time()
        else:
            self._last_control_log_time = time.time()
        
        # Scale actions (AFTER calculating target positions)
        pos_action_scaled = pos_action * self.pos_action_gain
        euler_action_scaled = euler_action * self.rot_action_gain
        gripper_action_scaled = gripper_action * self.gripper_action_gain
        
        # Limit velocities
        lin_vel, rot_vel, gripper_vel = self._limit_velocity(
            pos_action_scaled, euler_action_scaled, gripper_action_scaled
        )
        
        # Prepare action (clip to [-1, 1] range as per VRPolicy)
        # Action format: [vx, vy, vz, wx, wy, wz, gripper_vel]
        # This is a VELOCITY command in cartesian_velocity action space
        action = np.concatenate([lin_vel, rot_vel, [gripper_vel]]).clip(-1.0, 1.0)
        
        info = {
            "target_cartesian_position": target_cartesian,
            "target_gripper_position": self.vr_state["gripper"],
        }
        
        # Additional validation: Check if target position makes sense
        # When controller is stationary, target should be close to current robot position
        target_distance = np.linalg.norm(target_pos - robot_pos)
        if vr_movement < 0.001 and target_distance > 0.01:
            self.get_logger().warn(
                f"⚠️  Validation Warning: VR stationary but target far from robot!\n"
                f"   Robot pos: {robot_pos}\n"
                f"   Target pos: {target_pos}\n"
                f"   Distance: {target_distance:.4f} m\n"
                f"   Robot offset: {robot_pos_offset}\n"
                f"   VR offset: {target_pos_offset}\n"
                f"   This may indicate incorrect origin or calculation error."
            )
        
        return action.astype(np.float64), info
    
    def _compute_and_publish_action(self):
        """Compute and publish action."""
        try:
            # Get latest robot state
            with self._robot_state_lock:
                robot_state_msg = self._last_robot_state_msg
            
            if robot_state_msg is None:
                # No robot state yet, publish zero action
                action_msg = VRPolicyAction()
                action_msg.header.stamp = self.get_clock().now().to_msg()
                action_msg.header.frame_id = "base_link"
                action_msg.action = [0.0] * 7
                action_msg.movement_enabled = False
                action_msg.controller_on = False
                action_msg.success = False
                action_msg.failure = False
                action_msg.target_cartesian_position = [0.0] * 6
                action_msg.target_gripper_position = 0.0
                self._action_pub.publish(action_msg)
                return
            
            # Calculate action
            action, info = self._calculate_action(robot_state_msg)
            
            # Create action message
            action_msg = VRPolicyAction()
            action_msg.header.stamp = self.get_clock().now().to_msg()
            action_msg.header.frame_id = "base_link"
            
            if action is not None:
                action_msg.action = action.tolist()
                action_msg.movement_enabled = self._state["movement_enabled"]
                action_msg.controller_on = self._state["controller_on"]
                action_msg.success = self._state["buttons"]["A"] if self.controller_id == "r" else self._state["buttons"]["X"]
                action_msg.failure = self._state["buttons"]["B"] if self.controller_id == "r" else self._state["buttons"]["Y"]
                
                if info is not None:
                    action_msg.target_cartesian_position = info["target_cartesian_position"].tolist()
                    action_msg.target_gripper_position = float(info["target_gripper_position"])
                else:
                    action_msg.target_cartesian_position = [0.0] * 6
                    action_msg.target_gripper_position = 0.0
            else:
                # No valid action (controller off or movement disabled)
                action_msg.action = [0.0] * 7
                action_msg.movement_enabled = False
                action_msg.controller_on = self._state["controller_on"]
                action_msg.success = False
                action_msg.failure = False
                action_msg.target_cartesian_position = [0.0] * 6
                action_msg.target_gripper_position = 0.0
            
            # Publish action
            self._action_pub.publish(action_msg)
            
        except Exception as e:
            import traceback
            self.get_logger().error(f"Error computing action: {e}\n{traceback.format_exc()}")


def main(args=None):
    rclpy.init(args=args)
    node = VRPolicyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

