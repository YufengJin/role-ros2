#!/usr/bin/env python3
"""
Collect Trajectory Node - ROS2 Node for collecting robot trajectories using VR controller.

This node implements the collect_trajectory functionality as a ROS2 node:
1. Initializes RobotEnv and VRPolicy
2. Continuously collects observations and executes actions
3. Provides debug messages for monitoring

Usage:
    ros2 run role_ros2 collect_trajectory_node --ros-args -p action_space:=cartesian_velocity

Author: Role-ROS2 Team
"""

import time
import traceback
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node

from role_ros2.controllers.oculus_controller import VRPolicy
from role_ros2.robot_env import RobotEnv
from role_ros2.misc.time import time_ms


class CollectTrajectoryNode(Node):
    """
    ROS2 Node for collecting robot trajectories using VR controller.
    
    This node:
    1. Initializes RobotEnv and VRPolicy
    2. Continuously collects observations from RobotEnv
    3. Computes actions from VRPolicy
    4. Executes actions on RobotEnv
    5. Provides debug messages for monitoring
    """
    
    def __init__(self):
        super().__init__('collect_trajectory_node')
        
        # Declare parameters
        self.declare_parameter('action_space', 'cartesian_velocity')
        self.declare_parameter('gripper_action_space', None)
        self.declare_parameter('control_hz', 15.0)
        self.declare_parameter('reset_robot_on_start', True)
        self.declare_parameter('randomize_reset', False)
        self.declare_parameter('wait_for_controller', True)  # Wait for controller grip to enable movement
        self.declare_parameter('debug_log_frequency', 1.0)  # Log debug messages every N seconds
        self.declare_parameter('disable_rotation', False)  # Disable rotation control (only test position control)
        self.declare_parameter('invert_z_axis', False)  # Invert Z axis direction (fix joystick down -> robot up issue)
        self.declare_parameter('auto_reset_on_completion', True)  # Auto reset robot after trajectory completion (SUCCESS or FAILURE)
        self.declare_parameter('continue_after_reset', False)  # Continue collecting new trajectory after reset (loop mode)
        
        # VR Policy parameters
        self.declare_parameter('right_controller', True)
        self.declare_parameter('max_lin_vel', 1.0)
        self.declare_parameter('max_rot_vel', 1.0)
        self.declare_parameter('max_gripper_vel', 1.0)
        self.declare_parameter('spatial_coeff', 1.0)
        self.declare_parameter('pos_action_gain', 5.0)
        self.declare_parameter('rot_action_gain', 2.0)
        self.declare_parameter('gripper_action_gain', 3.0)
        
        # Get parameters
        action_space = self.get_parameter('action_space').get_parameter_value().string_value
        gripper_action_space = self.get_parameter('gripper_action_space').get_parameter_value().string_value
        if gripper_action_space == '':
            gripper_action_space = None
        control_hz = self.get_parameter('control_hz').get_parameter_value().double_value
        reset_robot_on_start = self.get_parameter('reset_robot_on_start').get_parameter_value().bool_value
        randomize_reset = self.get_parameter('randomize_reset').get_parameter_value().bool_value
        wait_for_controller = self.get_parameter('wait_for_controller').get_parameter_value().bool_value
        debug_log_frequency = self.get_parameter('debug_log_frequency').get_parameter_value().double_value
        
        # VR Policy parameters
        right_controller = self.get_parameter('right_controller').get_parameter_value().bool_value
        max_lin_vel = self.get_parameter('max_lin_vel').get_parameter_value().double_value
        max_rot_vel = self.get_parameter('max_rot_vel').get_parameter_value().double_value
        max_gripper_vel = self.get_parameter('max_gripper_vel').get_parameter_value().double_value
        spatial_coeff = self.get_parameter('spatial_coeff').get_parameter_value().double_value
        pos_action_gain = self.get_parameter('pos_action_gain').get_parameter_value().double_value
        rot_action_gain = self.get_parameter('rot_action_gain').get_parameter_value().double_value
        gripper_action_gain = self.get_parameter('gripper_action_gain').get_parameter_value().double_value
        
        # Get additional parameters
        disable_rotation = self.get_parameter('disable_rotation').get_parameter_value().bool_value
        invert_z_axis = self.get_parameter('invert_z_axis').get_parameter_value().bool_value
        auto_reset_on_completion = self.get_parameter('auto_reset_on_completion').get_parameter_value().bool_value
        continue_after_reset = self.get_parameter('continue_after_reset').get_parameter_value().bool_value
        
        # Store parameters
        self.control_hz = control_hz
        self.wait_for_controller = wait_for_controller
        self.debug_log_frequency = debug_log_frequency
        self.disable_rotation = disable_rotation
        self.invert_z_axis = invert_z_axis
        self.auto_reset_on_completion = auto_reset_on_completion
        self.continue_after_reset = continue_after_reset
        self._last_debug_log_time = time.time()
        self._num_steps = 0
        self._trajectory_completed = False
        self._robot_state_after = None
        
        # Initialize RobotEnv
        self.get_logger().info("=" * 70)
        self.get_logger().info("Initializing RobotEnv...")
        self.get_logger().info(f"  Action space: {action_space}")
        self.get_logger().info(f"  Gripper action space: {gripper_action_space}")
        try:
            self.env = RobotEnv(
                action_space=action_space,
                gripper_action_space=gripper_action_space,
                do_reset=False,  # We'll reset manually if needed
                node=self
            )
            self.get_logger().info("✅ RobotEnv initialized successfully")
        except Exception as e:
            self.get_logger().error(f"❌ Failed to initialize RobotEnv: {e}\n{traceback.format_exc()}")
            raise
        
        # Initialize VRPolicy
        self.get_logger().info("Initializing VRPolicy...")
        self.get_logger().info(f"  Controller: {'right' if right_controller else 'left'}")
        try:
            self.controller = VRPolicy(
                right_controller=right_controller,
                max_lin_vel=max_lin_vel,
                max_rot_vel=max_rot_vel,
                max_gripper_vel=max_gripper_vel,
                spatial_coeff=spatial_coeff,
                pos_action_gain=pos_action_gain,
                rot_action_gain=rot_action_gain,
                gripper_action_gain=gripper_action_gain,
            )
            self.get_logger().info("✅ VRPolicy initialized successfully")
        except Exception as e:
            self.get_logger().error(f"❌ Failed to initialize VRPolicy: {e}\n{traceback.format_exc()}")
            raise
        
        # Reset controller state
        self.controller.reset_state()
        
        # Reset robot if requested
        if reset_robot_on_start:
            self.get_logger().info("Resetting robot to home position...")
            try:
                self.env.reset(randomize=randomize_reset)
                self.get_logger().info("✅ Robot reset completed")
            except Exception as e:
                self.get_logger().warn(f"⚠️  Robot reset failed: {e}")
        
        # Create timer for control loop
        timer_period = 1.0 / max(control_hz, 1e-6)
        self._control_timer = self.create_timer(timer_period, self._control_loop)
        
        self.get_logger().info("=" * 70)
        self.get_logger().info("✅ CollectTrajectoryNode initialized successfully!")
        self.get_logger().info(f"  Control frequency: {control_hz} Hz")
        self.get_logger().info(f"  Debug log frequency: {debug_log_frequency} Hz")
        if self.disable_rotation:
            self.get_logger().info("  ⚠️  Rotation control DISABLED (position control only)")
        if self.invert_z_axis:
            self.get_logger().info("  ⚠️  Z axis INVERTED (fixing joystick down -> robot up issue)")
        if self.auto_reset_on_completion:
            self.get_logger().info("  🔄 Auto reset ENABLED (robot will reset after trajectory completion)")
            if self.continue_after_reset:
                self.get_logger().info("  🔁 Loop mode ENABLED (will continue collecting after reset)")
        self.get_logger().info("=" * 70)
        self.get_logger().info("📋 Control Instructions:")
        self.get_logger().info("   • Hold GRIP button to enable movement")
        self.get_logger().info("   • Press A (right) or X (left) to mark success")
        self.get_logger().info("   • Press B (right) or Y (left) to mark failure")
        if self.disable_rotation:
            self.get_logger().info("   • ⚠️  Rotation is disabled - only position control is active")
        if self.auto_reset_on_completion:
            self.get_logger().info("   • 🔄 Robot will automatically reset after trajectory completion")
        self.get_logger().info("=" * 70)
    
    def _control_loop(self):
        """
        Main control loop: collect observation, compute action, execute action.
        
        This runs at control_hz frequency and implements the collect_trajectory logic.
        """
        # Skip if trajectory completed and waiting for reset
        if self._trajectory_completed:
            return
        
        try:
            # Collect controller info (matches original collect_trajectory logic)
            controller_info = self.controller.get_info()
            control_timestamps = {"step_start": time_ms()}
            
            # Calculate skip_action (matches original: wait_for_controller and (not movement_enabled))
            skip_action = self.wait_for_controller and (not controller_info["movement_enabled"])
            
            # Get Observation
            control_timestamps["obs_start"] = time_ms()
            obs = self.env.get_observation()
            control_timestamps["obs_end"] = time_ms()
            
            # Add controller info and skip_action to observation (matches original)
            obs["controller_info"] = controller_info
            obs["timestamp"] = obs.get("timestamp", {})
            obs["timestamp"]["skip_action"] = skip_action
            
            # Get Action from VRPolicy
            control_timestamps["policy_start"] = time_ms()
            action, controller_action_info = self.controller.forward(obs, include_info=True)
            control_timestamps["policy_end"] = time_ms()
            
            # Disable rotation if requested (for testing position control only)
            if self.disable_rotation:
                # Set rotational velocity to zero
                action = np.array(action)
                action[3:6] = 0.0  # [wx, wy, wz] = 0
                # Also update target orientation to current orientation to avoid confusion
                if "target_cartesian_position" in controller_action_info:
                    current_cartesian = obs["robot_state"]["cartesian_position"]
                    controller_action_info["target_cartesian_position"] = current_cartesian.copy()
            
            # Invert Z axis if requested (fix joystick down -> robot up issue)
            if self.invert_z_axis:
                action = np.array(action)
                action[2] = -action[2]  # Invert Z axis (vz)
                # Also update target position Z to reflect inversion
                if "target_cartesian_position" in controller_action_info:
                    # Note: We don't invert target position, just the action
                    pass
            
            # Apply skip_action to action for debugging (show actual action that will be sent)
            actual_action = np.zeros_like(action) if skip_action else action
            
            # Debug logging (periodic)
            current_time = time.time()
            if current_time - self._last_debug_log_time >= 1.0 / self.debug_log_frequency:
                self._log_debug_info(obs, actual_action, controller_info, controller_action_info, control_timestamps, skip_action)
                self._last_debug_log_time = current_time
            
            # Regularize Control Frequency (matches original: uses env.control_hz)
            control_timestamps["sleep_start"] = time_ms()
            comp_time = time_ms() - control_timestamps["step_start"]
            # Use env.control_hz to match original logic (should be same as self.control_hz)
            sleep_left = (1 / self.env.control_hz) - (comp_time / 1000)
            if sleep_left > 0:
                time.sleep(sleep_left)
            control_timestamps["sleep_end"] = time_ms()
            
            # Step Environment (matches original logic)
            control_timestamps["control_start"] = time_ms()
            if skip_action:
                # Movement disabled - send zero action (matches original)
                action_info = self.env.create_action_dict(np.zeros_like(action))
            else:
                # Execute action (matches original)
                action_info = self.env.step(action)
            action_info.update(controller_action_info)
            control_timestamps["control_end"] = time_ms()
            
            # Get robot state after action execution for debugging
            obs_after = self.env.get_observation()
            robot_state_after = obs_after.get("robot_state", {})
            cartesian_pos_after = robot_state_after.get("cartesian_position", [0.0] * 6)
            
            # Add control timestamps to observation (matches original)
            control_timestamps["step_end"] = time_ms()
            obs["timestamp"]["control"] = control_timestamps
            
            # Store after-action state for debug logging
            self._robot_state_after = cartesian_pos_after
            
            # Update step counter (matches original: before termination check)
            self._num_steps += 1
            
            # Check Termination
            if controller_info["success"]:
                self._trajectory_completed = True
                self.get_logger().info("=" * 70)
                self.get_logger().info("✅ Trajectory collection completed: SUCCESS")
                self.get_logger().info(f"   Total steps: {self._num_steps}")
                self.get_logger().info("=" * 70)
                # Stop timer to end trajectory
                self._control_timer.cancel()
                # Auto reset if enabled
                if self.auto_reset_on_completion:
                    self._handle_trajectory_completion("SUCCESS")
            elif controller_info["failure"]:
                self._trajectory_completed = True
                self.get_logger().info("=" * 70)
                self.get_logger().info("❌ Trajectory collection completed: FAILURE")
                self.get_logger().info(f"   Total steps: {self._num_steps}")
                self.get_logger().info("=" * 70)
                # Stop timer to end trajectory
                self._control_timer.cancel()
                # Auto reset if enabled
                if self.auto_reset_on_completion:
                    self._handle_trajectory_completion("FAILURE")
            
        except Exception as e:
            self.get_logger().error(f"Error in control loop: {e}\n{traceback.format_exc()}")
    
    def _handle_trajectory_completion(self, result: str):
        """
        Handle trajectory completion: reset robot and optionally continue collecting.
        
        Args:
            result: "SUCCESS" or "FAILURE"
        """
        try:
            self.get_logger().info("")
            self.get_logger().info("=" * 70)
            self.get_logger().info(f"🔄 Auto-resetting robot after {result}...")
            self.get_logger().info("=" * 70)
            
            # Reset robot using service (via RobotEnv.reset())
            self.env.reset(randomize=False)
            
            self.get_logger().info("✅ Robot reset completed")
            
            # Reset controller state for next trajectory
            self.controller.reset_state()
            self._num_steps = 0
            self._trajectory_completed = False
            
            if self.continue_after_reset:
                self.get_logger().info("")
                self.get_logger().info("=" * 70)
                self.get_logger().info("🔁 Continuing to collect next trajectory...")
                self.get_logger().info("   • Hold GRIP button to enable movement")
                self.get_logger().info("   • Press A (right) or X (left) to mark success")
                self.get_logger().info("   • Press B (right) or Y (left) to mark failure")
                self.get_logger().info("=" * 70)
                # Restart control timer
                timer_period = 1.0 / max(self.control_hz, 1e-6)
                self._control_timer = self.create_timer(timer_period, self._control_loop)
            else:
                self.get_logger().info("")
                self.get_logger().info("=" * 70)
                self.get_logger().info("✅ Ready for next trajectory collection")
                self.get_logger().info("   Node will continue running. Press Ctrl+C to exit.")
                self.get_logger().info("=" * 70)
                
        except Exception as e:
            self.get_logger().error(f"Error handling trajectory completion: {e}\n{traceback.format_exc()}")
    
    def _log_debug_info(self, obs, action, controller_info, controller_action_info, timestamps, skip_action=False):
        """
        Log debug information for monitoring with detailed VRPolicy calculation states.
        
        Args:
            obs: Observation dictionary
            action: Action array (actual action that will be sent)
            controller_info: Controller info dictionary
            controller_action_info: Controller action info dictionary
            timestamps: Timing information
            skip_action: Whether action is being skipped (movement disabled)
        """
        try:
            robot_state = obs.get("robot_state", {})
            cartesian_pos = robot_state.get("cartesian_position", [0.0] * 6)
            gripper_pos = robot_state.get("gripper_position", 0.0)
            joint_pos = robot_state.get("joint_positions", [0.0] * 7)
            
            # Compute timing
            obs_time = timestamps.get("obs_end", 0) - timestamps.get("obs_start", 0)
            policy_time = timestamps.get("policy_end", 0) - timestamps.get("policy_start", 0)
            control_time = timestamps.get("control_end", 0) - timestamps.get("control_start", 0)
            total_time = timestamps.get("control_end", 0) - timestamps.get("step_start", 0)
            
            # Extract action components
            if len(action) >= 7:
                lin_vel = action[:3]
                rot_vel = action[3:6]
                gripper_vel = action[6]
            else:
                lin_vel = [0.0] * 3
                rot_vel = [0.0] * 3
                gripper_vel = 0.0
            
            # Extract target position if available
            target_cartesian = controller_action_info.get("target_cartesian_position", [0.0] * 6)
            target_gripper = controller_action_info.get("target_gripper_position", 0.0)
            
            # Get VRPolicy internal states for detailed debugging
            vr_state = None
            vr_origin = None
            robot_origin = None
            robot_pos_offset = None
            target_pos_offset = None
            pos_action_raw = None
            pos_action_scaled = None
            
            if hasattr(self.controller, 'vr_state') and self.controller.vr_state is not None:
                vr_state = self.controller.vr_state
            if hasattr(self.controller, 'vr_origin') and self.controller.vr_origin is not None:
                vr_origin = self.controller.vr_origin
            if hasattr(self.controller, 'robot_origin') and self.controller.robot_origin is not None:
                robot_origin = self.controller.robot_origin
                # Calculate offsets (matching VRPolicy._calculate_action logic)
                robot_pos = np.array(cartesian_pos[:3])
                robot_pos_offset = robot_pos - robot_origin["pos"]
                if vr_state is not None and vr_origin is not None:
                    target_pos_offset = vr_state["pos"] - vr_origin["pos"]
                    pos_action_raw = target_pos_offset - robot_pos_offset
                    pos_action_scaled = pos_action_raw * self.controller.pos_action_gain
            
            # Log debug information
            self.get_logger().info("=" * 70)
            self.get_logger().info(f"📊 Debug Info (Step {self._num_steps})")
            self.get_logger().info("=" * 70)
            
            # Robot State (Before Action)
            self.get_logger().info("🤖 Robot State (BEFORE action):")
            self.get_logger().info(f"   Position: [{cartesian_pos[0]:.6f}, {cartesian_pos[1]:.6f}, {cartesian_pos[2]:.6f}]")
            self.get_logger().info(f"   Orientation (euler): [{cartesian_pos[3]:.6f}, {cartesian_pos[4]:.6f}, {cartesian_pos[5]:.6f}]")
            self.get_logger().info(f"   Gripper position: {gripper_pos:.6f}")
            self.get_logger().info(f"   Joint positions: [{', '.join([f'{j:.6f}' for j in joint_pos[:3]])}...]")
            self.get_logger().info("")
            
            # Robot State (After Action)
            if self._robot_state_after is not None:
                pos_before = np.array(cartesian_pos[:3])
                pos_after = np.array(self._robot_state_after[:3])
                pos_displacement = pos_after - pos_before
                pos_displacement_norm = np.linalg.norm(pos_displacement)
                
                self.get_logger().info("🤖 Robot State (AFTER action):")
                self.get_logger().info(f"   Position: [{self._robot_state_after[0]:.6f}, {self._robot_state_after[1]:.6f}, {self._robot_state_after[2]:.6f}]")
                self.get_logger().info(f"   Orientation (euler): [{self._robot_state_after[3]:.6f}, {self._robot_state_after[4]:.6f}, {self._robot_state_after[5]:.6f}]")
                self.get_logger().info(f"   Position displacement: [{pos_displacement[0]:.6f}, {pos_displacement[1]:.6f}, {pos_displacement[2]:.6f}] (norm: {pos_displacement_norm:.6f} m)")
                self.get_logger().info("")
            
            # VR State
            self.get_logger().info("🎮 VR Controller State:")
            self.get_logger().info(f"   Movement enabled: {controller_info.get('movement_enabled', False)}")
            self.get_logger().info(f"   Controller on: {controller_info.get('controller_on', False)}")
            self.get_logger().info(f"   Success: {controller_info.get('success', False)}")
            self.get_logger().info(f"   Failure: {controller_info.get('failure', False)}")
            if vr_state is not None:
                self.get_logger().info(f"   VR Position: [{vr_state['pos'][0]:.6f}, {vr_state['pos'][1]:.6f}, {vr_state['pos'][2]:.6f}]")
                self.get_logger().info(f"   VR Orientation (quat): [{vr_state['quat'][0]:.6f}, {vr_state['quat'][1]:.6f}, {vr_state['quat'][2]:.6f}, {vr_state['quat'][3]:.6f}]")
                self.get_logger().info(f"   VR Gripper: {vr_state['gripper']:.6f}")
            else:
                self.get_logger().info("   VR State: Not available")
            self.get_logger().info("")
            
            # Origins
            self.get_logger().info("📍 Origins:")
            if robot_origin is not None:
                self.get_logger().info(f"   Robot Origin Position: [{robot_origin['pos'][0]:.6f}, {robot_origin['pos'][1]:.6f}, {robot_origin['pos'][2]:.6f}]")
            else:
                self.get_logger().info("   Robot Origin: Not set")
            if vr_origin is not None:
                self.get_logger().info(f"   VR Origin Position: [{vr_origin['pos'][0]:.6f}, {vr_origin['pos'][1]:.6f}, {vr_origin['pos'][2]:.6f}]")
            else:
                self.get_logger().info("   VR Origin: Not set")
            self.get_logger().info("")
            
            # VRPolicy Calculation Details
            self.get_logger().info("🔍 VRPolicy Calculation Details:")
            if robot_pos_offset is not None:
                self.get_logger().info(f"   Robot Position Offset: [{robot_pos_offset[0]:.6f}, {robot_pos_offset[1]:.6f}, {robot_pos_offset[2]:.6f}] (norm: {np.linalg.norm(robot_pos_offset):.6f})")
                self.get_logger().info(f"     → Robot moved: X={robot_pos_offset[0]:+.3f}, Y={robot_pos_offset[1]:+.3f}, Z={robot_pos_offset[2]:+.3f}")
            if target_pos_offset is not None:
                self.get_logger().info(f"   VR Position Offset: [{target_pos_offset[0]:.6f}, {target_pos_offset[1]:.6f}, {target_pos_offset[2]:.6f}] (norm: {np.linalg.norm(target_pos_offset):.6f})")
                self.get_logger().info(f"     → VR moved: X={target_pos_offset[0]:+.3f}, Y={target_pos_offset[1]:+.3f}, Z={target_pos_offset[2]:+.3f}")
            if pos_action_raw is not None:
                self.get_logger().info(f"   Raw Position Action: [{pos_action_raw[0]:.6f}, {pos_action_raw[1]:.6f}, {pos_action_raw[2]:.6f}] (norm: {np.linalg.norm(pos_action_raw):.6f})")
                self.get_logger().info(f"     → Action direction: X={pos_action_raw[0]:+.3f}, Y={pos_action_raw[1]:+.3f}, Z={pos_action_raw[2]:+.3f}")
                # Check if Z direction matches expected
                if robot_pos_offset is not None and target_pos_offset is not None:
                    expected_z_sign = np.sign(target_pos_offset[2] - robot_pos_offset[2])
                    actual_z_sign = np.sign(pos_action_raw[2])
                    if expected_z_sign != 0 and actual_z_sign != 0 and expected_z_sign != actual_z_sign:
                        self.get_logger().info(f"     ⚠️  Z direction mismatch: expected={expected_z_sign:+.0f}, actual={actual_z_sign:+.0f}")
            if pos_action_scaled is not None:
                self.get_logger().info(f"   Scaled Position Action (×{self.controller.pos_action_gain}): [{pos_action_scaled[0]:.6f}, {pos_action_scaled[1]:.6f}, {pos_action_scaled[2]:.6f}] (norm: {np.linalg.norm(pos_action_scaled):.6f})")
            self.get_logger().info(f"   Max Linear Velocity: {self.controller.max_lin_vel:.6f} m/s")
            self.get_logger().info("")
            
            # Action
            self.get_logger().info("⚡ Final Action (actual action sent to robot):")
            if skip_action:
                self.get_logger().info("   ⚠️  ACTION SKIPPED (Movement disabled - sending zero action)")
            self.get_logger().info(f"   Linear velocity: [{lin_vel[0]:.6f}, {lin_vel[1]:.6f}, {lin_vel[2]:.6f}] (norm: {np.linalg.norm(lin_vel):.6f})")
            if self.disable_rotation:
                self.get_logger().info(f"   Rotational velocity: [{rot_vel[0]:.6f}, {rot_vel[1]:.6f}, {rot_vel[2]:.6f}] (norm: {np.linalg.norm(rot_vel):.6f}) [DISABLED]")
            else:
                self.get_logger().info(f"   Rotational velocity: [{rot_vel[0]:.6f}, {rot_vel[1]:.6f}, {rot_vel[2]:.6f}] (norm: {np.linalg.norm(rot_vel):.6f})")
            self.get_logger().info(f"   Gripper velocity: {gripper_vel:.6f}")
            if self.invert_z_axis:
                self.get_logger().info(f"   ⚠️  Z axis inverted (original Z would be {-lin_vel[2]:.6f})")
            self.get_logger().info("")
            
            # Target
            self.get_logger().info("🎯 Target Pose:")
            self.get_logger().info(f"   Target position: [{target_cartesian[0]:.6f}, {target_cartesian[1]:.6f}, {target_cartesian[2]:.6f}]")
            self.get_logger().info(f"   Target orientation (euler): [{target_cartesian[3]:.6f}, {target_cartesian[4]:.6f}, {target_cartesian[5]:.6f}]")
            self.get_logger().info(f"   Target gripper: {target_gripper:.6f}")
            
            # Calculate relative position (target - current)
            relative_pos = np.array(target_cartesian[:3]) - np.array(cartesian_pos[:3])
            self.get_logger().info(f"   Relative position (target - current): [{relative_pos[0]:.6f}, {relative_pos[1]:.6f}, {relative_pos[2]:.6f}] (norm: {np.linalg.norm(relative_pos):.6f})")
            self.get_logger().info("")
            
            # Timing
            self.get_logger().info("⏱️  Timing:")
            self.get_logger().info(f"   Observation time: {obs_time:.2f} ms")
            self.get_logger().info(f"   Policy time: {policy_time:.2f} ms")
            self.get_logger().info(f"   Control time: {control_time:.2f} ms")
            self.get_logger().info(f"   Total time: {total_time:.2f} ms")
            if total_time > 0:
                self.get_logger().info(f"   Effective frequency: {1000.0 / total_time:.1f} Hz")
            self.get_logger().info("=" * 70)
            
        except Exception as e:
            self.get_logger().warn(f"Error logging debug info: {e}\n{traceback.format_exc()}")


def main(args=None):
    """Main function to run the Collect Trajectory Node."""
    rclpy.init(args=args)
    
    try:
        node = CollectTrajectoryNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}\n{traceback.format_exc()}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()

