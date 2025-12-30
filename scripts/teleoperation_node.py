#!/usr/bin/env python3
"""
Teleoperation Node (VR Policy Action -> Robot Execution)

This node subscribes to VR policy actions from vr_policy_node and
executes them on the robot using RobotEnv.

Architecture:
    vr_policy_node (publishes /vr_policy/action)
        ↓
    teleoperation_node (subscribes to /vr_policy/action, executes on robot)
        ↓
    RobotEnv (executes actions on robot)

Usage:
  # 1) Start robot bridge
  ros2 run role_ros2 polymetis_bridge_node

  # 2) Start Oculus reader (publishes /oculus/* topics)
  ros2 run role_ros2 oculus_reader_node --ros-args -p publish_rate:=50.0

  # 3) Start VR policy node (computes actions from VR input)
  ros2 run role_ros2 vr_policy_node --ros-args -p right_controller:=true

  # 4) Start teleop (executes actions on robot)
  ros2 run role_ros2 teleoperation_node

Teleoperation Control:
  - START: Hold controller GRIP button to enable motion (movement_enabled=True)
  - STOP: Release GRIP button to disable motion (movement_enabled=False)
  - Trigger: Controls gripper open/close (normalized 0..1 mapped to velocity)
  - Joystick (RJ/LJ): Reset VR orientation mapping (in vr_policy_node)

Reset Triggers:
  - On startup: If do_reset_on_start=True (default: True)
  - On success button (A/X): If reset_on_success=True (default: True)
  - On failure button (B/Y): If reset_on_failure=True (default: True)

Parameters:
  - action_space: Action space type (default: "cartesian_velocity")
  - action_topic: Topic to subscribe to (default: "vr_policy/action")
  - do_reset_on_start: Reset robot on startup (default: True)
  - reset_randomize: Randomize reset position (default: False)
  - reset_on_success: Reset when success button pressed (default: True)
  - reset_on_failure: Reset when failure button pressed (default: True)
"""

import numpy as np
import time
import rclpy
from rclpy.node import Node

from role_ros2.robot_env import RobotEnv
from role_ros2.msg import VRPolicyAction


class TeleoperationNode(Node):
    """
    Teleoperation node that subscribes to VR policy actions and executes them.
    
    This node:
    1. Subscribes to VR policy actions from vr_policy_node
    2. Executes actions on the robot using RobotEnv
    3. Provides robot control execution layer
    """

    def __init__(self):
        super().__init__("teleoperation_node")

        # Parameters
        self.declare_parameter("action_space", "cartesian_velocity")
        self.declare_parameter("action_topic", "vr_policy/action")
        
        # Robot reset parameters
        self.declare_parameter("do_reset_on_start", True)
        self.declare_parameter("reset_randomize", False)
        self.declare_parameter("reset_on_success", True)  # Reset when success button pressed
        self.declare_parameter("reset_on_failure", True)  # Reset when failure button pressed

        # Get parameters
        action_space = str(self.get_parameter("action_space").value)
        action_topic = str(self.get_parameter("action_topic").value)
        do_reset = bool(self.get_parameter("do_reset_on_start").value)
        reset_randomize = bool(self.get_parameter("reset_randomize").value)
        reset_on_success = bool(self.get_parameter("reset_on_success").value)
        reset_on_failure = bool(self.get_parameter("reset_on_failure").value)

        self.get_logger().info("=" * 70)
        self.get_logger().info("TeleoperationNode Initialization")
        self.get_logger().info("=" * 70)
        self.get_logger().info(f"  action_space: {action_space}")
        self.get_logger().info(f"  action_topic: {action_topic}")
        self.get_logger().info(f"  do_reset_on_start: {do_reset}")
        self.get_logger().info(f"  reset_randomize: {reset_randomize}")
        self.get_logger().info(f"  reset_on_success: {reset_on_success}")
        self.get_logger().info(f"  reset_on_failure: {reset_on_failure}")
        self.get_logger().info("=" * 70)
        self.get_logger().info("📋 Teleoperation Control:")
        self.get_logger().info("   • Teleoperation STARTS when: grip button is pressed (movement_enabled=True)")
        self.get_logger().info("   • Teleoperation STOPS when: grip button is released (movement_enabled=False)")
        self.get_logger().info("   • Reset triggers:")
        self.get_logger().info(f"     - On startup: {do_reset}")
        self.get_logger().info(f"     - On success button (A/X): {reset_on_success}")
        self.get_logger().info(f"     - On failure button (B/Y): {reset_on_failure}")
        self.get_logger().info("=" * 70)

        # Initialize RobotEnv (uses shared node)
        # NOTE: RobotEnv.__init__ will call reset() if do_reset=True
        # So we pass do_reset=False here to avoid double reset
        self.get_logger().info("Initializing RobotEnv...")
        self.get_logger().info(f"  Creating RobotEnv with do_reset={do_reset} (will reset in RobotEnv.__init__ if True)")
        
        self.env = RobotEnv(
            action_space=action_space,
            gripper_action_space="velocity",
            do_reset=do_reset,  # RobotEnv will handle reset
            node=self  # Share node for callbacks
        )
        self.get_logger().info("✅ RobotEnv initialized successfully")

        # Subscribe to VR policy actions (default RELIABLE QoS)
        self._action_sub = self.create_subscription(
            VRPolicyAction, action_topic, self._action_callback, 10
        )
        self.get_logger().info(f"✅ Subscribed to action topic: {action_topic}")
        
        # NOTE: Do NOT reset again here - RobotEnv.__init__ already called reset() if do_reset=True
        # This was causing double reset and long wait times
        if do_reset:
            self.get_logger().info("ℹ️  Robot reset was already performed during RobotEnv initialization")
            self.get_logger().info("   (Skipping duplicate reset to avoid double reset)")
        else:
            self.get_logger().info("ℹ️  Robot reset was skipped (do_reset_on_start=False)")

        self.get_logger().info("=" * 70)
        self.get_logger().info("✅ TeleoperationNode ready!")
        self.get_logger().info(f"   Waiting for VR policy actions on topic: {action_topic}")
        self.get_logger().info("=" * 70)
        
        # Track last action time for safety
        self._last_action_time = 0.0
        self._action_timeout = 1.0  # seconds
        self._action_count = 0
        self._last_log_time = time.time()
        
        # Teleoperation state tracking
        self._teleoperation_active = False  # Whether teleoperation is currently active
        self._last_movement_enabled = False
        self._reset_on_success = reset_on_success  # Reset robot when success button is pressed
        self._reset_on_failure = reset_on_failure  # Reset robot when failure button is pressed
        self._last_success_state = False
        self._last_failure_state = False

    def _action_callback(self, msg: VRPolicyAction):
        """Callback for VR policy action messages."""
        try:
            self._action_count += 1
            current_time = time.time()
            
            # Check if action is valid
            if len(msg.action) != 7:
                self.get_logger().warn(
                    f"❌ Invalid action length: {len(msg.action)}, expected 7. "
                    f"Action: {msg.action}"
                )
                return
            
            # Convert action to numpy array
            action = np.array(msg.action, dtype=np.float64)
            
            # Handle reset triggers (success/failure buttons)
            if self._reset_on_success and msg.success and not self._last_success_state:
                self.get_logger().info("🔄 Success button pressed - Resetting robot...")
                self.env.reset(randomize=False)
                self.get_logger().info("✅ Robot reset complete (success button)")
            
            if self._reset_on_failure and msg.failure and not self._last_failure_state:
                self.get_logger().info("🔄 Failure button pressed - Resetting robot...")
                self.env.reset(randomize=False)
                self.get_logger().info("✅ Robot reset complete (failure button)")
            
            self._last_success_state = msg.success
            self._last_failure_state = msg.failure
            
            # Detect teleoperation start/stop
            movement_enabled_changed = (msg.movement_enabled != self._last_movement_enabled)
            
            if movement_enabled_changed:
                if msg.movement_enabled:
                    self.get_logger().info("=" * 70)
                    self.get_logger().info("▶️  TELEOPERATION STARTED")
                    self.get_logger().info("   Grip button pressed - movement enabled")
                    self.get_logger().info("=" * 70)
                    self._teleoperation_active = True
                else:
                    self.get_logger().info("=" * 70)
                    self.get_logger().info("⏸️  TELEOPERATION STOPPED")
                    self.get_logger().info("   Grip button released - movement disabled")
                    self.get_logger().info("=" * 70)
                    self._teleoperation_active = False
            
            self._last_movement_enabled = msg.movement_enabled
            
            # Log first action and periodically (every 1 second)
            if self._action_count == 1 or (current_time - self._last_log_time) >= 1.0:
                self.get_logger().info(
                    f"📥 Received action #{self._action_count}: "
                    f"action=[{action[0]:.3f}, {action[1]:.3f}, {action[2]:.3f}, "
                    f"{action[3]:.3f}, {action[4]:.3f}, {action[5]:.3f}, {action[6]:.3f}], "
                    f"movement_enabled={msg.movement_enabled}, "
                    f"controller_on={msg.controller_on}, "
                    f"success={msg.success}, failure={msg.failure}"
                )
                self._last_log_time = current_time
            
            # Safety check: only execute if movement is enabled
            if not msg.movement_enabled:
                # Still update last action time to show we're receiving messages
                self._last_action_time = current_time
                if self._action_count <= 3:  # Log first few disabled actions
                    self.get_logger().debug(
                        f"⏸️  Movement disabled (grip not pressed), skipping action execution"
                    )
                return
            
            # Execute action
            self.get_logger().debug(
                f"▶️  Executing action: {[f'{x:.3f}' for x in action[:3]]}..."
            )
            self.env.step(action)
            self._last_action_time = current_time
            
            # Log execution periodically
            if self._action_count % 15 == 0:  # Every ~1 second at 15 Hz
                self.get_logger().info(
                    f"✅ Action executed #{self._action_count}: "
                    f"movement_enabled={msg.movement_enabled}, "
                    f"success={msg.success}, failure={msg.failure}"
                )
        
        except Exception as e:
            import traceback
            self.get_logger().error(
                f"❌ Error executing action #{self._action_count}: {e}\n"
                f"{traceback.format_exc()}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = TeleoperationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()


