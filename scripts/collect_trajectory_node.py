#!/usr/bin/env python3
"""
Collect Trajectory Node - ROS2 Node for collecting robot trajectories using VR controller.

This node provides a ROS2-native interface for trajectory collection.

Usage:
    # Teleoperation only (no saving)
    ros2 run role_ros2 collect_trajectory_node
    
    # Save trajectory to folder
    ros2 run role_ros2 collect_trajectory_node --ros-args -p save_folder:=/path/to/save

Author: Role-ROS2 Team
"""

import os
import time
import traceback
from datetime import datetime
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node

from role_ros2.controllers.oculus_controller import VRPolicy
from role_ros2.robot_env import RobotEnv
from role_ros2.trajectory_utils.trajectory_writer import TrajectoryWriter
from role_ros2.misc.ros2_utils import get_ros_time_ns


class CollectTrajectoryNode(Node):
    """
    ROS2 Node for collecting robot trajectories using VR controller.
    
    This node:
    1. Initializes RobotEnv and VRPolicy (controller)
    2. Continuously collects observations from RobotEnv
    3. Computes actions from VRPolicy
    4. Executes actions on RobotEnv
    5. Optionally saves trajectory data
    """
    
    def __init__(self):
        super().__init__('collect_trajectory_node')
        
        # Declare parameters
        self.declare_parameter('save_folder', '')
        self.declare_parameter('action_space', 'cartesian_velocity')
        self.declare_parameter('control_hz', 15.0)
        self.declare_parameter('reset_robot', True)
        self.declare_parameter('randomize_reset', False)
        self.declare_parameter('wait_for_controller', True)
        self.declare_parameter('save_images', False)
        self.declare_parameter('loop', False)
        self.declare_parameter('right_controller', True)
        self.declare_parameter('horizon', -1)  # -1 for unlimited
        
        # Get parameters
        self.save_folder = self.get_parameter('save_folder').get_parameter_value().string_value
        action_space = self.get_parameter('action_space').get_parameter_value().string_value
        self.control_hz = self.get_parameter('control_hz').get_parameter_value().double_value
        reset_robot = self.get_parameter('reset_robot').get_parameter_value().bool_value
        randomize_reset = self.get_parameter('randomize_reset').get_parameter_value().bool_value
        self.wait_for_controller = self.get_parameter('wait_for_controller').get_parameter_value().bool_value
        self.save_images = self.get_parameter('save_images').get_parameter_value().bool_value
        self.loop = self.get_parameter('loop').get_parameter_value().bool_value
        right_controller = self.get_parameter('right_controller').get_parameter_value().bool_value
        horizon = self.get_parameter('horizon').get_parameter_value().integer_value
        self.horizon = None if horizon <= 0 else horizon
        
        # State variables
        self._num_steps = 0
        self._traj_count = 0
        self._trajectory_completed = False
        self._traj_writer: Optional[TrajectoryWriter] = None
        
        # Initialize RobotEnv
        self.get_logger().info("=" * 70)
        self.get_logger().info("🚀 Collect Trajectory Node")
        self.get_logger().info("=" * 70)
        self.get_logger().info(f"  Action space: {action_space}")
        self.get_logger().info(f"  Control Hz: {self.control_hz}")
        
        try:
            self.env = RobotEnv(
                action_space=action_space,
                do_reset=False,
                node=self,
            )
            self.env.control_hz = self.control_hz
            self.get_logger().info("✅ RobotEnv initialized")
        except Exception as e:
            self.get_logger().error(f"❌ Failed to initialize RobotEnv: {e}")
            raise
        
        # Initialize VRPolicy (controller)
        self.get_logger().info("Initializing VRPolicy controller...")
        try:
            self.controller = VRPolicy(right_controller=right_controller)
            self.get_logger().info("✅ VRPolicy initialized")
        except Exception as e:
            self.get_logger().error(f"❌ Failed to initialize VRPolicy: {e}")
            raise
        
        # Reset robot if requested
        if reset_robot:
            self.get_logger().info("🤖 Resetting robot to home position...")
            try:
                self.env.reset(randomize=randomize_reset)
                self.get_logger().info("✅ Robot reset completed")
            except Exception as e:
                self.get_logger().warn(f"⚠️  Robot reset failed: {e}")
        
        # Mode info
        if self.save_folder:
            os.makedirs(self.save_folder, exist_ok=True)
            self.get_logger().info(f"📁 Mode: Recording trajectory")
            self.get_logger().info(f"   Save folder: {self.save_folder}")
        else:
            self.get_logger().info("🎮 Mode: Teleoperation only (no saving)")
        
        # Start new trajectory
        self._start_new_trajectory()
        
        # Create control timer
        timer_period = 1.0 / max(self.control_hz, 1e-6)
        self._control_timer = self.create_timer(timer_period, self._control_loop)
        
        self.get_logger().info("=" * 70)
        self.get_logger().info("📋 Control Instructions:")
        self.get_logger().info("   • Hold GRIP button to enable movement")
        self.get_logger().info("   • Press A (right) or X (left) to mark SUCCESS")
        self.get_logger().info("   • Press B (right) or Y (left) to mark FAILURE")
        self.get_logger().info("=" * 70)
        self.get_logger().info("Ready! Waiting for controller input...")
    
    def _start_new_trajectory(self):
        """Start a new trajectory recording."""
        self._traj_count += 1
        self._num_steps = 0
        self._trajectory_completed = False
        
        # Reset controller state
        self.controller.reset_state()
        
        # Create trajectory writer if save folder is specified
        if self.save_folder:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            save_filepath = os.path.join(self.save_folder, f"trajectory_{timestamp}.h5")
            self._traj_writer = TrajectoryWriter(
                save_filepath, 
                metadata={"trajectory_id": self._traj_count},
                save_images=self.save_images
            )
            self.get_logger().info(f"📁 Saving trajectory to: {save_filepath}")
        else:
            self._traj_writer = None
    
    def _control_loop(self):
        """Main control loop."""
        if self._trajectory_completed:
            return
        
        try:
            # Get controller info
            controller_info = self.controller.get_info()
            skip_action = self.wait_for_controller and (not controller_info["movement_enabled"])
            
            # Use ROS time for timestamps (nanoseconds)
            control_timestamps = {"step_start": get_ros_time_ns(self)}
            
            # Get Observation
            obs = self.env.get_observation()
            obs["controller_info"] = controller_info
            if "timestamp" not in obs:
                obs["timestamp"] = {}
            obs["timestamp"]["skip_action"] = skip_action
            
            # Get Action
            control_timestamps["policy_start"] = get_ros_time_ns(self)
            action, controller_action_info = self.controller.forward(obs, include_info=True)
            control_timestamps["policy_end"] = get_ros_time_ns(self)
            
            # Regularize Control Frequency
            control_timestamps["sleep_start"] = get_ros_time_ns(self)
            comp_time_ns = get_ros_time_ns(self) - control_timestamps["step_start"]
            comp_time_s = comp_time_ns / 1e9
            sleep_left = (1 / self.env.control_hz) - comp_time_s
            if sleep_left > 0:
                time.sleep(sleep_left)
            
            # Step Environment
            control_timestamps["control_start"] = get_ros_time_ns(self)
            if skip_action:
                action_info = self.env.create_action_dict(np.zeros_like(action))
            else:
                action_info = self.env.step(action)
            action_info.update(controller_action_info)
            
            # Save Data
            control_timestamps["step_end"] = get_ros_time_ns(self)
            obs["timestamp"]["control"] = control_timestamps
            timestep = {"observation": obs, "action": action_info}
            if self._traj_writer is not None:
                self._traj_writer.write_timestep(timestep)
            
            # Check Termination
            self._num_steps += 1
            if self.horizon is not None:
                end_traj = self.horizon == self._num_steps
            else:
                end_traj = controller_info["success"] or controller_info["failure"]
            
            # Progress logging
            if self._num_steps % 50 == 0:
                movement_status = "🟢 MOVING" if controller_info["movement_enabled"] else "🔴 STOPPED"
                self.get_logger().info(f"Step {self._num_steps}: {movement_status}")
            
            # Handle trajectory end
            if end_traj:
                self._handle_trajectory_end(controller_info)
                
        except Exception as e:
            self.get_logger().error(f"Error in control loop: {e}\n{traceback.format_exc()}")
    
    def _handle_trajectory_end(self, controller_info: dict):
        """Handle trajectory completion."""
        self._trajectory_completed = True
        result = "SUCCESS ✅" if controller_info.get("success") else "FAILURE ❌"
        
        self.get_logger().info("")
        self.get_logger().info("=" * 70)
        self.get_logger().info(f"Trajectory #{self._traj_count} completed: {result}")
        self.get_logger().info(f"Total steps: {self._num_steps}")
        
        # Close trajectory writer
        if self._traj_writer is not None:
            self._traj_writer.close(metadata=controller_info)
            self.get_logger().info("💾 Trajectory saved")
        
        self.get_logger().info("=" * 70)
        
        # Loop mode: start new trajectory
        if self.loop:
            self.get_logger().info("\n🔄 Ready for next trajectory...")
            time.sleep(1.0)
            self._start_new_trajectory()


def main(args=None):
    """Main function."""
    rclpy.init(args=args)
    
    node = None
    try:
        node = CollectTrajectoryNode()
        # Note: RobotEnv has its own MultiThreadedExecutor that spins the node
        # in a background thread. We just need to keep the main thread alive.
        # The timer callbacks are processed by RobotEnv's executor.
        while rclpy.ok() and not node._trajectory_completed:
            time.sleep(0.1)
            # Check for loop mode - continue if loop is enabled
            if node._trajectory_completed and node.loop:
                time.sleep(0.5)  # Brief wait before checking again
    except KeyboardInterrupt:
        print("\n⚠️  Interrupted by user")
    except Exception as e:
        print(f"❌ Error: {e}\n{traceback.format_exc()}")
    finally:
        if node is not None and hasattr(node, 'env'):
            node.env.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
