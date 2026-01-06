#!/usr/bin/env python3
"""
Replay Trajectory Node - ROS2 Node for replaying saved robot trajectories.

This node reads a saved trajectory HDF5 file and replays the actions on the robot.

Usage:
    ros2 run role_ros2 replay_trajectory_node --ros-args -p filepath:=/path/to/trajectory.h5

Author: Role-ROS2 Team
"""

import time
import traceback

import numpy as np
import rclpy
from rclpy.node import Node

from role_ros2.robot_env import RobotEnv
from role_ros2.trajectory_utils.trajectory_reader import TrajectoryReader
from role_ros2.misc.ros2_utils import get_ros_time_ms


class ReplayTrajectoryNode(Node):
    """
    ROS2 Node for replaying saved robot trajectories.
    
    This node:
    1. Reads trajectory data from HDF5 file
    2. Moves robot to initial position
    3. Replays the recorded actions
    """
    
    def __init__(self):
        super().__init__('replay_trajectory_node')
        
        # Declare parameters
        self.declare_parameter('filepath', '')
        self.declare_parameter('action_space', 'cartesian_velocity')
        self.declare_parameter('control_hz', 15.0)
        self.declare_parameter('speed_factor', 1.0)  # Speed multiplier for replay
        
        # Get parameters
        self.filepath = self.get_parameter('filepath').get_parameter_value().string_value
        action_space = self.get_parameter('action_space').get_parameter_value().string_value
        self.control_hz = self.get_parameter('control_hz').get_parameter_value().double_value
        self.speed_factor = self.get_parameter('speed_factor').get_parameter_value().double_value
        
        # Validate filepath
        if not self.filepath:
            self.get_logger().error("❌ No trajectory filepath provided!")
            self.get_logger().error("   Use: --ros-args -p filepath:=/path/to/trajectory.h5")
            raise ValueError("filepath parameter is required")
        
        # State variables
        self._current_step = 0
        self._horizon = 0
        self._replay_completed = False
        self._traj_reader = None
        
        # Initialize RobotEnv
        self.get_logger().info("=" * 70)
        self.get_logger().info("🔄 Replay Trajectory Node")
        self.get_logger().info("=" * 70)
        self.get_logger().info(f"  Filepath: {self.filepath}")
        self.get_logger().info(f"  Action space: {action_space}")
        self.get_logger().info(f"  Control Hz: {self.control_hz}")
        self.get_logger().info(f"  Speed factor: {self.speed_factor}x")
        
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
        
        # Load trajectory
        self.get_logger().info("Loading trajectory...")
        try:
            self._traj_reader = TrajectoryReader(self.filepath, read_images=False)
            self._horizon = self._traj_reader.length()
            self.get_logger().info(f"✅ Trajectory loaded: {self._horizon} steps")
        except Exception as e:
            self.get_logger().error(f"❌ Failed to load trajectory: {e}")
            raise
        
        # Determine gripper key based on action space
        self._gripper_key = "gripper_velocity" if "velocity" in action_space else "gripper_position"
        
        # Move to initial position
        self._move_to_initial_position()
        
        # Create control timer
        timer_period = (1.0 / max(self.control_hz, 1e-6)) / self.speed_factor
        self._control_timer = self.create_timer(timer_period, self._replay_loop)
        
        self.get_logger().info("=" * 70)
        self.get_logger().info("▶️  Starting replay...")
        self.get_logger().info("=" * 70)
    
    def _move_to_initial_position(self):
        """Move robot to the initial position from trajectory."""
        self.get_logger().info("🤖 Moving to initial position...")
        
        try:
            # Read first timestep
            timestep = self._traj_reader.read_timestep()
            self._current_step = 1  # We've read one step
            
            # Get initial joint positions
            robot_state = timestep["observation"]["robot_state"]
            init_joint_position = robot_state["joint_positions"]
            init_gripper_position = robot_state["gripper_position"]
            
            self.get_logger().info(f"   Initial joints: [{', '.join([f'{j:.3f}' for j in init_joint_position[:3]])}...]")
            self.get_logger().info(f"   Initial gripper: {init_gripper_position:.3f}")
            
            # Move robot to initial position
            action = np.concatenate([init_joint_position, [init_gripper_position]])
            self.env.update_robot(action, action_space="joint_position", blocking=True)
            
            self.get_logger().info("✅ Robot at initial position")
            
        except Exception as e:
            self.get_logger().error(f"❌ Failed to move to initial position: {e}")
            raise
    
    def _replay_loop(self):
        """Main replay loop."""
        if self._replay_completed:
            return
        
        try:
            # Check if we've reached the end
            if self._current_step >= self._horizon:
                self._handle_replay_complete()
                return
            
            # Read timestep (note: we already read first one in _move_to_initial_position)
            if self._current_step > 1:
                timestep = self._traj_reader.read_timestep()
            else:
                # For step 1, we need to read the next timestep
                timestep = self._traj_reader.read_timestep()
            
            # Get action in desired action space
            action_dict = timestep["action"]
            
            # Get arm action
            if self.env.action_space in action_dict:
                arm_action = action_dict[self.env.action_space]
            else:
                self.get_logger().warn(f"Action space '{self.env.action_space}' not found in trajectory")
                arm_action = np.zeros(6)
            
            # Get gripper action
            if self._gripper_key in action_dict:
                gripper_action = action_dict[self._gripper_key]
            else:
                gripper_action = 0.0
            
            # Combine action
            action = np.concatenate([np.array(arm_action).flatten(), [gripper_action]])
            
            # Check if movement was enabled
            controller_info = timestep["observation"].get("controller_info", {})
            movement_enabled = controller_info.get("movement_enabled", True)
            
            # Execute action
            if movement_enabled:
                self.env.step(action)
            
            # Progress logging
            self._current_step += 1
            if self._current_step % 50 == 0 or self._current_step == self._horizon:
                progress = (self._current_step / self._horizon) * 100
                self.get_logger().info(f"Step {self._current_step}/{self._horizon} ({progress:.1f}%)")
                
        except Exception as e:
            self.get_logger().error(f"Error in replay loop: {e}\n{traceback.format_exc()}")
            self._handle_replay_complete(success=False)
    
    def _handle_replay_complete(self, success=True):
        """Handle replay completion."""
        self._replay_completed = True
        self._control_timer.cancel()
        
        if self._traj_reader is not None:
            self._traj_reader.close()
        
        self.get_logger().info("")
        self.get_logger().info("=" * 70)
        if success:
            self.get_logger().info("✅ Replay completed successfully!")
        else:
            self.get_logger().info("❌ Replay completed with errors")
        self.get_logger().info(f"   Total steps: {self._current_step}")
        self.get_logger().info("=" * 70)


def main(args=None):
    """Main function."""
    rclpy.init(args=args)
    
    node = None
    try:
        node = ReplayTrajectoryNode()
        # Note: RobotEnv has its own MultiThreadedExecutor that spins the node
        # in a background thread. We just need to keep the main thread alive.
        # The timer callbacks are processed by RobotEnv's executor.
        while rclpy.ok() and not node._replay_completed:
            time.sleep(0.1)
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
