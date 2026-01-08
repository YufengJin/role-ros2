#!/usr/bin/env python3
"""
Replay Trajectory - Python script for replaying saved robot trajectories.

This script reads a saved trajectory HDF5 file and replays the actions on the robot.

Usage:
    python3 replay_trajectory_node.py --filepath /path/to/trajectory.h5
    
    # With options
    python3 replay_trajectory_node.py \\
        --filepath /path/to/trajectory.h5 \\
        --action_space cartesian_velocity \\
        --control-hz 15.0 \\
        --speed-factor 1.5

Author: Role-ROS2 Team
"""

import argparse
import time
import traceback

import numpy as np
import rclpy
from rclpy.node import Node

from role_ros2.robot_env import RobotEnv
from role_ros2.trajectory_utils.trajectory_reader import TrajectoryReader


class ReplayTrajectory:
    """
    Python class for replaying saved robot trajectories.
    
    This class:
    1. Reads trajectory data from HDF5 file
    2. Moves robot to initial position
    3. Replays the recorded actions
    """
    
    def __init__(self, args):
        """Initialize ReplayTrajectory with parsed arguments."""
        # Parse arguments
        self.filepath = args.filepath
        self.action_space = args.action_space
        self.control_hz = args.control_hz
        self.speed_factor = args.speed_factor
        
        # Validate filepath
        if not self.filepath:
            raise ValueError("filepath is required. Use --filepath /path/to/trajectory.h5")
        
        # State variables
        self._current_step = 0
        self._horizon = 0
        self._replay_completed = False
        self._traj_reader = None
        
        # Initialize ROS2 node (for RobotEnv)
        if not rclpy.ok():
            rclpy.init()
        self._node = Node('replay_trajectory_node')
        
        # Initialize RobotEnv
        self._print("=" * 70)
        self._print("🔄 Replay Trajectory")
        self._print("=" * 70)
        self._print(f"  Filepath: {self.filepath}")
        self._print(f"  Action space: {self.action_space}")
        self._print(f"  Control Hz: {self.control_hz}")
        self._print(f"  Speed factor: {self.speed_factor}x")
        
        try:
            self.env = RobotEnv(
                action_space=self.action_space,
                do_reset=False,
                node=self._node,
            )
            self.env.control_hz = self.control_hz
            self._print("✅ RobotEnv initialized")
        except Exception as e:
            self._print(f"❌ Failed to initialize RobotEnv: {e}")
            raise
        
        # Load trajectory
        self._print("Loading trajectory...")
        try:
            self._traj_reader = TrajectoryReader(self.filepath, read_images=False)
            self._horizon = self._traj_reader.length()
            if self._horizon is None or self._horizon == 0:
                raise ValueError(f"Invalid trajectory length: {self._horizon}")
            self._print(f"✅ Trajectory loaded: {self._horizon} steps")
            
            # Print metadata if available
            try:
                metadata = self._traj_reader.get_metadata()
                if metadata:
                    self._print(f"   Metadata: {metadata}")
            except Exception:
                pass  # Metadata reading is optional
        except Exception as e:
            self._print(f"❌ Failed to load trajectory: {e}")
            self._print(f"   File: {self.filepath}")
            self._print(f"   Traceback: {traceback.format_exc()}")
            raise
        
        # Determine gripper key based on action space
        self._gripper_key = "gripper_velocity" if "velocity" in self.action_space else "gripper_position"
        
        # Move to initial position
        self._move_to_initial_position()
        
        self._print("=" * 70)
        self._print("▶️  Starting replay...")
        self._print("=" * 70)
    
    def _print(self, msg: str):
        """Print message with timestamp."""
        print(msg)
    
    def _move_to_initial_position(self):
        """Move robot to the initial position from trajectory."""
        self._print("🤖 Moving to initial position...")
        
        try:
            # Read first timestep
            timestep = self._traj_reader.read_timestep()
            self._current_step = 1  # We've read one step
            
            # Get initial joint positions
            robot_state = timestep["observation"]["robot_state"]
            init_joint_position = robot_state["joint_positions"]
            init_gripper_position = robot_state["gripper_position"]
            
            self._print(f"   Initial joints: [{', '.join([f'{j:.3f}' for j in init_joint_position[:3]])}...]")
            self._print(f"   Initial gripper: {init_gripper_position:.3f}")
            
            # Move robot to initial position
            action = np.concatenate([init_joint_position, [init_gripper_position]])
            self.env.update_robot(action, action_space="joint_position", blocking=True)
            
            self._print("✅ Robot at initial position")
            
        except Exception as e:
            self._print(f"❌ Failed to move to initial position: {e}")
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
                self._print(f"Warning: Action space '{self.env.action_space}' not found in trajectory")
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
                self._print(f"Step {self._current_step}/{self._horizon} ({progress:.1f}%)")
                
        except Exception as e:
            self._print(f"Error in replay loop: {e}\n{traceback.format_exc()}")
            self._handle_replay_complete(success=False)
    
    def _handle_replay_complete(self, success=True):
        """Handle replay completion."""
        self._replay_completed = True
        
        if self._traj_reader is not None:
            self._traj_reader.close()
        
        self._print("")
        self._print("=" * 70)
        if success:
            self._print("✅ Replay completed successfully!")
        else:
            self._print("❌ Replay completed with errors")
        self._print(f"   Total steps: {self._current_step}")
        self._print("=" * 70)
    
    def shutdown(self):
        """Clean shutdown."""
        if self._traj_reader is not None:
            self._traj_reader.close()
        
        # Shutdown RobotEnv
        if hasattr(self, 'env'):
            self.env.shutdown()
        
        # Destroy node
        if hasattr(self, '_node'):
            self._node.destroy_node()


def parse_args():
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(
        description='Replay saved robot trajectories',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Basic usage
  python3 replay_trajectory_node.py --filepath /path/to/trajectory.h5
  
  # With options
  python3 replay_trajectory_node.py \\
      --filepath /path/to/trajectory.h5 \\
      --action_space cartesian_velocity \\
      --control-hz 15.0 \\
      --speed-factor 1.5
        """
    )
    
    parser.add_argument(
        '--filepath',
        type=str,
        required=True,
        help='Path to trajectory HDF5 file'
    )
    parser.add_argument(
        '--action_space',
        type=str,
        default='cartesian_velocity',
        choices=['joint_position', 'joint_velocity', 'cartesian_position', 'cartesian_velocity'],
        help='Action space for robot control: joint_position, joint_velocity, cartesian_position, or cartesian_velocity (default: cartesian_velocity)'
    )
    parser.add_argument(
        '--control-hz',
        type=float,
        default=15.0,
        help='Control frequency in Hz (default: 15.0)'
    )
    parser.add_argument(
        '--speed-factor',
        type=float,
        default=1.0,
        help='Speed multiplier for replay (default: 1.0, >1.0 for faster, <1.0 for slower)'
    )
    
    return parser.parse_args()


def main():
    """Main function."""
    args = parse_args()
    
    replayer = None
    
    try:
        replayer = ReplayTrajectory(args)
        
        # Main replay loop
        # Note: RobotEnv has its own MultiThreadedExecutor that spins the node
        # in a background thread. We just need to keep the main thread alive.
        timer_period = (1.0 / max(replayer.control_hz, 1e-6)) / replayer.speed_factor
        
        while rclpy.ok() and not replayer._replay_completed:
            replayer._replay_loop()
            time.sleep(timer_period)
            
    except KeyboardInterrupt:
        print("\n⚠️  Interrupted by user")
    except Exception as e:
        print(f"❌ Error: {e}\n{traceback.format_exc()}")
    finally:
        if replayer is not None:
            replayer.shutdown()
        try:
            rclpy.shutdown()
        except Exception:
            pass  # May already be shutdown


if __name__ == '__main__':
    main()
