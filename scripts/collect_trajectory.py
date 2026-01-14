#!/usr/bin/env python3
"""
Collect Trajectory - Python script for collecting robot trajectories using VR controller.

This script provides a pure Python interface for trajectory collection.

Usage:
    # Teleoperation only (no saving)
    python3 collect_trajectory_node.py
    
    # Save trajectory with task name
    python3 collect_trajectory_node.py --task pick_and_place
    
    # Full options
    python3 collect_trajectory_node.py \
        --task pick_and_place \
        --save-folder /path/to/save \
        --save-images \
        --action-space cartesian_velocity \
        --control-hz 15.0

Control:
    • Hold GRIP button to enable movement
    • Long press A (right) or X (left) to mark SUCCESS and SAVE trajectory
    • Long press B (right) or Y (left) to mark FAILURE and DISCARD trajectory
    • Ctrl+C to exit

Author: Role-ROS2 Team
"""

import argparse
import os
import signal
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


# Long press duration threshold (seconds)
LONG_PRESS_THRESHOLD = 0.5


class CollectTrajectory:
    """
    Python class for collecting robot trajectories using VR controller.
    
    This class:
    1. Initializes RobotEnv and VRPolicy (controller)
    2. Continuously collects observations from RobotEnv
    3. Computes actions from VRPolicy
    4. Executes actions on RobotEnv
    5. Optionally saves trajectory data (if task is provided)
    
    Control Logic:
    - Always loops (no single trajectory mode)
    - Long press A/X: SUCCESS → Save trajectory, start new trajectory
    - Long press B/Y: FAILURE → Discard trajectory, reset robot, start new trajectory
    - Ctrl+C: Clean shutdown
    """
    
    def __init__(self, args):
        """Initialize CollectTrajectory with parsed arguments."""
        # Parse arguments
        self.save_folder = args.save_folder
        self.task_name = args.task
        self.action_space = args.action_space
        self.control_hz = args.control_hz
        self.reset_robot_on_start = args.reset_robot
        self.randomize_reset = args.randomize_reset
        self.wait_for_controller = args.wait_for_controller
        self.save_images = args.save_images
        self.save_depths = args.save_depths
        self.right_controller = args.right_controller
        self.horizon = None if args.horizon <= 0 else args.horizon
        
        # Validate: save_depths requires save_images
        if self.save_depths and not self.save_images:
            raise ValueError("--save-depths requires --save-images. Cannot save depth without images.")
        
        # Build save_folder: if task is provided, automatically enable saving
        if self.task_name:
            self.save_folder = os.path.join(self.save_folder, self.task_name)
            self.save_trajectory = True  # Auto-enable saving when task is provided
        else:
            self.save_folder = ''
            self.task_name = None
            self.save_trajectory = False  # No saving if no task
        
        # State variables
        self._num_steps = 0
        self._traj_count = 0
        self._shutdown_requested = False
        self._traj_writer: Optional[TrajectoryWriter] = None
        self._current_traj_filepath: Optional[str] = None
        self._recording_started = False  # Track if recording has started (after A button press)
        
        # Long press detection state
        self._success_button_press_start: Optional[float] = None
        self._failure_button_press_start: Optional[float] = None
        
        # Initialize ROS2 node (for RobotEnv)
        if not rclpy.ok():
            rclpy.init()
        self._node = Node('collect_trajectory_node')
        
        # Initialize RobotEnv
        self._print("=" * 70)
        self._print("🚀 Collect Trajectory - Starting Initialization")
        self._print("=" * 70)
        self._print("📝 Parameters:")
        self._print(f"   • Action space: {self.action_space}")
        self._print(f"   • Control Hz: {self.control_hz}")
        self._print(f"   • Wait for controller: {self.wait_for_controller}")
        self._print(f"   • Right controller: {self.right_controller}")
        self._print(f"   • Horizon: {self.horizon if self.horizon else 'unlimited'}")
        self._print("-" * 70)
        
        self._print("🔧 [1/4] Initializing RobotEnv...")
        try:
            self.env = RobotEnv(
                action_space=self.action_space,
                do_reset=False,
                node=self._node,
            )
            self.env.control_hz = self.control_hz
            self._print("   ✅ RobotEnv initialized successfully")
        except Exception as e:
            self._print(f"   ❌ Failed to initialize RobotEnv: {e}")
            raise
        
        # Initialize VRPolicy (controller)
        self._print("🎮 [2/4] Initializing VRPolicy controller...")
        try:
            self.controller = VRPolicy(right_controller=self.right_controller)
            controller_side = "RIGHT" if self.right_controller else "LEFT"
            self._print(f"   ✅ VRPolicy initialized ({controller_side} controller)")
        except Exception as e:
            self._print(f"   ❌ Failed to initialize VRPolicy: {e}")
            raise
        
        # Reset robot if requested
        if self.reset_robot_on_start:
            self._print("🤖 [3/4] Resetting robot to home position...")
            try:
                self.env.reset(randomize=self.randomize_reset)
                self._print("   ✅ Robot reset completed")
            except Exception as e:
                self._print(f"   ⚠️ Robot reset failed: {e}")
        else:
            self._print("🤖 [3/4] Skipping robot reset (reset_robot=false)")
        
        # Mode info
        self._print("📁 [4/4] Setting up trajectory recording...")
        if self.save_trajectory and self.save_folder:
            os.makedirs(self.save_folder, exist_ok=True)
            self._print(f"   ✅ Mode: Recording trajectory")
            self._print(f"   📂 Save folder: {self.save_folder}")
            self._print(f"   📋 Task name: {self.task_name}")
            self._print(f"   🖼️ Save images: {self.save_images}")
            self._print(f"   📏 Save depths: {self.save_depths}")
        else:
            self._print("   ✅ Mode: Teleoperation only (no saving)")
            self._print("   ℹ️  No task specified (provide --task to enable saving)")
        
        # Start new trajectory
        self._start_new_trajectory()
        
        self._print("-" * 70)
        self._print("✅ Initialization Complete!")
        self._print("=" * 70)
        self._print("")
        self._print("📋 CONTROL INSTRUCTIONS:")
        self._print("   ┌─────────────────────────────────────────────────────────┐")
        self._print("   │  🎮 Press A/X to start recording                        │")
        self._print("   │  🎮 Hold GRIP button      → Enable robot movement       │")
        self._print(f"   │  ✅ Long press A/X ({LONG_PRESS_THRESHOLD}s)  → SUCCESS: Save & Reset        │")
        self._print(f"   │  ❌ Long press B/Y ({LONG_PRESS_THRESHOLD}s)  → FAILURE: Discard & Reset     │")
        self._print("   │  🛑 Ctrl+C               → Exit program                │")
        self._print("   └─────────────────────────────────────────────────────────┘")
        self._print("")
        self._print("=" * 70)
        self._print("🎯 Ready! Press A (right) or X (left) to start recording...")
    
    def _print(self, msg: str):
        """Print message with timestamp."""
        print(msg)
    
    def _start_new_trajectory(self):
        """Start a new trajectory recording."""
        self._traj_count += 1
        self._num_steps = 0
        self._recording_started = False  # Reset recording state
        
        # Reset long press detection
        self._success_button_press_start = None
        self._failure_button_press_start = None
        
        # Reset controller state
        self.controller.reset_state()
        
        # Create trajectory writer if task is provided (auto-enable saving)
        if self.save_trajectory and self.save_folder:
            # Ensure save folder exists
            os.makedirs(self.save_folder, exist_ok=True)
            
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            self._current_traj_filepath = os.path.join(
                self.save_folder, f"trajectory_{timestamp}.h5"
            )
            
            # Build metadata
            metadata = {"trajectory_id": self._traj_count}
            if self.task_name:
                metadata["task_name"] = self.task_name
            
            self._traj_writer = TrajectoryWriter(
                self._current_traj_filepath, 
                metadata=metadata,
                save_images=self.save_images,
                save_depths=self.save_depths
            )
            self._print(f"📁 Recording trajectory #{self._traj_count}...")
        else:
            self._traj_writer = None
            self._current_traj_filepath = None
            self._print(f"🎮 Trajectory #{self._traj_count} (teleoperation only)")
    
    def _check_long_press(self, controller_info: dict) -> tuple:
        """
        Check for long press of SUCCESS (A/X) or FAILURE (B/Y) buttons.
        
        Returns:
            tuple: (is_success_long_press, is_failure_long_press)
        """
        current_time = time.time()
        success_pressed = controller_info.get("success", False)
        failure_pressed = controller_info.get("failure", False)
        
        # Check SUCCESS button (A/X)
        if success_pressed:
            if self._success_button_press_start is None:
                self._success_button_press_start = current_time
            press_duration = current_time - self._success_button_press_start
            is_success_long_press = press_duration >= LONG_PRESS_THRESHOLD
        else:
            self._success_button_press_start = None
            is_success_long_press = False
        
        # Check FAILURE button (B/Y)
        if failure_pressed:
            if self._failure_button_press_start is None:
                self._failure_button_press_start = current_time
            press_duration = current_time - self._failure_button_press_start
            is_failure_long_press = press_duration >= LONG_PRESS_THRESHOLD
        else:
            self._failure_button_press_start = None
            is_failure_long_press = False
        
        return is_success_long_press, is_failure_long_press
    
    def _control_loop(self):
        """Main control loop."""
        if self._shutdown_requested:
            return
        
        try:
            # Get controller info
            controller_info = self.controller.get_info()
            skip_action = self.wait_for_controller and (not controller_info["movement_enabled"])
            
            # Check if recording has started (wait for A button press)
            if not self._recording_started:
                success_pressed = controller_info.get("success", False)
                if success_pressed:
                    self._recording_started = True
                    # Reset long press detection to avoid immediate trigger
                    self._success_button_press_start = None
                    self._failure_button_press_start = None
                    self._print("")
                    self._print("=" * 70)
                    self._print("✅ Recording started! Long press A/X to mark SUCCESS, B/Y to mark FAILURE")
                    self._print("=" * 70)
                    self._print("")
                else:
                    # Not started yet, just wait
                    time.sleep(0.05)
                    return
            
            # Check for long press termination (only after recording started)
            is_success, is_failure = self._check_long_press(controller_info)
            
            # Handle trajectory end
            if is_success:
                self._handle_trajectory_success()
                return
            elif is_failure:
                self._handle_trajectory_failure()
                return
            
            # Use ROS time for timestamps (nanoseconds)
            control_timestamps = {"step_start": get_ros_time_ns(self._node)}
            
            # Get Observation
            obs = self.env.get_observation(use_sync=False)
            obs["controller_info"] = controller_info
            if "timestamp" not in obs:
                obs["timestamp"] = {}
            obs["timestamp"]["skip_action"] = skip_action
            
            # Get Action
            control_timestamps["policy_start"] = get_ros_time_ns(self._node)
            action, controller_action_info = self.controller.forward(obs, include_info=True)
            control_timestamps["policy_end"] = get_ros_time_ns(self._node)
            
            # Regularize Control Frequency
            control_timestamps["sleep_start"] = get_ros_time_ns(self._node)
            comp_time_ns = get_ros_time_ns(self._node) - control_timestamps["step_start"]
            comp_time_s = comp_time_ns / 1e9
            sleep_left = (1 / self.env.control_hz) - comp_time_s
            if sleep_left > 0:
                time.sleep(sleep_left)
            
            # Step Environment
            control_timestamps["control_start"] = get_ros_time_ns(self._node)
            if skip_action:
                action_info = self.env.create_action_dict(np.zeros_like(action))
            else:
                action_info = self.env.step(action)
            action_info.update(controller_action_info)
            
            # Save Data
            control_timestamps["step_end"] = get_ros_time_ns(self._node)
            obs["timestamp"]["control"] = control_timestamps
            timestep = {"observation": obs, "action": action_info}
            if self._traj_writer is not None:
                self._traj_writer.write_timestep(timestep)
            
            # Check horizon termination
            self._num_steps += 1
            if self.horizon is not None and self._num_steps >= self.horizon:
                self._print(f"⏱️ Reached horizon ({self.horizon} steps)")
                self._handle_trajectory_success()
                return
            
            # Progress logging
            if self._num_steps % 50 == 0 and self._num_steps > 0:
                movement_status = "🟢 MOVING" if controller_info["movement_enabled"] else "🔴 STOPPED"
                self._print(f"Step {self._num_steps}: {movement_status}")
                
        except Exception as e:
            self._print(f"Error in control loop: {e}\n{traceback.format_exc()}")
    
    def _handle_trajectory_success(self):
        """Handle successful trajectory completion - SAVE, reset robot, and start new."""
        self._print("")
        self._print("=" * 70)
        self._print(f"✅ Trajectory #{self._traj_count} SUCCESS")
        self._print(f"   Total steps: {self._num_steps}")
        
        # Close and save trajectory
        if self._traj_writer is not None:
            self._traj_writer.close(metadata={"success": True, "failure": False})
            self._print(f"💾 Saved: {self._current_traj_filepath}")
        
        self._print("=" * 70)
        
        # Reset robot after success
        self._print("🤖 Resetting robot to home position...")
        try:
            self.env.reset(randomize=self.randomize_reset)
            self._print("✅ Robot reset completed")
        except Exception as e:
            self._print(f"⚠️ Robot reset failed: {e}")
        
        # Wait a moment then start new trajectory
        time.sleep(0.5)
        self._print("🔄 Starting new trajectory...")
        self._start_new_trajectory()
        self._print("⏸️  Press A (right) or X (left) to start recording...")
    
    def _handle_trajectory_failure(self):
        """Handle failed trajectory - DISCARD and reset robot."""
        self._print("")
        self._print("=" * 70)
        self._print(f"❌ Trajectory #{self._traj_count} FAILURE")
        self._print(f"   Total steps: {self._num_steps}")
        
        # Discard trajectory (close without saving, delete file)
        if self._traj_writer is not None:
            # Close the writer first
            try:
                self._traj_writer.close(metadata={"success": False, "failure": True})
            except Exception:
                pass
            
            # Delete the file
            if self._current_traj_filepath and os.path.exists(self._current_traj_filepath):
                try:
                    os.remove(self._current_traj_filepath)
                    self._print(f"🗑️ Discarded: {self._current_traj_filepath}")
                except Exception as e:
                    self._print(f"Failed to delete trajectory file: {e}")
        
        self._print("=" * 70)
        
        # Reset robot
        self._print("🤖 Resetting robot...")
        try:
            self.env.reset(randomize=self.randomize_reset)
            self._print("✅ Robot reset completed")
        except Exception as e:
            self._print(f"⚠️ Robot reset failed: {e}")
        
        # Wait a moment then start new trajectory
        time.sleep(0.5)
        self._print("🔄 Starting new trajectory...")
        self._start_new_trajectory()
        self._print("⏸️  Press A (right) or X (left) to start recording...")
    
    def shutdown(self):
        """Clean shutdown."""
        self._shutdown_requested = True
        
        # Close trajectory writer if open
        if self._traj_writer is not None:
            try:
                self._traj_writer.close(metadata={"interrupted": True})
                self._print(f"💾 Saved interrupted trajectory: {self._current_traj_filepath}")
            except Exception as e:
                self._print(f"Error closing trajectory writer: {e}")
        
        # Shutdown RobotEnv
        if hasattr(self, 'env'):
            self.env.shutdown()
        
        # Destroy node
        if hasattr(self, '_node'):
            self._node.destroy_node()


def parse_args():
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(
        description='Collect robot trajectories using VR controller',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Teleoperation only (no saving)
  python3 collect_trajectory_node.py
  
  # Save trajectory with task name
  python3 collect_trajectory_node.py --task pick_and_place
  
  # Full options
  python3 collect_trajectory_node.py \\
      --task pick_and_place \\
      --save-folder /path/to/save \\
      --save-images \\
      --action-space cartesian_velocity \\
      --control-hz 15.0
        """
    )
    
    # Save settings
    parser.add_argument(
        '--save-folder',
        type=str,
        default='/app/ros2_ws/src/role-ros2/data',
        help='Base folder to save trajectory files (default: /app/ros2_ws/src/role-ros2/data)'
    )
    parser.add_argument(
        '--task',
        type=str,
        default='',
        help='Task name. If provided, automatically enables saving to save_folder/task_name/'
    )
    parser.add_argument(
        '--save-images',
        action='store_true',
        help='Save RGB images in trajectory files (MP4 video)'
    )
    parser.add_argument(
        '--save-depths',
        action='store_true',
        help='Save depth images in trajectory files (PNG-in-HDF5, lossless). Requires --save-images.'
    )
    
    # Control settings
    parser.add_argument(
        '--action-space',
        type=str,
        default='cartesian_velocity',
        choices=['cartesian_velocity', 'cartesian_position', 'joint_velocity', 'joint_position'],
        help='Action space for robot control (default: cartesian_velocity)'
    )
    parser.add_argument(
        '--control-hz',
        type=float,
        default=15.0,
        help='Control frequency in Hz (default: 15.0)'
    )
    parser.add_argument(
        '--wait-for-controller',
        action='store_true',
        default=True,
        help='Wait for controller movement before executing actions (default: True)'
    )
    parser.add_argument(
        '--no-wait-for-controller',
        dest='wait_for_controller',
        action='store_false',
        help='Disable waiting for controller movement'
    )
    
    # Controller settings
    parser.add_argument(
        '--right-controller',
        action='store_true',
        default=True,
        help='Use right controller (default: True)'
    )
    parser.add_argument(
        '--left-controller',
        dest='right_controller',
        action='store_false',
        help='Use left controller'
    )
    
    # Robot reset settings
    parser.add_argument(
        '--reset-robot',
        action='store_true',
        default=True,
        help='Reset robot to home position on startup (default: True)'
    )
    parser.add_argument(
        '--no-reset-robot',
        dest='reset_robot',
        action='store_false',
        help='Skip robot reset on startup'
    )
    parser.add_argument(
        '--randomize-reset',
        action='store_true',
        help='Add random offset to home position on reset'
    )
    
    # Trajectory settings
    parser.add_argument(
        '--horizon',
        type=int,
        default=-1,
        help='Maximum steps per trajectory (-1 for unlimited, default: -1)'
    )
    
    return parser.parse_args()


def main():
    """Main function."""
    args = parse_args()
    
    collector = None
    
    # Setup signal handler for clean shutdown
    def signal_handler(signum, frame):
        nonlocal collector
        print("\n⚠️ Shutdown signal received (Ctrl+C)")
        if collector is not None:
            collector.shutdown()
        raise KeyboardInterrupt()
    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    try:
        collector = CollectTrajectory(args)
        
        # Main control loop
        # Note: RobotEnv has its own MultiThreadedExecutor that spins the node
        # in a background thread. We just need to keep the main thread alive.
        # Note: Frequency control is handled inside _control_loop(), so no external sleep needed
        
        while rclpy.ok() and not collector._shutdown_requested:
            collector._control_loop()
            
    except KeyboardInterrupt:
        print("👋 Exiting...")
    except Exception as e:
        print(f"❌ Error: {e}\n{traceback.format_exc()}")
    finally:
        if collector is not None:
            collector.shutdown()
        try:
            rclpy.shutdown()
        except Exception:
            pass  # May already be shutdown


if __name__ == '__main__':
    main()
