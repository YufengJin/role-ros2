"""
Trajectory utilities for role_ros2.

This module provides functions for replaying and loading trajectories.
Adapted from droid.trajectory_utils.misc.

Note: For trajectory collection, use the collect_trajectory_node.py script directly.
"""

import time

import numpy as np

from role_ros2.trajectory_utils.trajectory_reader import TrajectoryReader


def replay_trajectory(
    env, 
    filepath=None, 
    assert_replayable_keys=["cartesian_position", "gripper_position", "joint_positions"]
):
    """
    Replay a saved trajectory.
    
    Args:
        env: RobotEnv instance
        filepath: Path to trajectory HDF5 file
        assert_replayable_keys: Keys to check for replayability (not currently enforced)
    """
    print("WARNING: STATE 'CLOSENESS' FOR REPLAYABILITY HAS NOT BEEN CALIBRATED")
    gripper_key = "gripper_velocity" if "velocity" in env.action_space else "gripper_position"
    
    # Prepare Trajectory Reader
    traj_reader = TrajectoryReader(filepath, read_images=False)
    horizon = traj_reader.length()
    
    for i in range(horizon):
        # Get HDF5 Data
        timestep = traj_reader.read_timestep()
        
        # Move To Initial Position
        if i == 0:
            init_joint_position = timestep["observation"]["robot_state"]["joint_positions"]
            init_gripper_position = timestep["observation"]["robot_state"]["gripper_position"]
            action = np.concatenate([init_joint_position, [init_gripper_position]])
            env.update_robot(action, action_space="joint_position", blocking=True)
        
        # Regularize Control Frequency
        time.sleep(1 / env.control_hz)
        
        # Get Action In Desired Action Space
        arm_action = timestep["action"][env.action_space]
        gripper_action = timestep["action"][gripper_key]
        action = np.concatenate([arm_action, [gripper_action]])
        controller_info = timestep["observation"]["controller_info"]
        movement_enabled = controller_info.get("movement_enabled", True)
        
        # Follow Trajectory
        if movement_enabled:
            env.step(action)
    
    traj_reader.close()


def load_trajectory(
    filepath=None,
    read_cameras=True,
    remove_skipped_steps=False,
    num_samples_per_traj=None,
    num_samples_per_traj_coeff=1.5,
):
    """
    Load a trajectory from file.
    
    Args:
        filepath: Path to trajectory HDF5 file
        read_cameras: If True, read camera/image data
        remove_skipped_steps: If True, remove timesteps where movement was disabled
        num_samples_per_traj: If provided, randomly sample this many timesteps
        num_samples_per_traj_coeff: Coefficient for oversampling when removing skipped steps
    
    Returns:
        List of timestep dictionaries
    """
    traj_reader = TrajectoryReader(filepath, read_images=read_cameras)
    horizon = traj_reader.length()
    timestep_list = []
    
    # Choose Timesteps To Save
    if num_samples_per_traj:
        num_to_save = num_samples_per_traj
        if remove_skipped_steps:
            num_to_save = int(num_to_save * num_samples_per_traj_coeff)
        max_size = min(num_to_save, horizon)
        indices_to_save = np.sort(np.random.choice(horizon, size=max_size, replace=False))
    else:
        indices_to_save = np.arange(horizon)
    
    # Iterate Over Trajectory
    for i in indices_to_save:
        # Get HDF5 Data
        timestep = traj_reader.read_timestep(index=i)
        
        # Filter Steps
        step_skipped = not timestep["observation"]["controller_info"].get("movement_enabled", True)
        delete_skipped_step = step_skipped and remove_skipped_steps
        
        # Save Filtered Timesteps
        if delete_skipped_step:
            del timestep
        else:
            timestep_list.append(timestep)
    
    # Remove Extra Transitions
    timestep_list = np.array(timestep_list)
    if (num_samples_per_traj is not None) and (len(timestep_list) > num_samples_per_traj):
        ind_to_keep = np.random.choice(len(timestep_list), size=num_samples_per_traj, replace=False)
        timestep_list = timestep_list[ind_to_keep]
    
    # Close Reader
    traj_reader.close()
    
    return timestep_list
