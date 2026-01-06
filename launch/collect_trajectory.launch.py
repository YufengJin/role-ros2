#!/usr/bin/env python3
"""
Launch file for Collect Trajectory Node

This launch file starts the trajectory collection node with configurable parameters.

Usage:
    # Basic usage (teleoperation only)
    ros2 launch role_ros2 collect_trajectory.launch.py
    
    # With save folder (data collection mode)
    ros2 launch role_ros2 collect_trajectory.launch.py save_folder:=/path/to/save
    
    # With all parameters
    ros2 launch role_ros2 collect_trajectory.launch.py \
        save_folder:=/home/user/trajectories \
        save_images:=true \
        action_space:=cartesian_velocity \
        control_hz:=15.0 \
        reset_robot:=true \
        randomize_reset:=false \
        wait_for_controller:=true \
        loop:=true \
        right_controller:=true \
        horizon:=-1

Author: Role-ROS2 Team
"""

import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Enable colored output
os.environ["RCUTILS_COLORIZED_OUTPUT"] = "1"


def load_config_yaml(package_name: str, config_file: str) -> dict:
    """
    Load launch configuration YAML file.
    
    Args:
        package_name: ROS2 package name
        config_file: Config file name (e.g., 'collect_trajectory_config.yaml')
    
    Returns:
        dict: Configuration dictionary, or empty dict if file not found
    """
    try:
        package_share_dir = get_package_share_directory(package_name)
        config_path = os.path.join(package_share_dir, 'config', config_file)
        
        if os.path.exists(config_path):
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)
                return config if config else {}
    except Exception as e:
        print(f"Warning: Could not load config file {config_file}: {e}")
    
    return {}


def get_default(config_dict: dict, key: str, default_value) -> str:
    """
    Get default value from config dictionary and convert to string.
    
    Args:
        config_dict: Configuration dictionary
        key: Key to look up
        default_value: Default value if key not found
    
    Returns:
        str: Value as string for launch argument
    """
    value = config_dict.get(key, default_value)
    # Convert boolean to string
    if isinstance(value, bool):
        return 'true' if value else 'false'
    # Handle None values for optional parameters
    if value is None:
        return ''
    return str(value)


def generate_launch_description():
    """Generate launch description for collect trajectory node."""
    
    # Load configuration file
    config = load_config_yaml('role_ros2', 'collect_trajectory_config.yaml')
    
    # ========== Launch Arguments ==========
    
    # Save settings
    save_folder_arg = DeclareLaunchArgument(
        'save_folder',
        default_value=get_default(config, 'save_folder', ''),
        description='Folder to save trajectory files. Empty for teleoperation-only mode.'
    )
    
    save_images_arg = DeclareLaunchArgument(
        'save_images',
        default_value=get_default(config, 'save_images', 'false'),
        description='Whether to save images in trajectory files.'
    )
    
    # Control settings
    action_space_arg = DeclareLaunchArgument(
        'action_space',
        default_value=get_default(config, 'action_space', 'cartesian_velocity'),
        description='Action space: cartesian_velocity, cartesian_position, joint_velocity, joint_position'
    )
    
    control_hz_arg = DeclareLaunchArgument(
        'control_hz',
        default_value=get_default(config, 'control_hz', '15.0'),
        description='Control frequency in Hz.'
    )
    
    wait_for_controller_arg = DeclareLaunchArgument(
        'wait_for_controller',
        default_value=get_default(config, 'wait_for_controller', 'true'),
        description='Whether to wait for controller movement before executing actions.'
    )
    
    # Controller settings
    right_controller_arg = DeclareLaunchArgument(
        'right_controller',
        default_value=get_default(config, 'right_controller', 'true'),
        description='Use right controller (true) or left controller (false).'
    )
    
    # Robot reset settings
    reset_robot_arg = DeclareLaunchArgument(
        'reset_robot',
        default_value=get_default(config, 'reset_robot', 'true'),
        description='Whether to reset robot to home position on startup.'
    )
    
    randomize_reset_arg = DeclareLaunchArgument(
        'randomize_reset',
        default_value=get_default(config, 'randomize_reset', 'false'),
        description='Whether to add random offset to home position on reset.'
    )
    
    # Trajectory settings
    loop_arg = DeclareLaunchArgument(
        'loop',
        default_value=get_default(config, 'loop', 'false'),
        description='Whether to continuously collect trajectories in loop mode.'
    )
    
    horizon_arg = DeclareLaunchArgument(
        'horizon',
        default_value=get_default(config, 'horizon', '-1'),
        description='Maximum steps per trajectory (-1 for unlimited).'
    )
    
    # ========== Node ==========
    
    collect_trajectory_node = Node(
        package='role_ros2',
        executable='collect_trajectory_node',
        name='collect_trajectory_node',
        parameters=[{
            'save_folder': LaunchConfiguration('save_folder'),
            'save_images': LaunchConfiguration('save_images'),
            'action_space': LaunchConfiguration('action_space'),
            'control_hz': LaunchConfiguration('control_hz'),
            'wait_for_controller': LaunchConfiguration('wait_for_controller'),
            'right_controller': LaunchConfiguration('right_controller'),
            'reset_robot': LaunchConfiguration('reset_robot'),
            'randomize_reset': LaunchConfiguration('randomize_reset'),
            'loop': LaunchConfiguration('loop'),
            'horizon': LaunchConfiguration('horizon'),
        }],
        output='screen',
    )
    
    # ========== Launch Description ==========
    
    return LaunchDescription([
        # Launch arguments
        save_folder_arg,
        save_images_arg,
        action_space_arg,
        control_hz_arg,
        wait_for_controller_arg,
        right_controller_arg,
        reset_robot_arg,
        randomize_reset_arg,
        loop_arg,
        horizon_arg,
        
        # Info message
        LogInfo(msg=[
            '\n',
            '=' * 70 + '\n',
            '🎮 Collect Trajectory Node Launch\n',
            '=' * 70 + '\n',
            'Configuration:\n',
            '  • Config file: config/collect_trajectory_config.yaml\n',
            '  • Parameters can be overridden via command line\n',
            '\n',
            'Control Instructions:\n',
            '  • Hold GRIP button to enable movement\n',
            '  • Press A (right) or X (left) to mark SUCCESS\n',
            '  • Press B (right) or Y (left) to mark FAILURE\n',
            '=' * 70 + '\n',
        ]),
        
        # Node
        collect_trajectory_node,
    ])
