#!/usr/bin/env python3
"""
Launch file for Collect Trajectory Node

This launch file starts the collect_trajectory_node for collecting robot trajectories
using VR controller.

Usage:
    # Basic usage (default parameters)
    ros2 launch role_ros2 collect_trajectory.launch.py
    
    # With custom parameters
    ros2 launch role_ros2 collect_trajectory.launch.py \
        action_space:=cartesian_velocity \
        reset_robot_on_start:=true \
        wait_for_controller:=true \
        control_hz:=15.0

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


def load_config_yaml(package_name, config_file):
    """
    Load launch configuration YAML file.
    
    Args:
        package_name: ROS2 package name
        config_file: Config file name (e.g., 'vr_policy_config.yaml')
    
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


def generate_launch_description():
    """Generate launch description for collect trajectory node."""
    
    # Load configuration files
    vr_policy_config = load_config_yaml('role_ros2', 'vr_policy_config.yaml')
    
    # Helper function to convert config values to strings for launch arguments
    def get_default(config_dict, key, default_value):
        value = config_dict.get(key, default_value)
        # Convert boolean to string
        if isinstance(value, bool):
            return 'true' if value else 'false'
        # Handle None values for optional parameters
        if value is None:
            return ''
        return str(value)
    
    # ========== Launch Arguments ==========
    
    # Action space parameters
    action_space_arg = DeclareLaunchArgument(
        'action_space',
        default_value='cartesian_velocity',
        description='Action space: cartesian_velocity, joint_velocity, etc.'
    )
    
    gripper_action_space_arg = DeclareLaunchArgument(
        'gripper_action_space',
        default_value='',
        description='Gripper action space: velocity or position (empty = auto)'
    )
    
    control_hz_arg = DeclareLaunchArgument(
        'control_hz',
        default_value='15.0',
        description='Control frequency in Hz'
    )
    
    # Reset parameters
    reset_robot_on_start_arg = DeclareLaunchArgument(
        'reset_robot_on_start',
        default_value='true',
        description='Reset robot to home position on startup'
    )
    
    randomize_reset_arg = DeclareLaunchArgument(
        'randomize_reset',
        default_value='false',
        description='Add random cartesian noise to reset position'
    )
    
    # Controller parameters
    wait_for_controller_arg = DeclareLaunchArgument(
        'wait_for_controller',
        default_value='true',
        description='Wait for controller grip button to enable movement'
    )
    
    # VR Policy parameters
    right_controller_arg = DeclareLaunchArgument(
        'right_controller',
        default_value=get_default(vr_policy_config, 'right_controller', 'true'),
        description='Use right controller (true) or left (false)'
    )
    
    max_lin_vel_arg = DeclareLaunchArgument(
        'max_lin_vel',
        default_value=get_default(vr_policy_config, 'max_lin_vel', '1.0'),
        description='Maximum linear velocity'
    )
    
    max_rot_vel_arg = DeclareLaunchArgument(
        'max_rot_vel',
        default_value=get_default(vr_policy_config, 'max_rot_vel', '1.0'),
        description='Maximum rotational velocity'
    )
    
    max_gripper_vel_arg = DeclareLaunchArgument(
        'max_gripper_vel',
        default_value=get_default(vr_policy_config, 'max_gripper_vel', '1.0'),
        description='Maximum gripper velocity'
    )
    
    spatial_coeff_arg = DeclareLaunchArgument(
        'spatial_coeff',
        default_value=get_default(vr_policy_config, 'spatial_coeff', '1.0'),
        description='Spatial scaling coefficient'
    )
    
    pos_action_gain_arg = DeclareLaunchArgument(
        'pos_action_gain',
        default_value=get_default(vr_policy_config, 'pos_action_gain', '5.0'),
        description='Position action gain'
    )
    
    rot_action_gain_arg = DeclareLaunchArgument(
        'rot_action_gain',
        default_value=get_default(vr_policy_config, 'rot_action_gain', '2.0'),
        description='Rotation action gain'
    )
    
    gripper_action_gain_arg = DeclareLaunchArgument(
        'gripper_action_gain',
        default_value=get_default(vr_policy_config, 'gripper_action_gain', '3.0'),
        description='Gripper action gain'
    )
    
    # Debug parameters
    debug_log_frequency_arg = DeclareLaunchArgument(
        'debug_log_frequency',
        default_value='1.0',
        description='Debug log frequency in Hz (how often to print debug info)'
    )
    
    # ========== Nodes ==========
    
    # Collect Trajectory Node
    collect_trajectory_node = Node(
        package='role_ros2',
        executable='collect_trajectory_node',
        name='collect_trajectory_node',
        parameters=[{
            'action_space': LaunchConfiguration('action_space'),
            'gripper_action_space': LaunchConfiguration('gripper_action_space'),
            'control_hz': LaunchConfiguration('control_hz'),
            'reset_robot_on_start': LaunchConfiguration('reset_robot_on_start'),
            'randomize_reset': LaunchConfiguration('randomize_reset'),
            'wait_for_controller': LaunchConfiguration('wait_for_controller'),
            'right_controller': LaunchConfiguration('right_controller'),
            'max_lin_vel': LaunchConfiguration('max_lin_vel'),
            'max_rot_vel': LaunchConfiguration('max_rot_vel'),
            'max_gripper_vel': LaunchConfiguration('max_gripper_vel'),
            'spatial_coeff': LaunchConfiguration('spatial_coeff'),
            'pos_action_gain': LaunchConfiguration('pos_action_gain'),
            'rot_action_gain': LaunchConfiguration('rot_action_gain'),
            'gripper_action_gain': LaunchConfiguration('gripper_action_gain'),
            'debug_log_frequency': LaunchConfiguration('debug_log_frequency'),
        }],
        output='screen',
    )
    
    # ========== Launch Description ==========
    
    return LaunchDescription([
        # Launch arguments
        action_space_arg,
        gripper_action_space_arg,
        control_hz_arg,
        reset_robot_on_start_arg,
        randomize_reset_arg,
        wait_for_controller_arg,
        right_controller_arg,
        max_lin_vel_arg,
        max_rot_vel_arg,
        max_gripper_vel_arg,
        spatial_coeff_arg,
        pos_action_gain_arg,
        rot_action_gain_arg,
        gripper_action_gain_arg,
        debug_log_frequency_arg,
        
        # Info message
        LogInfo(msg=[
            '=' * 70,
            'Collect Trajectory Node Launch',
            '=' * 70,
            'Node: collect_trajectory_node - Collect robot trajectories using VR controller',
            '',
            'Control Instructions:',
            '  • Hold GRIP button to enable movement',
            '  • Press A (right) or X (left) to mark success',
            '  • Press B (right) or Y (left) to mark failure',
            '',
            'Reset Robot:',
            '  • Use service: ros2 service call /polymetis/reset role_ros2/srv/Reset "{randomize: false}"',
            '  • Or set reset_robot_on_start:=true (default)',
            '=' * 70,
        ]),
        
        # Node
        collect_trajectory_node,
    ])

