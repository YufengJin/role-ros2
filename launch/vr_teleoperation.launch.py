#!/usr/bin/env python3
"""
Launch file for VR Teleoperation System

This launch file starts all nodes required for VR teleoperation:
1. oculus_reader_node - Reads Oculus controller data
2. vr_policy_node - Computes VR policy actions
3. teleoperation_node - Executes actions on robot

Usage:
    ros2 launch role_ros2 vr_teleoperation.launch.py
    
    # With parameters:
    ros2 launch role_ros2 vr_teleoperation.launch.py \
        right_controller:=true \
        publish_rate:=50.0 \
        action_space:=cartesian_velocity
"""

import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

# Enable colored output
os.environ["RCUTILS_COLORIZED_OUTPUT"] = "1"


def load_config_yaml(package_name, config_file):
    """
    Load launch configuration YAML file.
    
    This is a plain YAML file for launch configuration, NOT a ROS2 node parameter file.
    The format is simple key-value pairs at the root level.
    
    Args:
        package_name: ROS2 package name
        config_file: Config file name (e.g., 'oculus_reader_config.yaml')
    
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
    """Generate launch description for VR teleoperation system."""
    
    # Load configuration files
    oculus_reader_config = load_config_yaml('role_ros2', 'oculus_reader_config.yaml')
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
    
    # Oculus reader parameters
    oculus_publish_rate_arg = DeclareLaunchArgument(
        'oculus_publish_rate',
        default_value=get_default(oculus_reader_config, 'publish_rate', '50.0'),
        description='Oculus reader publish rate (Hz). Default from config/oculus_reader_config.yaml'
    )
    
    # VR policy parameters
    right_controller_arg = DeclareLaunchArgument(
        'right_controller',
        default_value=get_default(vr_policy_config, 'right_controller', 'true'),
        description='Use right controller (true) or left (false). Default from config/vr_policy_config.yaml'
    )
    
    vr_policy_publish_rate_arg = DeclareLaunchArgument(
        'vr_policy_publish_rate',
        default_value=get_default(vr_policy_config, 'publish_rate', '15.0'),
        description='VR policy action publish rate (Hz). Default from config/vr_policy_config.yaml'
    )
    
    max_lin_vel_arg = DeclareLaunchArgument(
        'max_lin_vel',
        default_value=get_default(vr_policy_config, 'max_lin_vel', '1.0'),
        description='Maximum linear velocity. Default from config/vr_policy_config.yaml'
    )
    
    max_rot_vel_arg = DeclareLaunchArgument(
        'max_rot_vel',
        default_value=get_default(vr_policy_config, 'max_rot_vel', '1.0'),
        description='Maximum rotational velocity. Default from config/vr_policy_config.yaml'
    )
    
    max_gripper_vel_arg = DeclareLaunchArgument(
        'max_gripper_vel',
        default_value=get_default(vr_policy_config, 'max_gripper_vel', '1.0'),
        description='Maximum gripper velocity. Default from config/vr_policy_config.yaml'
    )
    
    spatial_coeff_arg = DeclareLaunchArgument(
        'spatial_coeff',
        default_value=get_default(vr_policy_config, 'spatial_coeff', '1.0'),
        description='Spatial scaling coefficient. Default from config/vr_policy_config.yaml'
    )
    
    pos_action_gain_arg = DeclareLaunchArgument(
        'pos_action_gain',
        default_value=get_default(vr_policy_config, 'pos_action_gain', '5.0'),
        description='Position action gain. Default from config/vr_policy_config.yaml'
    )
    
    rot_action_gain_arg = DeclareLaunchArgument(
        'rot_action_gain',
        default_value=get_default(vr_policy_config, 'rot_action_gain', '2.0'),
        description='Rotation action gain. Default from config/vr_policy_config.yaml'
    )
    
    gripper_action_gain_arg = DeclareLaunchArgument(
        'gripper_action_gain',
        default_value=get_default(vr_policy_config, 'gripper_action_gain', '3.0'),
        description='Gripper action gain. Default from config/vr_policy_config.yaml'
    )
    
    # Teleoperation parameters
    action_space_arg = DeclareLaunchArgument(
        'action_space',
        default_value='cartesian_velocity',
        description='Action space: cartesian_velocity, joint_velocity, etc.'
    )
    
    do_reset_on_start_arg = DeclareLaunchArgument(
        'do_reset_on_start',
        default_value='true',
        description='Reset robot on startup'
    )
    
    reset_randomize_arg = DeclareLaunchArgument(
        'reset_randomize',
        default_value='false',
        description='Randomize robot pose on reset'
    )
    
    # Topic remapping (optional)
    robot_state_topic_arg = DeclareLaunchArgument(
        'robot_state_topic',
        default_value=get_default(vr_policy_config, 'robot_state_topic', 'polymetis/robot_state'),
        description='Robot state topic name. Default from config/vr_policy_config.yaml'
    )
    
    action_topic_arg = DeclareLaunchArgument(
        'action_topic',
        default_value=get_default(vr_policy_config, 'action_topic', 'vr_policy/action'),
        description='VR policy action topic name. Default from config/vr_policy_config.yaml'
    )
    
    # ========== Nodes ==========
    
    # Node 1: Oculus Reader Node
    oculus_reader_node = Node(
        package='role_ros2',
        executable='oculus_reader_node',
        name='oculus_reader_node',
        parameters=[{
            'publish_rate': LaunchConfiguration('oculus_publish_rate'),
        }],
        output='screen',
    )
    
    # Node 2: VR Policy Node
    vr_policy_node = Node(
        package='role_ros2',
        executable='vr_policy_node',
        name='vr_policy_node',
        parameters=[{
            'right_controller': LaunchConfiguration('right_controller'),
            'publish_rate': LaunchConfiguration('vr_policy_publish_rate'),
            'max_lin_vel': LaunchConfiguration('max_lin_vel'),
            'max_rot_vel': LaunchConfiguration('max_rot_vel'),
            'max_gripper_vel': LaunchConfiguration('max_gripper_vel'),
            'spatial_coeff': LaunchConfiguration('spatial_coeff'),
            'pos_action_gain': LaunchConfiguration('pos_action_gain'),
            'rot_action_gain': LaunchConfiguration('rot_action_gain'),
            'gripper_action_gain': LaunchConfiguration('gripper_action_gain'),
            'robot_state_topic': LaunchConfiguration('robot_state_topic'),
            'action_topic': LaunchConfiguration('action_topic'),
        }],
        output='screen',
    )
    
    # Node 3: Teleoperation Node
    teleoperation_node = Node(
        package='role_ros2',
        executable='teleoperation_node',
        name='teleoperation_node',
        parameters=[{
            'action_space': LaunchConfiguration('action_space'),
            'action_topic': LaunchConfiguration('action_topic'),
            'do_reset_on_start': LaunchConfiguration('do_reset_on_start'),
            'reset_randomize': LaunchConfiguration('reset_randomize'),
        }],
        output='screen',
    )
    
    # ========== Launch Description ==========
    
    return LaunchDescription([
        # Launch arguments
        oculus_publish_rate_arg,
        right_controller_arg,
        vr_policy_publish_rate_arg,
        max_lin_vel_arg,
        max_rot_vel_arg,
        max_gripper_vel_arg,
        spatial_coeff_arg,
        pos_action_gain_arg,
        rot_action_gain_arg,
        gripper_action_gain_arg,
        action_space_arg,
        do_reset_on_start_arg,
        reset_randomize_arg,
        robot_state_topic_arg,
        action_topic_arg,
        
        # Info message
        LogInfo(msg=[
            '=' * 70,
            'VR Teleoperation System Launch',
            '=' * 70,
            'Nodes:',
            '  1. oculus_reader_node - Oculus controller data reader',
            '  2. vr_policy_node - VR policy action computation',
            '  3. teleoperation_node - Robot action execution',
            '=' * 70,
        ]),
        
        # Nodes
        oculus_reader_node,
        vr_policy_node,
        teleoperation_node,
    ])

