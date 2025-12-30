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
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

# Enable colored output
os.environ["RCUTILS_COLORIZED_OUTPUT"] = "1"


def generate_launch_description():
    """Generate launch description for VR teleoperation system."""
    
    # ========== Launch Arguments ==========
    
    # Oculus reader parameters
    oculus_publish_rate_arg = DeclareLaunchArgument(
        'oculus_publish_rate',
        default_value='50.0',
        description='Oculus reader publish rate (Hz)'
    )
    
    # VR policy parameters
    right_controller_arg = DeclareLaunchArgument(
        'right_controller',
        default_value='true',
        description='Use right controller (true) or left (false)'
    )
    
    vr_policy_publish_rate_arg = DeclareLaunchArgument(
        'vr_policy_publish_rate',
        default_value='15.0',
        description='VR policy action publish rate (Hz)'
    )
    
    max_lin_vel_arg = DeclareLaunchArgument(
        'max_lin_vel',
        default_value='1.0',
        description='Maximum linear velocity'
    )
    
    max_rot_vel_arg = DeclareLaunchArgument(
        'max_rot_vel',
        default_value='1.0',
        description='Maximum rotational velocity'
    )
    
    max_gripper_vel_arg = DeclareLaunchArgument(
        'max_gripper_vel',
        default_value='1.0',
        description='Maximum gripper velocity'
    )
    
    spatial_coeff_arg = DeclareLaunchArgument(
        'spatial_coeff',
        default_value='1.0',
        description='Spatial scaling coefficient'
    )
    
    pos_action_gain_arg = DeclareLaunchArgument(
        'pos_action_gain',
        default_value='5.0',
        description='Position action gain'
    )
    
    rot_action_gain_arg = DeclareLaunchArgument(
        'rot_action_gain',
        default_value='2.0',
        description='Rotation action gain'
    )
    
    gripper_action_gain_arg = DeclareLaunchArgument(
        'gripper_action_gain',
        default_value='3.0',
        description='Gripper action gain'
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
        default_value='polymetis/robot_state',
        description='Robot state topic name'
    )
    
    action_topic_arg = DeclareLaunchArgument(
        'action_topic',
        default_value='vr_policy/action',
        description='VR policy action topic name'
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

