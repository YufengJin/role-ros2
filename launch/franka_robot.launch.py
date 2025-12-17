# Copyright (c) 2024
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    Shutdown
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import xacro


def load_config_yaml(package_name, config_file):
    """
    Load launch configuration YAML file.
    
    This is a plain YAML file for launch configuration, NOT a ROS2 node parameter file.
    The format is simple key-value pairs at the root level.
    
    Args:
        package_name: ROS2 package name
        config_file: Config file name (e.g., 'franka_robot_config.yaml')
    
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


def robot_description_dependent_nodes_spawner(
        context: LaunchContext,
        robot_ip,
        arm_id,
        use_fake_hardware,
        fake_sensor_commands,
        load_gripper,
        controller_name):

    robot_ip_str = context.perform_substitution(robot_ip)
    arm_id_str = context.perform_substitution(arm_id)
    use_fake_hardware_str = context.perform_substitution(use_fake_hardware)
    fake_sensor_commands_str = context.perform_substitution(
        fake_sensor_commands)
    load_gripper_str = context.perform_substitution(load_gripper)
    controller_name_str = context.perform_substitution(controller_name)

    franka_xacro_filepath = os.path.join(get_package_share_directory(
        'franka_description'), 'robots', arm_id_str, arm_id_str+'.urdf.xacro')
    robot_description = xacro.process_file(franka_xacro_filepath,
                                           mappings={
                                               'ros2_control': 'true',
                                               'arm_id': arm_id_str,
                                               'robot_ip': robot_ip_str,
                                               'hand': load_gripper_str,
                                               'use_fake_hardware': use_fake_hardware_str,
                                               'fake_sensor_commands': fake_sensor_commands_str,
                                           }).toprettyxml(indent='  ')

    # Use fr3_ros_controllers.yaml for MoveIt config, or controllers.yaml for basic control
    franka_controllers = PathJoinSubstitution(
        [FindPackageShare('franka_fr3_moveit_config'), 'config', 'fr3_ros_controllers.yaml'])

    return [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
            # 确保 robot_state_publisher 订阅 /joint_states 话题
            # 默认就是 /joint_states，但显式指定以确保正确
            remappings=[('joint_states', '/joint_states')],
        ),
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[franka_controllers,
                        {'robot_description': robot_description},
                        {'arm_id': arm_id},
                        ],
            remappings=[('joint_states', 'franka/joint_states')],
            output={
                'stdout': 'screen',
                'stderr': 'screen',
            },
            on_exit=Shutdown(),
        )]


def generate_launch_description():
    # Load default values from YAML config file
    config = load_config_yaml('role_ros2', 'franka_robot_config.yaml')
    
    # Helper function to convert config values to strings for launch arguments
    def get_default(key, default_value):
        value = config.get(key, default_value)
        # Convert boolean to string
        if isinstance(value, bool):
            return 'true' if value else 'false'
        return str(value)
    
    arm_id_parameter_name = 'arm_id'
    robot_ip_parameter_name = 'robot_ip'
    load_gripper_parameter_name = 'load_gripper'
    use_fake_hardware_parameter_name = 'use_fake_hardware'
    fake_sensor_commands_parameter_name = 'fake_sensor_commands'
    controller_name_parameter_name = 'controller_name'

    arm_id = LaunchConfiguration(arm_id_parameter_name)
    robot_ip = LaunchConfiguration(robot_ip_parameter_name)
    load_gripper = LaunchConfiguration(load_gripper_parameter_name)
    use_fake_hardware = LaunchConfiguration(use_fake_hardware_parameter_name)
    fake_sensor_commands = LaunchConfiguration(
        fake_sensor_commands_parameter_name)
    controller_name = LaunchConfiguration(controller_name_parameter_name)

    robot_description_dependent_nodes_spawner_opaque_function = OpaqueFunction(
        function=robot_description_dependent_nodes_spawner,
        args=[
            robot_ip,
            arm_id,
            use_fake_hardware,
            fake_sensor_commands,
            load_gripper,
            controller_name])

    launch_description = LaunchDescription([
        DeclareLaunchArgument(
            robot_ip_parameter_name,
            default_value=get_default('robot_ip', '172.16.0.2'),
            description='Hostname or IP address of the robot. Default from config/franka_robot_config.yaml'),
        DeclareLaunchArgument(
            arm_id_parameter_name,
            default_value=get_default('arm_id', 'fr3'),
            description='ID of the type of arm used. Supported values: fer, fr3, fp3. Default from config/franka_robot_config.yaml'),
        DeclareLaunchArgument(
            use_fake_hardware_parameter_name,
            default_value=get_default('use_fake_hardware', 'false'),
            description='Use fake hardware. Default from config/franka_robot_config.yaml'),
        DeclareLaunchArgument(
            fake_sensor_commands_parameter_name,
            default_value=get_default('fake_sensor_commands', 'false'),
            description='Fake sensor commands. Only valid when "{}" is true. Default from config/franka_robot_config.yaml'.format(
                use_fake_hardware_parameter_name)),
        DeclareLaunchArgument(
            load_gripper_parameter_name,
            default_value=get_default('load_gripper', 'true'),
            description='Use Franka Gripper as an end-effector, otherwise, the robot is loaded '
                        'without an end-effector. Default from config/franka_robot_config.yaml'),
        DeclareLaunchArgument(
            controller_name_parameter_name,
            default_value=get_default('controller_name', 'fr3_arm_controller'),
            description='Name of the arm controller to use. Default from config/franka_robot_config.yaml'),
        # Note: joint_state_publisher is optional
        # The joint_state_broadcaster controller already publishes joint states to /joint_states
        # This node can merge multiple sources, but is not essential for basic operation
        # Using hardcoded 'fr3' since arm_id is LaunchConfiguration and cannot be used in f-string
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[
                {'source_list': ['franka/joint_states', 'fr3_gripper/joint_states'],
                 'rate': 30}],
        ),
        robot_description_dependent_nodes_spawner_opaque_function,
        # Spawn joint state broadcaster
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
            output='screen',
        ),
        # Spawn franka robot state broadcaster
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['franka_robot_state_broadcaster'],
            parameters=[{'arm_id': arm_id}],
            output='screen',
            condition=UnlessCondition(use_fake_hardware),
        ),
        # Spawn arm controller
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[controller_name],
            output='screen',
        ),
        # Include gripper launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([PathJoinSubstitution(
                [FindPackageShare('franka_gripper'), 'launch', 'gripper.launch.py'])]),
            launch_arguments={robot_ip_parameter_name: robot_ip,
                              use_fake_hardware_parameter_name: use_fake_hardware,
                              arm_id_parameter_name: arm_id}.items(),
            condition=IfCondition(load_gripper)
        ),
    ])

    return launch_description

