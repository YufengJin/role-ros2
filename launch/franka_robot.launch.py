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

    # Load URDF from role_ros2 package (converted from MuJoCo XML)
    package_share_dir = get_package_share_directory('role_ros2')
    # Get source directory
    src_dir = os.path.join(package_share_dir, '..', '..', 'src', 'role_ros2')
    urdf_filepath = os.path.join(src_dir, 'role_ros2', 'robot_ik', 'franka', f'{arm_id_str}.urdf')
    
    # If URDF doesn't exist, try to generate it
    if not os.path.exists(urdf_filepath):
        # Try to generate from XML
        xml_filepath = os.path.join(src_dir, 'role_ros2', 'robot_ik', 'franka', f'{arm_id_str}.xml')
        if os.path.exists(xml_filepath):
            import subprocess
            convert_script = os.path.join(src_dir, 'scripts', 'mujoco_to_urdf.py')
            if os.path.exists(convert_script):
                result = subprocess.run(['python3', convert_script, xml_filepath, urdf_filepath, arm_id_str],
                                      capture_output=True, text=True)
                if result.returncode != 0:
                    print(f"Warning: Failed to generate URDF: {result.stderr}")
    
    # Read URDF file
    if os.path.exists(urdf_filepath):
        with open(urdf_filepath, 'r') as f:
            robot_description = f.read()
        # Replace mesh paths to use absolute file paths (since we're not using package://)
        mesh_dir = os.path.join(src_dir, 'role_ros2', 'robot_ik', 'franka', 'mesh')
        robot_description = robot_description.replace(
            'package://role_ros2/meshes/franka/',
            f'file://{mesh_dir}/'
        )
    else:
        raise FileNotFoundError(f"URDF file not found: {urdf_filepath}. Please run the conversion script first.")

    return [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
            # robot_state_publisher subscribes to /joint_states and publishes /tf
            # This will convert joint_states from polymetis_manager to TF transforms
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
        # Polymetis manager node - replaces franka_ros2 hardware interface
        Node(
            package='role_ros2',
            executable='polymetis_manager',
            name='polymetis_manager_node',
            output='screen',
            parameters=[
                {'arm_id': arm_id},
                {'ip_address': robot_ip},
                {'urdf_file': ''},  # Auto-detect from package
                {'launch_controller': True},
                {'launch_robot': True},
            ],
        ),
        # Robot state publisher - subscribes to /joint_states and publishes /tf
        robot_description_dependent_nodes_spawner_opaque_function,
        # Note: Gripper is controlled via Polymetis (polymetis_manager handles both arm and gripper)
        # If you need franka_gripper ROS2 node, uncomment below and add franka_gripper dependency
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([PathJoinSubstitution(
        #         [FindPackageShare('franka_gripper'), 'launch', 'gripper.launch.py'])]),
        #     launch_arguments={robot_ip_parameter_name: robot_ip,
        #                       use_fake_hardware_parameter_name: use_fake_hardware,
        #                       arm_id_parameter_name: arm_id}.items(),
        #     condition=IfCondition(load_gripper)
        # ),
    ])

    return launch_description

