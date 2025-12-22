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
    OpaqueFunction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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
        controller_name,
        use_mock=None):

    robot_ip_str = context.perform_substitution(robot_ip)
    arm_id_str = context.perform_substitution(arm_id)
    use_fake_hardware_str = context.perform_substitution(use_fake_hardware)
    fake_sensor_commands_str = context.perform_substitution(
        fake_sensor_commands)
    load_gripper_str = context.perform_substitution(load_gripper)
    controller_name_str = context.perform_substitution(controller_name)
    
    # Get use_mock from context if not provided
    if use_mock is not None:
        use_mock_str = context.perform_substitution(use_mock)
    else:
        use_mock_str = context.launch_configurations.get('use_mock', 'false')

    # Load URDF from role_ros2 package (converted from MuJoCo XML)
    # Try multiple paths to find URDF file (works in both source and install contexts)
    package_share_dir = get_package_share_directory('role_ros2')
    
    # Path 1: Try source directory (for development with volume mounts)
    src_dir = os.path.join(package_share_dir, '..', '..', 'src', 'role_ros2')
    urdf_filepath = os.path.join(src_dir, 'role_ros2', 'robot_ik', 'franka', f'{arm_id_str}.urdf')
    
    # Path 2: If not found, try direct source path (when running from source)
    if not os.path.exists(urdf_filepath):
        # Try /app/ros2_ws/src/role-ros2/role_ros2/robot_ik/franka/fr3.urdf
        direct_path = f'/app/ros2_ws/src/role-ros2/role_ros2/robot_ik/franka/{arm_id_str}.urdf'
        if os.path.exists(direct_path):
            urdf_filepath = direct_path
            src_dir = '/app/ros2_ws/src/role-ros2'
    
    # Path 3: If still not found, try relative to launch file location
    if not os.path.exists(urdf_filepath):
        launch_file_dir = os.path.dirname(os.path.abspath(__file__))
        relative_path = os.path.join(launch_file_dir, '..', 'role_ros2', 'robot_ik', 'franka', f'{arm_id_str}.urdf')
        relative_path = os.path.abspath(relative_path)
        if os.path.exists(relative_path):
            urdf_filepath = relative_path
            src_dir = os.path.join(os.path.dirname(launch_file_dir), 'role_ros2')
    
    # Check if URDF file exists - if not, raise error with helpful message
    if not os.path.exists(urdf_filepath):
        # Find XML file path for conversion script reference
        xml_filepath = os.path.join(os.path.dirname(urdf_filepath), f'{arm_id_str}.xml')
        convert_script_path = os.path.join(src_dir, 'scripts', 'mujoco_to_urdf.py')
        
        error_msg = f"""
================================================================================
ERROR: URDF file not found: {urdf_filepath}
================================================================================

To generate the URDF file, run the conversion script:

    python3 {convert_script_path} {xml_filepath} {urdf_filepath} {arm_id_str}

Or from the role_ros2 package directory:

    python3 scripts/mujoco_to_urdf.py \\
        role_ros2/robot_ik/franka/{arm_id_str}.xml \\
        role_ros2/robot_ik/franka/{arm_id_str}.urdf \\
        {arm_id_str}

Note: The URDF file must conform to ROS standards and include proper mesh links.
Make sure the mesh directory exists at: {os.path.dirname(urdf_filepath)}/mesh/
================================================================================
"""
        raise FileNotFoundError(error_msg)
    
    # Read URDF file
    with open(urdf_filepath, 'r') as f:
        robot_description = f.read()
    
    # Handle mesh paths for rviz visualization
    # Option 1: Use package:// paths (requires mesh files to be installed via CMakeLists.txt)
    # Option 2: Use absolute file:// paths (works in development mode)
    # We try package:// first, then fall back to absolute paths if needed
    
    mesh_dir = os.path.join(os.path.dirname(urdf_filepath), 'mesh')
    mesh_dir = os.path.abspath(mesh_dir)  # Ensure absolute path
    
    # Check if we're running from install space (mesh files should be installed)
    # or from source space (use absolute paths)
    package_share_dir = get_package_share_directory('role_ros2')
    installed_mesh_dir = os.path.join(package_share_dir, 'robot_ik', 'franka', 'mesh')
    
    if os.path.exists(installed_mesh_dir):
        # Mesh files are installed, use package:// paths (standard ROS2 way)
        # Keep package:// paths as-is - ROS2 will resolve them correctly
        print(f"Info: Using package:// paths for mesh files (installed at {installed_mesh_dir})")
        # No replacement needed - package:// paths work when files are installed
    elif os.path.exists(mesh_dir):
        # Running from source, replace with absolute file:// paths
        mesh_file_prefix = f'file://{mesh_dir}/'
        robot_description = robot_description.replace(
            'package://role_ros2/meshes/franka/',
            mesh_file_prefix
        )
        robot_description = robot_description.replace(
            'package://role_ros2/robot_ik/franka/mesh/',
            mesh_file_prefix
        )
        print(f"Info: Mesh directory found at {mesh_dir}. Using absolute file:// paths for rviz.")
    else:
        print(f"Warning: Mesh directory not found at {mesh_dir} or {installed_mesh_dir}.")
        print(f"  URDF may not display correctly in rviz.")
        print(f"  Expected mesh files at:")
        print(f"    - Source: {mesh_dir}/")
        print(f"    - Install: {installed_mesh_dir}/")
        print(f"  URDF file location: {urdf_filepath}")

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
    use_mock_parameter_name = 'use_mock'

    arm_id = LaunchConfiguration(arm_id_parameter_name)
    robot_ip = LaunchConfiguration(robot_ip_parameter_name)
    load_gripper = LaunchConfiguration(load_gripper_parameter_name)
    use_fake_hardware = LaunchConfiguration(use_fake_hardware_parameter_name)
    fake_sensor_commands = LaunchConfiguration(
        fake_sensor_commands_parameter_name)
    controller_name = LaunchConfiguration(controller_name_parameter_name)
    use_mock = LaunchConfiguration(use_mock_parameter_name)
    use_fake_joint_states = LaunchConfiguration('use_fake_joint_states')

    robot_description_dependent_nodes_spawner_opaque_function = OpaqueFunction(
        function=robot_description_dependent_nodes_spawner,
        args=[
            robot_ip,
            arm_id,
            use_fake_hardware,
            fake_sensor_commands,
            load_gripper,
            controller_name,
            use_mock])

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
        DeclareLaunchArgument(
            use_mock_parameter_name,
            default_value=get_default('use_mock', 'false'),
            description='Use mock mode (no real robot). If true, uses fake joint_states publisher and mock interfaces. Default from config/franka_robot_config.yaml'),
        DeclareLaunchArgument(
            'use_fake_joint_states',
            default_value='false',
            description='Use fake_joint_states_publisher instead of polymetis_bridge for joint_states. Only valid in mock mode. Default: false'),
        # Polymetis bridge node - replaces franka_ros2 hardware interface
        # Real robot mode: connects to actual Polymetis server
        Node(
            package='role_ros2',
            executable='polymetis_bridge',
            name='polymetis_bridge_node',
            output='screen',
            parameters=[
                {'use_mock': False},  # Real robot mode
                {'ip_address': robot_ip},
                {'publish_rate': 50.0},
                {'arm_id': arm_id},
            ],
            condition=UnlessCondition(use_mock),  # Only run if NOT in mock mode
        ),
        # Polymetis bridge node in mock mode (uses MockRobotInterface)
        # This publishes joint_states, robot_state, and gripper_state
        # Only runs if use_mock=true AND use_fake_joint_states=false
        # We need to combine conditions: use_mock=true AND use_fake_joint_states=false
        # Since launch doesn't support AND conditions easily, we use a workaround:
        # If use_fake_joint_states=true, this node won't run (fake_joint_states_publisher will run instead)
        # Note: In practice, if both run, the last publisher wins, but we prefer polymetis_bridge
        # when use_fake_joint_states is false
        Node(
            package='role_ros2',
            executable='polymetis_bridge',
            name='polymetis_bridge_node',
            output='screen',
            parameters=[
                {'use_mock': True},  # Force mock mode
                {'ip_address': 'localhost'},
                {'publish_rate': 50.0},
                {'arm_id': arm_id},
            ],
            condition=IfCondition(use_mock),  # Run in mock mode
        ),
        # Fake joint states publisher (alternative to polymetis_bridge in mock mode)
        # Use this if you only want joint_states without full polymetis_bridge functionality
        # This is mainly for testing robot_state_publisher independently
        Node(
            package='role_ros2',
            executable='fake_joint_states_publisher',
            name='fake_joint_states_publisher',
            output='screen',
            parameters=[
                {'arm_id': arm_id},
                {'publish_rate': 50.0},
            ],
            condition=IfCondition(use_fake_joint_states),  # Only if explicitly requested
        ),
        # Robot state publisher - subscribes to /joint_states and publishes /tf
        # This is always launched to convert joint_states to TF
        robot_description_dependent_nodes_spawner_opaque_function,
        # Note: Gripper is controlled via Polymetis (polymetis_bridge handles both arm and gripper)
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

