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
    ExecuteProcess,
    TimerAction,
    GroupAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
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

    # Load URDF from install directory only (must be built first)
    package_share_dir = get_package_share_directory('role_ros2')
    urdf_filepath = os.path.join(package_share_dir, 'robot', 'franka', arm_id_str, f'{arm_id_str}.urdf')
    
    # Check if URDF file exists in install directory
    if not os.path.exists(urdf_filepath):
        error_msg = f"""
================================================================================
ERROR: URDF file not found in install directory: {urdf_filepath}
================================================================================

The URDF file must be installed to the workspace install directory.
Please rebuild the workspace:

    cd /app/ros2_ws
    colcon build --packages-select role_ros2
    source install/setup.bash

The URDF file should be installed to:
    install/role_ros2/share/role_ros2/robot/franka/{arm_id_str}/{arm_id_str}.urdf

Make sure the source URDF file exists at:
    src/role-ros2/role_ros2/robot_ik/franka/{arm_id_str}.urdf

And that CMakeLists.txt includes the install rule for URDF files.
================================================================================
"""
        raise FileNotFoundError(error_msg)
    
    # Read URDF file
    try:
        with open(urdf_filepath, 'r') as f:
            robot_description = f.read()
        
        if not robot_description or len(robot_description.strip()) == 0:
            raise ValueError(f"URDF file is empty: {urdf_filepath}")
        
        print(f"Info: Successfully loaded URDF from {urdf_filepath}")
        print(f"Info: URDF length: {len(robot_description)} characters")
    except Exception as e:
        error_msg = f"""
================================================================================
ERROR: Failed to read URDF file: {urdf_filepath}
================================================================================

Error: {e}

Please check:
  1. File exists and is readable
  2. File is not empty
  3. File is valid XML/URDF format
================================================================================
"""
        raise FileNotFoundError(error_msg)
    
    # Handle mesh paths for rviz visualization
    # Mesh files must be installed to install directory (use package:// paths only)
    installed_mesh_dir = os.path.join(package_share_dir, 'robot', 'franka', arm_id_str, 'mesh')
    
    if not os.path.exists(installed_mesh_dir):
        print(f"Warning: Mesh directory not found in install directory: {installed_mesh_dir}")
        print(f"  URDF may not display correctly in rviz.")
        print(f"  Please rebuild the workspace:")
        print(f"    cd /app/ros2_ws")
        print(f"    colcon build --packages-select role_ros2")
        print(f"    source install/setup.bash")
        print(f"  Expected mesh files at: {installed_mesh_dir}/")
    else:
        # Mesh files are installed, update URDF mesh paths to match new install location
        # Replace old path: package://role_ros2/robot_ik/franka/mesh/
        # With new path: package://role_ros2/robot/franka/{arm_id}/mesh/
        old_mesh_path = 'package://role_ros2/robot_ik/franka/mesh/'
        new_mesh_path = f'package://role_ros2/robot/franka/{arm_id_str}/mesh/'
        
        # Count replacements to verify they happened
        count_before = robot_description.count(old_mesh_path)
        robot_description = robot_description.replace(old_mesh_path, new_mesh_path)
        count_after = robot_description.count(new_mesh_path)
        
        # Also handle alternative path pattern if it exists
        old_mesh_path_alt = 'package://role_ros2/meshes/franka/'
        count_alt_before = robot_description.count(old_mesh_path_alt)
        robot_description = robot_description.replace(old_mesh_path_alt, new_mesh_path)
        
        print(f"Info: Using package:// paths for mesh files (installed at {installed_mesh_dir})")
        print(f"Info: Updated mesh paths in URDF:")
        print(f"  - Replaced {count_before} occurrences of '{old_mesh_path}'")
        print(f"  - Replaced {count_alt_before} occurrences of '{old_mesh_path_alt}'")
        print(f"  - Total '{new_mesh_path}' occurrences: {count_after}")
    
    # Verify robot_description is valid before passing to node
    if not robot_description or len(robot_description.strip()) == 0:
        raise ValueError("robot_description is empty after processing!")
    
    # Check if robot_description contains basic URDF structure
    if '<robot' not in robot_description:
        print(f"Warning: URDF may be invalid - '<robot' tag not found")
    
    print(f"Info: robot_description ready, length: {len(robot_description)} characters")
    
    # robot_state_publisher automatically publishes /robot_description topic when robot_description parameter is set
    # It uses TransientLocal durability QoS, so subscribers can get the message even if they connect late
    # The parameter must be a string containing the URDF XML content
    # Standard ROS2 pattern: read URDF file and pass as string parameter
    
    return [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,  # String parameter containing URDF XML
            }],
            # robot_state_publisher will:
            # 1. Use robot_description parameter to publish /robot_description topic (std_msgs/String) with TransientLocal QoS
            # 2. Subscribe to /joint_states and publish /tf transforms
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
    auto_launch_controller_parameter_name = 'auto_launch_controller'

    arm_id = LaunchConfiguration(arm_id_parameter_name)
    robot_ip = LaunchConfiguration(robot_ip_parameter_name)
    load_gripper = LaunchConfiguration(load_gripper_parameter_name)
    use_fake_hardware = LaunchConfiguration(use_fake_hardware_parameter_name)
    fake_sensor_commands = LaunchConfiguration(
        fake_sensor_commands_parameter_name)
    controller_name = LaunchConfiguration(controller_name_parameter_name)
    use_mock = LaunchConfiguration(use_mock_parameter_name)
    use_fake_joint_states = LaunchConfiguration('use_fake_joint_states')
    auto_launch_controller = LaunchConfiguration(auto_launch_controller_parameter_name)

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
        DeclareLaunchArgument(
            auto_launch_controller_parameter_name,
            default_value=get_default('auto_launch_controller', 'true'),
            description='Auto-launch robot and gripper servers before polymetis_bridge (only in real robot mode). Default: true'),
        
        # ========== Real Robot Mode: Auto-launch robot and gripper servers ==========
        # Step 1: Kill any existing server processes (cleanup)
        ExecuteProcess(
            cmd=['pkill', '-9', 'run_server'],
            output='screen',
            condition=IfCondition(
                PythonExpression(["'", use_mock, "' == 'false' and '", auto_launch_controller, "' == 'true'"])
            ),
        ),
        ExecuteProcess(
            cmd=['pkill', '-9', 'franka_panda_cl'],
            output='screen',
            condition=IfCondition(
                PythonExpression(["'", use_mock, "' == 'false' and '", auto_launch_controller, "' == 'true'"])
            ),
        ),
        ExecuteProcess(
            cmd=['pkill', '-9', 'gripper'],
            output='screen',
            condition=IfCondition(
                PythonExpression(["'", use_mock, "' == 'false' and '", auto_launch_controller, "' == 'true'"])
            ),
        ),
        
        # Step 2: Launch robot server (launch_robot.py)
        # This needs to run in the polymetis conda/micromamba environment
        # In Docker, the environment should already be activated via entrypoint.sh
        ExecuteProcess(
            cmd=[
                'bash', '-c',
                # Try micromamba first (Docker), then conda (host)
                'if command -v micromamba &> /dev/null; then '
                '  eval "$(micromamba shell hook --shell=bash)" && micromamba activate polymetis-local && '
                '  launch_robot.py robot_client=franka_hardware robot_client.executable_cfg.robot_ip=' + '${ROBOT_IP}' + '; '
                'elif command -v conda &> /dev/null; then '
                '  source $(conda info --base)/etc/profile.d/conda.sh && conda activate polymetis-local && '
                '  launch_robot.py robot_client=franka_hardware robot_client.executable_cfg.robot_ip=' + '${ROBOT_IP}' + '; '
                'else '
                '  launch_robot.py robot_client=franka_hardware robot_client.executable_cfg.robot_ip=' + '${ROBOT_IP}' + '; '
                'fi'
            ],
            output='screen',
            name='launch_robot_server',
            additional_env={'ROBOT_IP': robot_ip},
            condition=IfCondition(
                PythonExpression(["'", use_mock, "' == 'false' and '", auto_launch_controller, "' == 'true'"])
            ),
        ),
        
        # Step 3: Launch gripper server (launch_gripper.py) - delayed 2 seconds after robot
        TimerAction(
            period=2.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'bash', '-c',
                        # Try micromamba first (Docker), then conda (host)
                        'if command -v micromamba &> /dev/null; then '
                        '  eval "$(micromamba shell hook --shell=bash)" && micromamba activate polymetis-local && '
                        '  launch_gripper.py gripper=franka_hand gripper.executable_cfg.robot_ip=' + '${ROBOT_IP}' + '; '
                        'elif command -v conda &> /dev/null; then '
                        '  source $(conda info --base)/etc/profile.d/conda.sh && conda activate polymetis-local && '
                        '  launch_gripper.py gripper=franka_hand gripper.executable_cfg.robot_ip=' + '${ROBOT_IP}' + '; '
                        'else '
                        '  launch_gripper.py gripper=franka_hand gripper.executable_cfg.robot_ip=' + '${ROBOT_IP}' + '; '
                        'fi'
                    ],
                    output='screen',
                    name='launch_gripper_server',
                    additional_env={'ROBOT_IP': robot_ip},
                    condition=IfCondition(
                        PythonExpression(["'", use_mock, "' == 'false' and '", auto_launch_controller, "' == 'true' and '", load_gripper, "' == 'true'"])
                    ),
                ),
            ],
        ),
        
        # Step 4: Polymetis bridge node - delayed 5 seconds to wait for servers
        # Real robot mode with auto_launch_controller: wait for servers to start
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='role_ros2',
                    executable='polymetis_bridge',
                    name='polymetis_bridge_node',
                    output='screen',
                    parameters=[
                        {'use_mock': False},  # Real robot mode
                        {'ip_address': 'localhost'},  # Connect to local polymetis server
                        {'publish_rate': 50.0},
                        {'auto_launch_controller': False},  # Don't launch again, already launched above
                        {'robot_ip': robot_ip},
                        {'auto_reset_on_startup': config.get('auto_reset_on_startup', True)},  # From config
                        {'auto_reset_delay': config.get('auto_reset_delay', 5.0)},  # From config (5s default to wait for servers)
                    ],
                    condition=IfCondition(
                        PythonExpression(["'", use_mock, "' == 'false' and '", auto_launch_controller, "' == 'true'"])
                    ),
                ),
            ],
        ),
        
        # Real robot mode WITHOUT auto_launch_controller: start immediately
        # User is responsible for starting robot/gripper servers manually
        Node(
            package='role_ros2',
            executable='polymetis_bridge',
            name='polymetis_bridge_node',
            output='screen',
            parameters=[
                {'use_mock': False},  # Real robot mode
                {'ip_address': 'localhost'},  # Connect to local polymetis server
                {'publish_rate': 50.0},
                {'auto_launch_controller': False},  # Don't auto-launch
                {'robot_ip': robot_ip},
                {'auto_reset_on_startup': config.get('auto_reset_on_startup', True)},  # From config
                {'auto_reset_delay': config.get('auto_reset_delay', 5.0)},  # From config
            ],
            condition=IfCondition(
                PythonExpression(["'", use_mock, "' == 'false' and '", auto_launch_controller, "' == 'false'"])
            ),
        ),
        
        # ========== Mock Mode ==========
        # Polymetis bridge node in mock mode (uses MockRobotInterface)
        # This publishes joint_states, robot_state, and gripper_state
        Node(
            package='role_ros2',
            executable='polymetis_bridge',
            name='polymetis_bridge_node',
            output='screen',
            parameters=[
                {'use_mock': True},  # Force mock mode
                {'ip_address': 'localhost'},
                {'publish_rate': 50.0},
                {'auto_launch_controller': False},  # No controller in mock mode
                {'auto_reset_on_startup': config.get('auto_reset_on_startup', True)},  # From config
                {'auto_reset_delay': config.get('auto_reset_delay', 5.0)},  # From config
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

