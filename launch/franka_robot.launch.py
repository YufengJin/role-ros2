#!/usr/bin/env python3
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

"""
Franka Robot Launch File

This launch file starts the refactored robot interface nodes:
- franka_robot_interface_node: Arm control (namespace: /{arm_namespace})
- franka_gripper_interface_node: Gripper control (namespace: /{gripper_namespace})
- robot_state_aggregator_node: Merges states and publishes /joint_states

Features:
- Support for multiple robots via namespaces
- Clean separation of arm and gripper control
- Backward compatibility via robot_state_aggregator
- Mock mode for testing without hardware
- Auto-launch of Polymetis robot/gripper servers
- Auto-cleanup on shutdown
- Real-time control support

Usage:
    ros2 launch role_ros2 franka_robot.launch.py
    ros2 launch role_ros2 franka_robot.launch.py use_mock:=true
    ros2 launch role_ros2 franka_robot.launch.py arm_namespace:=robot1_arm gripper_namespace:=robot1_gripper

Author: Role-ROS2 Team
"""

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    GroupAction,
    ExecuteProcess,
    TimerAction,
    RegisterEventHandler,
)
from launch.event_handlers import OnShutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushRosNamespace

# Delay (seconds) before starting servers after cleanup so ports are released.
CLEANUP_DELAY = 3.0


def _get_cleanup_script_path():
    """Return path to kill_all.sh (external script so pkill -f does not self-match)."""
    return os.path.join(get_package_share_directory('role_ros2'), 'kill_all.sh')


def _run_hardware_script(script_path, *args):
    """Build bash -c string to activate conda/micromamba and run script with args."""
    quoted = ' '.join(f'"{a}"' for a in args)
    return (
        'if command -v micromamba &> /dev/null; then '
        '  eval "$(micromamba shell hook --shell=bash)" && micromamba activate polymetis-local && '
        f'  "{script_path}" {quoted}; '
        'elif command -v conda &> /dev/null; then '
        '  source $(conda info --base)/etc/profile.d/conda.sh && conda activate polymetis-local && '
        f'  "{script_path}" {quoted}; '
        'else '
        f'  "{script_path}" {quoted}; '
        'fi'
    )


def load_config_yaml(package_name, config_file):
    """Load launch configuration YAML file."""
    try:
        package_share_dir = get_package_share_directory(package_name)
        config_path = os.path.join(package_share_dir, 'config', config_file)
        if os.path.exists(config_path):
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)
                return config if config else {}
        print(f"Warning: Config file not found: {config_path}")
    except Exception as e:
        print(f"Warning: Could not load config file {config_file}: {e}")
    return {}


def create_server_launch_processes(context: LaunchContext, robot_ip, use_mock,
                                   auto_launch_controller, load_gripper,
                                   robot_port, gripper_port, use_real_time,
                                   gripper_type):
    """Create processes for launching Polymetis robot and gripper servers."""
    robot_ip_str = context.perform_substitution(robot_ip)
    use_mock_str = context.perform_substitution(use_mock)
    auto_launch_str = context.perform_substitution(auto_launch_controller)
    load_gripper_str = context.perform_substitution(load_gripper)
    robot_port_str = context.perform_substitution(robot_port)
    gripper_port_str = context.perform_substitution(gripper_port)
    gripper_type_str = context.perform_substitution(gripper_type)

    use_mock_bool = use_mock_str.lower() == 'true'
    auto_launch_bool = auto_launch_str.lower() == 'true'
    load_gripper_bool = load_gripper_str.lower() == 'true'

    processes = []
    if use_mock_bool or not auto_launch_bool:
        print(f"Info: Skipping server launch (use_mock={use_mock_bool}, auto_launch={auto_launch_bool})")
        return processes

    print(f"Info: Auto-launching Polymetis servers (robot_ip={robot_ip_str}, robot_port={robot_port_str})")

    processes.append(
        ExecuteProcess(
            cmd=['bash', _get_cleanup_script_path()],
            output='screen',
            name='cleanup_robot_gripper_servers',
        )
    )

    pkg_share = get_package_share_directory('role_ros2')
    launch_robot_script = os.path.join(pkg_share, 'launch_robot.sh')
    launch_gripper_script = os.path.join(pkg_share, 'launch_gripper.sh')

    server_actions_after_cleanup = [
        ExecuteProcess(
            cmd=['bash', '-c', _run_hardware_script(launch_robot_script, robot_ip_str, robot_port_str)],
            output='screen',
            name='launch_robot_server',
            additional_env={'ROBOT_IP': robot_ip_str},
        ),
    ]
    if load_gripper_bool:
        server_actions_after_cleanup.append(
            TimerAction(
                period=5.0,
                actions=[
                    ExecuteProcess(
                        cmd=['bash', '-c', _run_hardware_script(
                            launch_gripper_script, robot_ip_str, gripper_port_str, gripper_type_str
                        )],
                        output='screen',
                        name='launch_gripper_server',
                        additional_env={'ROBOT_IP': robot_ip_str},
                    ),
                ],
            )
        )

    processes.append(TimerAction(period=CLEANUP_DELAY, actions=server_actions_after_cleanup))
    return processes


def create_robot_nodes(context: LaunchContext, arm_id, urdf_file, use_mock,
                       arm_namespace, gripper_namespace, publish_rate, load_gripper,
                       auto_launch_controller, robot_ip, robot_port, gripper_port,
                       auto_reset_on_startup, auto_reset_delay):
    """Create robot nodes with resolved parameters using OpaqueFunction."""
    # Resolve LaunchConfiguration values
    arm_id_str = context.perform_substitution(arm_id)
    urdf_file_str = context.perform_substitution(urdf_file)
    use_mock_str = context.perform_substitution(use_mock)
    arm_namespace_str = context.perform_substitution(arm_namespace)
    gripper_namespace_str = context.perform_substitution(gripper_namespace)
    publish_rate_str = context.perform_substitution(publish_rate)
    load_gripper_str = context.perform_substitution(load_gripper)
    auto_launch_str = context.perform_substitution(auto_launch_controller)
    robot_ip_str = context.perform_substitution(robot_ip)
    robot_port_str = context.perform_substitution(robot_port)
    gripper_port_str = context.perform_substitution(gripper_port)
    auto_reset_str = context.perform_substitution(auto_reset_on_startup)
    auto_reset_delay_str = context.perform_substitution(auto_reset_delay)
    
    # Convert string to bool/float
    use_mock_bool = use_mock_str.lower() == 'true'
    load_gripper_bool = load_gripper_str.lower() == 'true'
    auto_launch_bool = auto_launch_str.lower() == 'true'
    auto_reset_bool = auto_reset_str.lower() == 'true'
    publish_rate_float = float(publish_rate_str)
    auto_reset_delay_float = float(auto_reset_delay_str)
    
    # Load config (URDF path and joint names)
    config = load_config_yaml('role_ros2', 'franka_robot_config.yaml')
    arm_joints_from_config = config.get('arm_joints', [])
    gripper_joints_from_config = config.get('gripper_joints', [])

    if not urdf_file_str:
        urdf_file_str = config.get('urdf_file', f'{arm_id_str}.urdf')

    package_share_dir = get_package_share_directory('role_ros2')
    urdf_filepath = os.path.join(package_share_dir, 'robot_ik', 'franka', urdf_file_str)
    
    if not os.path.exists(urdf_filepath):
        raise FileNotFoundError(
            f"URDF file not found: {urdf_filepath}\n"
            "Please rebuild the workspace: colcon build --packages-select role_ros2"
        )
    
    with open(urdf_filepath, 'r') as f:
        robot_description = f.read()
    
    if not robot_description or len(robot_description.strip()) == 0:
        raise ValueError(f"URDF file is empty: {urdf_filepath}")
    
    print(f"Info: Successfully loaded URDF from {urdf_filepath}")
    print(f"Info: use_mock={use_mock_bool}, arm_namespace={arm_namespace_str}, gripper_namespace={gripper_namespace_str}")
    print(f"Info: auto_launch_controller={auto_launch_bool}, auto_reset_on_startup={auto_reset_bool}")
    
    nodes = []
    
    # Calculate delay for nodes (if auto-launching servers, wait for them to start)
    # Arm node delay: 5s to wait for robot server
    # Gripper node needs extra delay: 5s (robot server) + 5s (gripper server) + 3s (initialization)
    arm_node_delay = 5.0 if (auto_launch_bool and not use_mock_bool) else 0.0
    gripper_node_delay = 13.0 if (auto_launch_bool and not use_mock_bool) else 0.0
    
    # ========== Robot Arm Interface Node ==========
    arm_node_params = [
        {'use_mock': use_mock_bool},
        {'ip_address': 'localhost'},
        {'publish_rate': publish_rate_float},
        {'namespace': arm_namespace_str},
        {'auto_reset_on_startup': auto_reset_bool},
        {'auto_reset_delay': auto_reset_delay_float},
        {'arm_joint_names': arm_joints_from_config},
        {'polymetis_port': int(robot_port_str)},
    ]
    arm_node = GroupAction([
        PushRosNamespace(arm_namespace_str),
        Node(
            package='role_ros2',
            executable='franka_robot_interface',
            name='franka_robot_interface_node',
            output='screen',
            parameters=arm_node_params,
        ),
    ])
    
    # ========== Gripper Interface Node ==========
    gripper_node_params = [
        {'use_mock': use_mock_bool},
        {'ip_address': 'localhost'},
        {'publish_rate': publish_rate_float},
        {'namespace': gripper_namespace_str},
        {'gripper_joint_names': gripper_joints_from_config},
        {'polymetis_port': int(gripper_port_str)},
    ]
    gripper_node = None
    if load_gripper_bool:
        gripper_node = GroupAction([
            PushRosNamespace(gripper_namespace_str),
            Node(
                package='role_ros2',
                executable='franka_gripper_interface',
                name='franka_gripper_interface_node',
                output='screen',
                parameters=gripper_node_params,
            ),
        ])
    
    # ========== Robot State Aggregator Node ==========
    robot_namespaces = [arm_namespace_str]
    if load_gripper_bool:
        robot_namespaces.append(gripper_namespace_str)
    
    aggregator_node = Node(
        package='role_ros2',
        executable='robot_state_aggregator',
        name='robot_state_aggregator_node',
        output='screen',
        parameters=[
            {'robot_namespaces': robot_namespaces},
            {'publish_rate': publish_rate_float},
            {'timeout_threshold': 1.0},
        ],
    )
    
    # ========== Robot State Publisher ==========
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
        }],
    )
    
    # If auto-launching servers, delay the nodes appropriately
    # Arm and other nodes: wait for robot server (5s)
    # Gripper node: wait longer for gripper server to fully initialize (13s)
    if arm_node_delay > 0:
        # Arm node and other nodes with arm_node_delay
        arm_and_other_nodes = [arm_node, aggregator_node, robot_state_publisher_node]
        nodes.append(
            TimerAction(
                period=arm_node_delay,
                actions=arm_and_other_nodes
            )
        )
        
        # Gripper node with longer delay (gripper server needs more time)
        if gripper_node:
            nodes.append(
                TimerAction(
                    period=gripper_node_delay,
                    actions=[gripper_node]
                )
            )
    else:
        nodes.append(arm_node)
        if gripper_node:
            nodes.append(gripper_node)
        nodes.append(aggregator_node)
        nodes.append(robot_state_publisher_node)
    
    return nodes


def generate_launch_description():
    # Load default values from YAML config file
    config = load_config_yaml('role_ros2', 'franka_robot_config.yaml')
    
    def get_default(key, default_value):
        value = config.get(key, default_value)
        if isinstance(value, bool):
            return 'true' if value else 'false'
        return str(value)
    
    # Launch configurations
    arm_id = LaunchConfiguration('arm_id')
    use_mock = LaunchConfiguration('use_mock')
    arm_namespace = LaunchConfiguration('arm_namespace')
    gripper_namespace = LaunchConfiguration('gripper_namespace')
    urdf_file = LaunchConfiguration('urdf_file')
    publish_rate = LaunchConfiguration('publish_rate')
    load_gripper = LaunchConfiguration('load_gripper')
    auto_launch_controller = LaunchConfiguration('auto_launch_controller')
    robot_ip = LaunchConfiguration('robot_ip')
    robot_port = LaunchConfiguration('robot_port')
    gripper_port = LaunchConfiguration('gripper_port')
    use_real_time = LaunchConfiguration('use_real_time')
    gripper_type = LaunchConfiguration('gripper_type')
    auto_reset_on_startup = LaunchConfiguration('auto_reset_on_startup')
    auto_reset_delay = LaunchConfiguration('auto_reset_delay')
    
    # Create server launch processes using OpaqueFunction
    server_processes_opaque = OpaqueFunction(
        function=create_server_launch_processes,
        args=[robot_ip, use_mock, auto_launch_controller, load_gripper,
              robot_port, gripper_port, use_real_time, gripper_type]
    )
    
    # Create all nodes using OpaqueFunction to resolve parameters
    robot_nodes_opaque = OpaqueFunction(
        function=create_robot_nodes,
        args=[arm_id, urdf_file, use_mock, arm_namespace, gripper_namespace, publish_rate,
              load_gripper, auto_launch_controller, robot_ip, robot_port, gripper_port,
              auto_reset_on_startup, auto_reset_delay]
    )
    
    launch_description = LaunchDescription([
        # ========== Declare Launch Arguments ==========
        DeclareLaunchArgument(
            'arm_id',
            default_value=get_default('arm_id', 'fr3'),
            description='ID of the arm type (fer, fr3, fp3)'
        ),
        DeclareLaunchArgument(
            'robot_ip',
            default_value=get_default('robot_ip', '172.17.0.2'),
            description='IP address of the robot'
        ),
        DeclareLaunchArgument(
            'use_mock',
            default_value=get_default('use_mock', 'false'),
            description='Use mock interfaces (no real robot)'
        ),
        DeclareLaunchArgument(
            'arm_namespace',
            default_value=get_default('arm_namespace', 'fr3_arm'),
            description='ROS namespace for arm interface node'
        ),
        DeclareLaunchArgument(
            'gripper_namespace',
            default_value=get_default('gripper_namespace', 'fr3_gripper'),
            description='ROS namespace for gripper interface node'
        ),
        DeclareLaunchArgument(
            'urdf_file',
            default_value=get_default('urdf_file', 'fr3.urdf'),
            description='URDF filename (fr3.urdf, fr3_w_fin.urdf, fr3_w_soft_fin.urdf)'
        ),
        DeclareLaunchArgument(
            'publish_rate',
            default_value=get_default('publish_rate', '50.0'),
            description='State publishing rate in Hz'
        ),
        DeclareLaunchArgument(
            'load_gripper',
            default_value=get_default('load_gripper', 'true'),
            description='Load gripper interface node'
        ),
        DeclareLaunchArgument(
            'auto_launch_controller',
            default_value=get_default('auto_launch_controller', 'true'),
            description='Auto-launch robot and gripper servers (only in real robot mode)'
        ),
        DeclareLaunchArgument(
            'robot_port',
            default_value=get_default('robot_port', '50051'),
            description='Polymetis robot server port'
        ),
        DeclareLaunchArgument(
            'gripper_port',
            default_value=get_default('gripper_port', '50052'),
            description='Polymetis gripper server port'
        ),
        DeclareLaunchArgument(
            'use_real_time',
            default_value=get_default('use_real_time', 'true'),
            description='Use real-time control for Polymetis server'
        ),
        DeclareLaunchArgument(
            'gripper_type',
            default_value=get_default('gripper_type', 'franka_hand'),
            description='Gripper type: franka_hand or robotiq_2f'
        ),
        DeclareLaunchArgument(
            'auto_reset_on_startup',
            default_value=get_default('auto_reset_on_startup', 'false'),
            description='Auto-reset robot to home position on startup'
        ),
        DeclareLaunchArgument(
            'auto_reset_delay',
            default_value=get_default('auto_reset_delay', '5.0'),
            description='Delay before auto-reset (seconds)'
        ),
        
        # ========== Launch Server Processes (Real Robot Only) ==========
        server_processes_opaque,
        
        # ========== Create All Robot Nodes ==========
        robot_nodes_opaque,
        
        # ========== Cleanup on Shutdown ==========
        RegisterEventHandler(
            OnShutdown(
                on_shutdown=[
                    ExecuteProcess(
                        cmd=['bash', '-c', 'echo "Cleaning up robot and gripper servers..." && bash "%s" && echo "Cleanup complete."' % _get_cleanup_script_path()],
                        output='screen',
                        condition=IfCondition(
                            PythonExpression(["'", use_mock, "' == 'false' and '", auto_launch_controller, "' == 'true'"])
                        ),
                    ),
                ],
            ),
        ),
    ])
    
    return launch_description
