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
Bimanual Franka Robot Launch File

Starts two arms and two grippers (left_arm, right_arm, left_gripper, right_gripper)
using franka_robot_interface_node and franka_gripper_interface_node.
All parameters (robot_ip, robot_port, gripper_port, etc.) from bimanual_franka_robot_config.yaml.

Usage:
    ros2 launch role_ros2 bimanual_franka_robot.launch.py
    ros2 launch role_ros2 bimanual_franka_robot.launch.py use_mock:=true
"""

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    OpaqueFunction,
    TimerAction,
    RegisterEventHandler,
)
from launch.event_handlers import OnShutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushRosNamespace

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
        else:
            print(f"Warning: Config file not found: {config_path}")
    except Exception as e:
        print(f"Warning: Could not load config file {config_file}: {e}")
    return {}


def create_bimanual_server_launch_processes(context: LaunchContext, use_mock):
    """Create processes for launching Polymetis robot and gripper servers for both arms.

    Ports and IPs are read from bimanual_franka_robot_config.yaml (left_arm / right_arm).
    When auto_launch is used, servers run on the same machine; use different robot_port /
    gripper_port per side in config to avoid conflict (e.g. left 50051/50052, right 50053/50054).
    """
    use_mock_str = context.perform_substitution(use_mock)
    use_mock_bool = use_mock_str.lower() == 'true'

    config = load_config_yaml('role_ros2', 'bimanual_franka_robot_config.yaml')
    if not config:
        return []

    left = config.get('left_arm', {})
    right = config.get('right_arm', {})
    left_auto = left.get('auto_launch_controller', False)
    right_auto = right.get('auto_launch_controller', False)
    left_load_gripper = left.get('load_gripper', True)
    right_load_gripper = right.get('load_gripper', True)
    left_ip = left.get('robot_ip', '172.17.0.2')
    right_ip = right.get('robot_ip', '172.16.0.2')
    left_robot_port = str(left.get('robot_port', '50051'))
    left_gripper_port = str(left.get('gripper_port', '50052'))
    right_robot_port = str(right.get('robot_port', '50053'))
    right_gripper_port = str(right.get('gripper_port', '50054'))
    left_gripper_type = left.get('gripper_type', 'franka_hand')
    right_gripper_type = right.get('gripper_type', 'franka_hand')

    if use_mock_bool or (not left_auto and not right_auto):
        print("Info: Skipping bimanual server launch (use_mock or auto_launch disabled)")
        return []

    processes = []
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

    server_actions_after_cleanup = []
    if left_auto:
        server_actions_after_cleanup.append(
            ExecuteProcess(
                cmd=['bash', '-c', _run_hardware_script(launch_robot_script, left_ip, left_robot_port)],
                output='screen',
                name='launch_left_robot_server',
                additional_env={'ROBOT_IP': left_ip},
            )
        )
    if right_auto:
        server_actions_after_cleanup.append(
            ExecuteProcess(
                cmd=['bash', '-c', _run_hardware_script(launch_robot_script, right_ip, right_robot_port)],
                output='screen',
                name='launch_right_robot_server',
                additional_env={'ROBOT_IP': right_ip},
            )
        )

    gripper_actions = []
    if left_auto and left_load_gripper:
        gripper_actions.append(
            ExecuteProcess(
                cmd=['bash', '-c', _run_hardware_script(
                    launch_gripper_script, left_ip, left_gripper_port, left_gripper_type
                )],
                output='screen',
                name='launch_left_gripper_server',
                additional_env={'ROBOT_IP': left_ip},
            )
        )
    if right_auto and right_load_gripper:
        gripper_actions.append(
            ExecuteProcess(
                cmd=['bash', '-c', _run_hardware_script(
                    launch_gripper_script, right_ip, right_gripper_port, right_gripper_type
                )],
                output='screen',
                name='launch_right_gripper_server',
                additional_env={'ROBOT_IP': right_ip},
            )
        )
    if gripper_actions:
        server_actions_after_cleanup.append(TimerAction(period=5.0, actions=gripper_actions))

    processes.append(TimerAction(period=CLEANUP_DELAY, actions=server_actions_after_cleanup))
    return processes


def create_bimanual_nodes(context: LaunchContext, use_mock, publish_rate):
    """Create left/right arm and gripper nodes with joint names from bimanual config."""
    use_mock_str = context.perform_substitution(use_mock)
    publish_rate_str = context.perform_substitution(publish_rate)
    use_mock_bool = use_mock_str.lower() == 'true'
    publish_rate_float = float(publish_rate_str)

    config = load_config_yaml('role_ros2', 'bimanual_franka_robot_config.yaml')
    if not config:
        raise RuntimeError("Failed to load bimanual_franka_robot_config.yaml")

    left = config.get('left_arm', {})
    right = config.get('right_arm', {})
    urdf_file = left.get('urdf_file', right.get('urdf_file', 'bimanual_fr3_w_soft_fin.urdf'))
    left_use_mock = left.get('use_mock', use_mock_bool) if isinstance(left.get('use_mock'), bool) else use_mock_bool
    right_use_mock = right.get('use_mock', use_mock_bool) if isinstance(right.get('use_mock'), bool) else use_mock_bool
    left_load_gripper = left.get('load_gripper', True)
    right_load_gripper = right.get('load_gripper', True)
    left_auto = left.get('auto_launch_controller', False)
    right_auto = right.get('auto_launch_controller', False)
    auto_launch_bool = (left_auto or right_auto) and not use_mock_bool

    # Ports always from config. When auto_launch: connect to localhost; else connect to robot_ip.
    left_arm_port = int(left.get('robot_port', 50051))
    right_arm_port = int(right.get('robot_port', 50053))
    left_gripper_port = int(left.get('gripper_port', 50052))
    right_gripper_port = int(right.get('gripper_port', 50054))
    if auto_launch_bool:
        left_arm_ip = right_arm_ip = left_gripper_ip = right_gripper_ip = 'localhost'
    else:
        left_arm_ip = left_gripper_ip = left.get('robot_ip', 'localhost')
        right_arm_ip = right_gripper_ip = right.get('robot_ip', 'localhost')

    package_share_dir = get_package_share_directory('role_ros2')
    urdf_path = os.path.join(package_share_dir, 'robot_ik', 'franka', urdf_file)
    if not os.path.exists(urdf_path):
        raise FileNotFoundError(f"URDF not found: {urdf_path}")

    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    nodes = []

    # ---------- Left arm node ----------
    left_arm_params = [
        {'use_mock': left_use_mock},
        {'ip_address': left_arm_ip},
        {'publish_rate': float(left.get('publish_rate', publish_rate_float))},
        {'namespace': left.get('arm_namespace', 'left_arm')},
        {'auto_reset_on_startup': left.get('auto_reset_on_startup', False)},
        {'auto_reset_delay': float(left.get('auto_reset_delay', 5.0))},
        {'arm_joint_names': left.get('arm_joints', [])},
        {'polymetis_port': left_arm_port},
    ]
    nodes.append(
        GroupAction([
            PushRosNamespace(left.get('arm_namespace', 'left_arm')),
            Node(
                package='role_ros2',
                executable='franka_robot_interface',
                name='franka_robot_interface_node',
                output='screen',
                parameters=left_arm_params,
            ),
        ])
    )

    # ---------- Right arm node ----------
    right_arm_params = [
        {'use_mock': right_use_mock},
        {'ip_address': right_arm_ip},
        {'publish_rate': float(right.get('publish_rate', publish_rate_float))},
        {'namespace': right.get('arm_namespace', 'right_arm')},
        {'auto_reset_on_startup': right.get('auto_reset_on_startup', False)},
        {'auto_reset_delay': float(right.get('auto_reset_delay', 5.0))},
        {'arm_joint_names': right.get('arm_joints', [])},
        {'polymetis_port': right_arm_port},
    ]
    nodes.append(
        GroupAction([
            PushRosNamespace(right.get('arm_namespace', 'right_arm')),
            Node(
                package='role_ros2',
                executable='franka_robot_interface',
                name='franka_robot_interface_node',
                output='screen',
                parameters=right_arm_params,
            ),
        ])
    )

    # ---------- Left gripper node ----------
    if left_load_gripper:
        left_gripper_params = [
            {'use_mock': left_use_mock},
            {'ip_address': left_gripper_ip},
            {'publish_rate': float(left.get('publish_rate', publish_rate_float))},
            {'namespace': left.get('gripper_namespace', 'left_gripper')},
            {'gripper_joint_names': left.get('gripper_joints', [])},
            {'polymetis_port': left_gripper_port},
        ]
        nodes.append(
            GroupAction([
                PushRosNamespace(left.get('gripper_namespace', 'left_gripper')),
                Node(
                    package='role_ros2',
                    executable='franka_gripper_interface',
                    name='franka_gripper_interface_node',
                    output='screen',
                    parameters=left_gripper_params,
                ),
            ])
        )
    # ---------- Right gripper node ----------
    if right_load_gripper:
        right_gripper_params = [
            {'use_mock': right_use_mock},
            {'ip_address': right_gripper_ip},
            {'publish_rate': float(right.get('publish_rate', publish_rate_float))},
            {'namespace': right.get('gripper_namespace', 'right_gripper')},
            {'gripper_joint_names': right.get('gripper_joints', [])},
            {'polymetis_port': right_gripper_port},
        ]
        nodes.append(
            GroupAction([
                PushRosNamespace(right.get('gripper_namespace', 'right_gripper')),
                Node(
                    package='role_ros2',
                    executable='franka_gripper_interface',
                    name='franka_gripper_interface_node',
                    output='screen',
                    parameters=right_gripper_params,
                ),
            ])
        )

    # ---------- Robot state aggregator (all namespaces) ----------
    robot_namespaces = [
        left.get('arm_namespace', 'left_arm'),
        right.get('arm_namespace', 'right_arm'),
    ]
    if left_load_gripper:
        robot_namespaces.append(left.get('gripper_namespace', 'left_gripper'))
    if right_load_gripper:
        robot_namespaces.append(right.get('gripper_namespace', 'right_gripper'))

    agg_timeout = float(left.get('timeout_threshold', right.get('timeout_threshold', 1.0)))
    aggregator_node = Node(
        package='role_ros2',
        executable='robot_state_aggregator',
        name='robot_state_aggregator_node',
        output='screen',
        parameters=[
            {'robot_namespaces': robot_namespaces},
            {'publish_rate': publish_rate_float},
            {'timeout_threshold': agg_timeout},
        ],
    )

    # ---------- Robot state publisher (bimanual URDF) ----------
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    arm_node_delay = 5.0 if auto_launch_bool else 0.0
    gripper_node_delay = 13.0 if auto_launch_bool else 0.0

    arm_nodes = [nodes[0], nodes[1]]
    gripper_node_list = [n for n in nodes[2:] if isinstance(n, GroupAction)]

    if arm_node_delay > 0:
        nodes.clear()
        nodes.append(
            TimerAction(
                period=arm_node_delay,
                actions=arm_nodes + [aggregator_node, robot_state_publisher_node],
            )
        )
        for gripper_ga in gripper_node_list:
            nodes.append(TimerAction(period=gripper_node_delay, actions=[gripper_ga]))
    else:
        nodes.clear()
        nodes.extend(arm_nodes)
        nodes.extend(gripper_node_list)
        nodes.append(aggregator_node)
        nodes.append(robot_state_publisher_node)

    return nodes


def generate_launch_description():
    config = load_config_yaml('role_ros2', 'bimanual_franka_robot_config.yaml')
    left = config.get('left_arm', {})

    def get_default(key, default_value):
        value = left.get(key, default_value)
        if isinstance(value, bool):
            return 'true' if value else 'false'
        return str(value)

    use_mock = LaunchConfiguration('use_mock', default=get_default('use_mock', 'false'))
    publish_rate = LaunchConfiguration('publish_rate', default=get_default('publish_rate', '50.0'))
    left_auto = left.get('auto_launch_controller', False)
    right_auto = config.get('right_arm', {}).get('auto_launch_controller', False)
    auto_launch_default = 'true' if (left_auto or right_auto) else 'false'
    auto_launch_controller = LaunchConfiguration(
        'auto_launch_controller', default=auto_launch_default
    )

    server_processes_opaque = OpaqueFunction(
        function=create_bimanual_server_launch_processes,
        args=[use_mock],
    )

    robot_nodes_opaque = OpaqueFunction(
        function=create_bimanual_nodes,
        args=[use_mock, publish_rate],
    )

    cleanup_on_shutdown = RegisterEventHandler(
        OnShutdown(
            on_shutdown=[
                ExecuteProcess(
                    cmd=['bash', '-c', 'echo "Bimanual: Cleaning up servers..." && bash "%s" && echo "Bimanual: Cleanup complete."' % _get_cleanup_script_path()],
                    output='screen',
                    condition=IfCondition(
                        PythonExpression([
                            "'", use_mock, "' == 'false' and '",
                            auto_launch_controller, "' == 'true'"
                        ])
                    ),
                ),
            ],
        ),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_mock', default_value=get_default('use_mock', 'false'),
                             description='Use mock interfaces (no real robot)'),
        DeclareLaunchArgument('publish_rate', default_value=get_default('publish_rate', '50.0'),
                             description='State publishing rate (Hz)'),
        DeclareLaunchArgument('auto_launch_controller', default_value=auto_launch_default,
                             description='Auto-launch Polymetis servers for both arms (from config)'),
        server_processes_opaque,
        robot_nodes_opaque,
        cleanup_on_shutdown,
    ])
