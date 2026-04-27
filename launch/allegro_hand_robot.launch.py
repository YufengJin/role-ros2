#!/usr/bin/env python3
"""
Allegro Hand V4 Launch File

Brings up the official allegro_hand_ros2 stack (controller_manager + URDF +
ros2_controllers) and the role-ros2 wrapper that adapts it to role-ros2
conventions (namespaced topics, custom messages, blocking services).

Usage:
    ros2 launch role_ros2 allegro_hand_robot.launch.py
    ros2 launch role_ros2 allegro_hand_robot.launch.py use_mock:=true
    ros2 launch role_ros2 allegro_hand_robot.launch.py namespace:=right_hand hand_side:=right
"""

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def load_config_yaml(package_name, config_file):
    """Load launch configuration YAML from the package's share/config dir."""
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


def create_allegro_nodes(context: LaunchContext, use_mock, namespace, hand_side,
                         device, publish_rate, auto_home_on_startup, auto_home_delay):
    """Resolve LaunchConfigurations and instantiate the wrapper + aggregator."""
    use_mock_str = context.perform_substitution(use_mock)
    namespace_str = context.perform_substitution(namespace)
    hand_side_str = context.perform_substitution(hand_side)
    device_str = context.perform_substitution(device)
    publish_rate_str = context.perform_substitution(publish_rate)
    auto_home_str = context.perform_substitution(auto_home_on_startup)
    auto_home_delay_str = context.perform_substitution(auto_home_delay)

    use_mock_bool = use_mock_str.lower() == 'true'
    auto_home_bool = auto_home_str.lower() == 'true'
    publish_rate_float = float(publish_rate_str)
    auto_home_delay_float = float(auto_home_delay_str)

    config = load_config_yaml('role_ros2', 'allegro_hand_config.yaml')
    hand_joint_names = config.get('hand_joint_names', [])
    if not hand_joint_names:
        raise ValueError(
            "allegro_hand_config.yaml is missing 'hand_joint_names'. "
            "Did you forget to colcon build?"
        )
    prefix = config.get('prefix', 'ah_')
    home_joints = config.get('home_joints', [0.0] * len(hand_joint_names))
    move_tolerance = float(config.get('move_tolerance', 0.02))
    move_timeout = float(config.get('move_timeout', 5.0))

    hardware_type = 'mock_components' if use_mock_bool else 'physical_device'

    print(
        f"Info: allegro launch — use_mock={use_mock_bool}, "
        f"namespace={namespace_str}, hand_side={hand_side_str}, device={device_str}, "
        f"hardware={hardware_type}, n_joints={len(hand_joint_names)}"
    )

    # 1) Include the official Wonik bringup. It owns the controller_manager,
    #    the URDF/robot_state_publisher, joint_state_broadcaster, and the
    #    forward / posture / grasp controllers. We only override the hardware
    #    type so use_mock:=true stays purely a launch-arg toggle.
    official_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('allegro_hand_bringup'), 'launch', 'allegro_hand.launch.py',
            ])
        ),
        launch_arguments={
            'device': device_str,
            'hand': hand_side_str,
            'ros2_control_hardware_type': hardware_type,
            'use_sim_time': 'false',
        }.items(),
    )

    # 2) role-ros2 wrapper inside our namespace
    wrapper = GroupAction([
        PushRosNamespace(namespace_str),
        Node(
            package='role_ros2',
            executable='allegro_hand_interface',
            name='allegro_hand_interface_node',
            output='screen',
            parameters=[{
                'use_mock': use_mock_bool,
                'namespace': namespace_str,
                'hand_joint_names': hand_joint_names,
                'prefix': prefix,
                'publish_rate': publish_rate_float,
                'home_joints': [float(v) for v in home_joints],
                'move_tolerance': move_tolerance,
                'move_timeout': move_timeout,
                'auto_home_on_startup': auto_home_bool,
                'auto_home_delay': auto_home_delay_float,
                'official_joint_states_topic': '/joint_states',
                'official_position_command_topic': '/allegro_hand_position_controller/commands',
                'official_posture_action': '/allegro_hand_posture_controller/grasp_cmd',
                'official_grasp_action': '/allegro_hand_grasp_controller/grasp_cmd',
            }],
        ),
    ])

    # 3) Reuse the existing aggregator so /{ns}/joint_states is also visible
    #    on root /joint_states for downstream RobotEnv consumers.
    aggregator = Node(
        package='role_ros2',
        executable='robot_state_aggregator',
        name='robot_state_aggregator_node',
        output='screen',
        parameters=[
            {'robot_namespaces': [namespace_str]},
            {'publish_rate': publish_rate_float},
            {'timeout_threshold': 1.0},
        ],
    )

    return [official_launch, wrapper, aggregator]


def generate_launch_description():
    config = load_config_yaml('role_ros2', 'allegro_hand_config.yaml')

    use_mock_arg = DeclareLaunchArgument(
        'use_mock',
        default_value=str(config.get('use_mock', False)).lower(),
        description='Use ros2_control mock_components instead of the real CAN driver.',
    )
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value=config.get('namespace', 'allegro_hand'),
        description='ROS namespace for the role-ros2 wrapper.',
    )
    hand_side_arg = DeclareLaunchArgument(
        'hand_side',
        default_value=config.get('hand_side', 'right'),
        description='Which Allegro hand to launch: right | left.',
    )
    device_arg = DeclareLaunchArgument(
        'device',
        default_value=config.get('device', 'v4'),
        description='Allegro hand device version (currently only v4 is supported).',
    )
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value=str(config.get('publish_rate', 50.0)),
        description='Wrapper state publish rate (Hz).',
    )
    auto_home_arg = DeclareLaunchArgument(
        'auto_home_on_startup',
        default_value=str(config.get('auto_home_on_startup', False)).lower(),
        description='Drive hand to posture "home" after delay on startup.',
    )
    auto_home_delay_arg = DeclareLaunchArgument(
        'auto_home_delay',
        default_value=str(config.get('auto_home_delay', 5.0)),
        description='Seconds to wait before auto-home, if enabled.',
    )

    return LaunchDescription([
        use_mock_arg,
        namespace_arg,
        hand_side_arg,
        device_arg,
        publish_rate_arg,
        auto_home_arg,
        auto_home_delay_arg,
        OpaqueFunction(
            function=create_allegro_nodes,
            kwargs={
                'use_mock': LaunchConfiguration('use_mock'),
                'namespace': LaunchConfiguration('namespace'),
                'hand_side': LaunchConfiguration('hand_side'),
                'device': LaunchConfiguration('device'),
                'publish_rate': LaunchConfiguration('publish_rate'),
                'auto_home_on_startup': LaunchConfiguration('auto_home_on_startup'),
                'auto_home_delay': LaunchConfiguration('auto_home_delay'),
            },
        ),
    ])
