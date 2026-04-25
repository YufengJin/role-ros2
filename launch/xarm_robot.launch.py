#!/usr/bin/env python3
"""
xArm Robot Launch File

Starts three nodes (mirroring launch/franka_robot.launch.py):
  - xarm_robot_interface_node  (in /{arm_namespace})  — publishes joint_states,
                                                        arm_state, ee_pose,
                                                        controller_status
  - robot_state_aggregator_node (root)  — merges /{arm_namespace}/joint_states
                                          into root /joint_states
  - robot_state_publisher       (root)  — reads URDF + /joint_states, publishes /tf

Usage:
    ros2 launch role_ros2 xarm_robot.launch.py
    ros2 launch role_ros2 xarm_robot.launch.py use_mock:=true
    ros2 launch role_ros2 xarm_robot.launch.py arm_namespace:=robot1_arm
"""

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def load_config_yaml(package_name, config_file):
    """Load launch configuration YAML file from the package's share dir."""
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


def create_xarm_node(context: LaunchContext, use_mock, arm_namespace,
                     robot_ip, publish_rate, auto_reset_on_startup,
                     auto_reset_delay):
    """OpaqueFunction body — resolves substitutions and builds the Node."""
    use_mock_str = context.perform_substitution(use_mock)
    arm_namespace_str = context.perform_substitution(arm_namespace)
    robot_ip_str = context.perform_substitution(robot_ip)
    publish_rate_str = context.perform_substitution(publish_rate)
    auto_reset_str = context.perform_substitution(auto_reset_on_startup)
    auto_reset_delay_str = context.perform_substitution(auto_reset_delay)

    use_mock_bool = use_mock_str.lower() == 'true'
    auto_reset_bool = auto_reset_str.lower() == 'true'
    publish_rate_float = float(publish_rate_str)
    auto_reset_delay_float = float(auto_reset_delay_str)

    config = load_config_yaml('role_ros2', 'xarm_robot_config.yaml')
    arm_joints = config.get('arm_joints', [])
    if not arm_joints:
        raise ValueError(
            "xarm_robot_config.yaml is missing 'arm_joints'. "
            "Did you forget to colcon build?"
        )
    ee_frame_id = config.get('ee_frame_id', 'xarm6_link_base')
    home_joints = config.get('home_joints', [0.0] * len(arm_joints))
    max_joint_velocity = float(config.get('max_joint_velocity', 1.0))

    # Load URDF for robot_state_publisher (TF tree).
    urdf_filepath = os.path.join(
        get_package_share_directory('role_ros2'),
        'robot_ik', 'xarm',
        config.get('urdf_file', 'xarm6.urdf'),
    )
    if not os.path.exists(urdf_filepath):
        raise FileNotFoundError(
            f"URDF not found: {urdf_filepath}. "
            "Did you run colcon build after vendoring xarm6.urdf?"
        )
    with open(urdf_filepath, 'r') as f:
        robot_description = f.read()
    print(f"Info: loaded xArm URDF from {urdf_filepath} ({len(robot_description)} bytes)")

    # Allow the launch arg to override the yaml value when explicitly passed.
    if not robot_ip_str:
        robot_ip_str = config.get('robot_ip', '192.168.1.185')

    print(
        f"Info: xarm launch — use_mock={use_mock_bool}, "
        f"namespace={arm_namespace_str}, robot_ip={robot_ip_str}, "
        f"n_dof={len(arm_joints)}"
    )

    arm_node_params = [
        {'use_mock': use_mock_bool},
        {'ip_address': robot_ip_str},
        {'publish_rate': publish_rate_float},
        {'namespace': arm_namespace_str},
        {'arm_joint_names': arm_joints},
        {'ee_frame_id': ee_frame_id},
        {'home_joints': home_joints},
        {'max_joint_velocity': max_joint_velocity},
        {'auto_reset_on_startup': auto_reset_bool},
        {'auto_reset_delay': auto_reset_delay_float},
    ]

    arm_node = GroupAction([
        PushRosNamespace(arm_namespace_str),
        Node(
            package='role_ros2',
            executable='xarm_robot_interface',
            name='xarm_robot_interface_node',
            output='screen',
            parameters=arm_node_params,
        ),
    ])

    # Root-level aggregator: merge /{arm_namespace}/joint_states -> /joint_states
    aggregator_node = Node(
        package='role_ros2',
        executable='robot_state_aggregator',
        name='robot_state_aggregator_node',
        output='screen',
        parameters=[
            {'robot_namespaces': [arm_namespace_str]},
            {'publish_rate': publish_rate_float},
            {'timeout_threshold': 1.0},
        ],
    )

    # Root-level robot_state_publisher: read /joint_states, publish /tf
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    return [arm_node, aggregator_node, robot_state_publisher_node]


def generate_launch_description():
    config = load_config_yaml('role_ros2', 'xarm_robot_config.yaml')

    use_mock_arg = DeclareLaunchArgument(
        'use_mock',
        default_value=str(config.get('use_mock', False)).lower(),
        description='Use MockXArmInterface (no hardware connection).',
    )
    arm_namespace_arg = DeclareLaunchArgument(
        'arm_namespace',
        default_value=config.get('arm_namespace', 'xarm6_arm'),
        description='ROS namespace for the xArm node.',
    )
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value=config.get('robot_ip', '192.168.1.185'),
        description='IP address of the xArm controller box.',
    )
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value=str(config.get('publish_rate', 50.0)),
        description='State publish rate (Hz).',
    )
    auto_reset_on_startup_arg = DeclareLaunchArgument(
        'auto_reset_on_startup',
        default_value=str(config.get('auto_reset_on_startup', False)).lower(),
        description='Drive robot to home_joints on startup.',
    )
    auto_reset_delay_arg = DeclareLaunchArgument(
        'auto_reset_delay',
        default_value=str(config.get('auto_reset_delay', 5.0)),
        description='Delay (s) before auto-reset, if enabled.',
    )

    return LaunchDescription([
        use_mock_arg,
        arm_namespace_arg,
        robot_ip_arg,
        publish_rate_arg,
        auto_reset_on_startup_arg,
        auto_reset_delay_arg,
        OpaqueFunction(
            function=create_xarm_node,
            kwargs={
                'use_mock': LaunchConfiguration('use_mock'),
                'arm_namespace': LaunchConfiguration('arm_namespace'),
                'robot_ip': LaunchConfiguration('robot_ip'),
                'publish_rate': LaunchConfiguration('publish_rate'),
                'auto_reset_on_startup': LaunchConfiguration('auto_reset_on_startup'),
                'auto_reset_delay': LaunchConfiguration('auto_reset_delay'),
            },
        ),
    ])
