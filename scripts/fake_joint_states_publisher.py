#!/usr/bin/env python3
"""
Fake Joint States Publisher Node

This node publishes fake joint states for testing robot_state_publisher independently.
It's a lightweight alternative to polymetis_bridge when you only need joint_states
without the full Polymetis functionality.

Features:
- Publishes /joint_states at configurable rate
- Loads joint names from config file based on arm_id
- Generates simple sinusoidal motion for visualization
- Supports both arm and gripper joints
"""

import time
import math
import sys
import yaml
from pathlib import Path
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


def load_joint_names_from_config(config_file=None, arm_id=None):
    """
    Load joint names from config/franka_robot_config.yaml.
    
    Args:
        config_file: Path to config file. If None, use ROS2 package share directory.
        arm_id: Arm ID to filter joints (optional, for validation)
    
    Returns:
        tuple: (arm_joint_names, gripper_joint_names)
    
    Raises:
        FileNotFoundError: If config file not found
        ValueError: If joint names are missing or don't match arm_id format
    """
    if config_file is None:
        # Use ROS2 package share directory
        try:
            from ament_index_python.packages import get_package_share_directory
            package_share_dir = get_package_share_directory('role_ros2')
            config_file = Path(package_share_dir) / 'config' / 'franka_robot_config.yaml'
        except Exception as e:
            raise FileNotFoundError(
                f"Failed to get package share directory: {e}\n"
                f"Please ensure ROS2 workspace is built and sourced."
            )
    else:
        config_file = Path(config_file)
    
    # Check file exists
    if not config_file.exists():
        raise FileNotFoundError(
            f"Config file not found: {config_file}\n"
            f"Please ensure franka_robot_config.yaml exists in config directory."
        )
    
    # Read config file
    try:
        with open(config_file, 'r') as f:
            config = yaml.safe_load(f)
    except Exception as e:
        raise ValueError(f"Failed to parse config file {config_file}: {e}")
    
    # Validate config is not None
    if config is None:
        raise ValueError(f"Config file {config_file} is empty or invalid YAML")
    
    # Extract joint names
    arm_joints = config.get('arm_joints', [])
    gripper_joints = config.get('gripper_joints', [])
    config_arm_id = config.get('arm_id', '')
    
    # Validate types
    if not isinstance(arm_joints, list):
        raise ValueError(
            f"arm_joints must be a list, got {type(arm_joints).__name__}: {arm_joints}"
        )
    if not isinstance(gripper_joints, list):
        raise ValueError(
            f"gripper_joints must be a list, got {type(gripper_joints).__name__}: {gripper_joints}"
        )
    
    # Validate
    if not arm_joints:
        raise ValueError("arm_joints is empty or missing in franka_robot_config.yaml")
    
    if not gripper_joints:
        raise ValueError("gripper_joints is empty or missing in franka_robot_config.yaml")
    
    # If arm_id is provided, validate joint names match
    if arm_id:
        for joint in arm_joints:
            if not isinstance(joint, str):
                raise ValueError(
                    f"Joint name must be a string, got {type(joint).__name__}: {joint}"
                )
            if not joint.startswith(f'{arm_id}_'):
                raise ValueError(
                    f"Joint name '{joint}' does not match arm_id '{arm_id}'. "
                    f"Expected format: '{arm_id}_panda_jointX'"
                )
        
        for joint in gripper_joints:
            if not isinstance(joint, str):
                raise ValueError(
                    f"Gripper joint name must be a string, got {type(joint).__name__}: {joint}"
                )
            if not joint.startswith(f'{arm_id}_'):
                raise ValueError(
                    f"Gripper joint name '{joint}' does not match arm_id '{arm_id}'. "
                    f"Expected format: '{arm_id}_panda_finger_jointX'"
                )
    elif config_arm_id:
        # Use arm_id from config if not provided
        arm_id = config_arm_id
    
    return arm_joints, gripper_joints


class FakeJointStatesPublisher(Node):
    """
    ROS 2 Node that publishes fake joint states for testing.
    
    This node:
    - Publishes /joint_states at configurable rate
    - Generates simple sinusoidal motion for visualization
    - Supports both arm and gripper joints
    """
    
    def __init__(self):
        """Initialize the Fake Joint States Publisher Node."""
        super().__init__('fake_joint_states_publisher')
        
        # Declare parameters
        self.declare_parameter('arm_id', 'fr3')
        self.declare_parameter('publish_rate', 50.0)  # Hz
        
        # Get parameters
        arm_id = self.get_parameter('arm_id').get_parameter_value().string_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        
        # Load joint names from config file
        try:
            arm_joints, gripper_joints = load_joint_names_from_config(arm_id=arm_id)
            self.arm_joint_names = arm_joints
            self.gripper_joint_names = gripper_joints
            self.all_joint_names = self.arm_joint_names + self.gripper_joint_names
            self.get_logger().info(
                f"Loaded joint names from config: {len(self.arm_joint_names)} arm, "
                f"{len(self.gripper_joint_names)} gripper"
            )
            self.get_logger().info(f"Arm joints: {self.arm_joint_names}")
            self.get_logger().info(f"Gripper joints: {self.gripper_joint_names}")
        except (FileNotFoundError, ValueError) as e:
            error_msg = f"Failed to load joint names: {e}"
            print(f"[ERROR] {error_msg}", file=sys.stderr)
            self.get_logger().error(error_msg)
            raise  # Re-raise error to prevent node from starting with invalid config
        
        # Publisher for joint states
        self._joint_state_publisher = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )
        
        # State for generating fake motion
        self._start_time = time.time()
        self._gripper_width = 0.08  # Max width in meters (default for Franka)
        
        # Timer for publishing joint states
        timer_period = 1.0 / self.publish_rate  # Convert Hz to seconds
        self._state_timer = self.create_timer(timer_period, self._publish_joint_states)
        
        self.get_logger().info(
            f'FakeJointStatesPublisher initialized. Publishing at {self.publish_rate} Hz'
        )
        self.get_logger().info(f'Publishing to: /joint_states')
        self.get_logger().info(f'Total joints: {len(self.all_joint_names)}')
    
    def _publish_joint_states(self):
        """
        Publish fake joint states at high frequency.
        
        Generates simple sinusoidal motion for visualization.
        """
        try:
            # Calculate time since start
            elapsed_time = time.time() - self._start_time
            
            # Generate fake joint positions (sinusoidal motion)
            # Each joint has a different frequency and amplitude for interesting motion
            arm_positions = []
            for i, joint_name in enumerate(self.arm_joint_names):
                # Different frequency for each joint (0.1 to 0.5 Hz)
                frequency = 0.1 + (i * 0.05)
                # Different amplitude for each joint (0.2 to 0.8 radians)
                amplitude = 0.2 + (i * 0.1)
                # Different phase offset for each joint
                phase = i * 0.5
                # Generate sinusoidal position
                position = amplitude * math.sin(2 * math.pi * frequency * elapsed_time + phase)
                arm_positions.append(position)
            
            # Generate fake gripper positions
            # Simple open/close motion
            gripper_frequency = 0.1  # 0.1 Hz (10 second period)
            gripper_width = 0.04 + 0.04 * math.sin(2 * math.pi * gripper_frequency * elapsed_time)
            self._gripper_width = max(0.0, min(gripper_width, 0.08))
            
            # Calculate gripper finger positions (each finger moves half the width)
            finger1_position = self._gripper_width / 2.0
            finger2_position = self._gripper_width / 2.0
            
            # Generate fake velocities (derivative of position)
            # For simplicity, use small random-like values
            arm_velocities = []
            for i in range(len(self.arm_joint_names)):
                frequency = 0.1 + (i * 0.05)
                amplitude = 0.2 + (i * 0.1)
                phase = i * 0.5
                # Velocity is derivative: d/dt(sin) = cos
                velocity = amplitude * 2 * math.pi * frequency * math.cos(
                    2 * math.pi * frequency * elapsed_time + phase
                )
                arm_velocities.append(velocity)
            
            # Create JointState message
            joint_state_msg = JointState()
            
            # Set header with proper timestamp
            now = self.get_clock().now()
            joint_state_msg.header.stamp = now.to_msg()
            joint_state_msg.header.frame_id = 'base_link'
            
            # Set joint names (arm + gripper)
            joint_state_msg.name = list(self.all_joint_names)
            
            # Set joint positions (arm + gripper)
            joint_state_msg.position = arm_positions + [finger1_position, finger2_position]
            
            # Set joint velocities (arm + gripper)
            joint_state_msg.velocity = arm_velocities + [0.0, 0.0]  # Gripper velocities
            
            # Set joint efforts (torques) - all zeros for fake data
            joint_state_msg.effort = [0.0] * len(self.all_joint_names)
            
            # Publish
            self._joint_state_publisher.publish(joint_state_msg)
            
        except Exception as e:
            self.get_logger().error(f"Error publishing joint states: {e}")


def main(args=None):
    """Main function to run the Fake Joint States Publisher Node."""
    rclpy.init(args=args)
    
    try:
        # Create node
        node = FakeJointStatesPublisher()
        
        # Spin node
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"[ERROR] Failed to start node: {e}", file=sys.stderr)
        sys.exit(1)
    finally:
        try:
            node.destroy_node()
        except:
            pass
        finally:
            rclpy.shutdown()


if __name__ == '__main__':
    main()


