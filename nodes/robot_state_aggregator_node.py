#!/usr/bin/env python3
"""
Robot State Aggregator Node - Merges multiple robot/gripper states

This node subscribes to multiple robot arm and gripper state topics from
different namespaces and publishes aggregated states for robot_state_publisher
and visualization.

Features:
- Subscribes to /{namespace}/joint_states from each robot arm
- Subscribes to /{namespace}/joint_states from each gripper
- Subscribes to /{namespace}/arm_state from each robot arm
- Subscribes to /{namespace}/gripper_state from each gripper
- Publishes aggregated /joint_states for robot_state_publisher
- Publishes aggregated /robot_state with all arm and gripper states

This enables multi-robot support and compatibility with robot_state_publisher.

Author: Role-ROS2 Team
"""

import threading
from typing import Dict, List, Optional
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import JointState

# Import custom messages (V2)
from role_ros2.msg import (
    ArmState, GripperState, RobotState,
)
# Import legacy messages for backward compatibility
from role_ros2.msg import (
    PolymetisGripperState, PolymetisRobotState
)


class RobotStateAggregatorNode(Node):
    """
    ROS 2 Node that aggregates joint states and robot states from multiple robots.
    
    This node:
    1. Subscribes to /{namespace}/joint_states for each robot arm and gripper
    2. Merges all joint states into a single /joint_states topic
    3. Subscribes to /{namespace}/arm_state and /{namespace}/gripper_state
    4. Publishes aggregated /robot_state with all states
    """
    
    def __init__(self, robot_namespaces: Optional[List[str]] = None):
        """
        Initialize the Robot State Aggregator Node.
        
        Args:
            robot_namespaces: List of robot namespaces to aggregate.
                             If None, uses default ["fr3_arm", "fr3_gripper"]
        """
        super().__init__('robot_state_aggregator_node')
        
        # Declare parameters
        default_namespaces = ['fr3_arm', 'fr3_gripper']
        self.declare_parameter('robot_namespaces', robot_namespaces or default_namespaces)
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('timeout_threshold', 1.0)  # seconds
        
        # Get parameters
        self._robot_namespaces = list(
            self.get_parameter('robot_namespaces').get_parameter_value().string_array_value
        )
        self._publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self._timeout_threshold = self.get_parameter('timeout_threshold').get_parameter_value().double_value
        
        self.get_logger().info(f"Aggregating states from namespaces: {self._robot_namespaces}")
        
        # State storage (thread-safe)
        self._state_lock = threading.Lock()
        self._joint_states: Dict[str, JointState] = {}
        self._arm_states: Dict[str, ArmState] = {}
        self._gripper_states: Dict[str, GripperState] = {}
        self._last_update_times: Dict[str, float] = {}
        
        # Callback group
        self._callback_group = ReentrantCallbackGroup()
        
        # QoS profile for reliable communication
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # ===== Publishers =====
        # Main /joint_states publisher for robot_state_publisher
        self._joint_states_publisher = self.create_publisher(
            JointState, '/joint_states', qos
        )
        
        # V2 robot state publisher
        self._robot_state_publisher = self.create_publisher(
            RobotState, '/robot_state', qos
        )
        
        # Legacy PolymetisRobotState publisher for backward compatibility
        self._legacy_robot_state_publisher = self.create_publisher(
            PolymetisRobotState, '/polymetis/robot_state', qos
        )
        
        # Legacy gripper state publisher for backward compatibility
        self._legacy_gripper_state_publisher = self.create_publisher(
            PolymetisGripperState, '/polymetis/gripper_state', qos
        )
        
        # ===== Subscribers =====
        self._joint_state_subscribers = {}
        self._arm_state_subscribers = {}
        self._gripper_state_subscribers = {}
        
        for namespace in self._robot_namespaces:
            # Subscribe to joint states from each namespace
            topic_name = f'/{namespace}/joint_states' if namespace else '/joint_states'
            self.get_logger().info(f"Subscribing to {topic_name}")
            
            self._joint_state_subscribers[namespace] = self.create_subscription(
                JointState,
                topic_name,
                lambda msg, ns=namespace: self._joint_state_callback(msg, ns),
                qos,
                callback_group=self._callback_group
            )
            
            # Subscribe to arm state if this is an arm namespace
            if 'arm' in namespace.lower():
                arm_topic = f'/{namespace}/arm_state'
                self.get_logger().info(f"Subscribing to {arm_topic}")
                
                self._arm_state_subscribers[namespace] = self.create_subscription(
                    ArmState,
                    arm_topic,
                    lambda msg, ns=namespace: self._arm_state_callback(msg, ns),
                    qos,
                    callback_group=self._callback_group
                )
            
            # Subscribe to gripper state if this is a gripper namespace
            if 'gripper' in namespace.lower():
                gripper_topic = f'/{namespace}/gripper_state'
                self.get_logger().info(f"Subscribing to {gripper_topic}")
                
                self._gripper_state_subscribers[namespace] = self.create_subscription(
                    GripperState,
                    gripper_topic,
                    lambda msg, ns=namespace: self._gripper_state_callback(msg, ns),
                    qos,
                    callback_group=self._callback_group
                )
        
        # Timer for publishing aggregated states
        timer_period = 1.0 / self._publish_rate
        self._publish_timer = self.create_timer(
            timer_period,
            self._publish_aggregated_states,
            callback_group=self._callback_group
        )
        
        self.get_logger().info(
            f'RobotStateAggregatorNode initialized. '
            f'Publishing aggregated states at {self._publish_rate} Hz'
        )
    
    def _joint_state_callback(self, msg: JointState, namespace: str):
        """Callback for joint state messages from a specific namespace."""
        with self._state_lock:
            self._joint_states[namespace] = msg
            self._last_update_times[f'{namespace}_joints'] = self.get_clock().now().nanoseconds / 1e9
    
    def _arm_state_callback(self, msg: ArmState, namespace: str):
        """Callback for arm state messages from a specific namespace."""
        with self._state_lock:
            self._arm_states[namespace] = msg
            self._last_update_times[f'{namespace}_arm'] = self.get_clock().now().nanoseconds / 1e9
    
    def _gripper_state_callback(self, msg: GripperState, namespace: str):
        """Callback for gripper state messages from a specific namespace."""
        with self._state_lock:
            self._gripper_states[namespace] = msg
            self._last_update_times[f'{namespace}_gripper'] = self.get_clock().now().nanoseconds / 1e9
    
    def _publish_aggregated_states(self):
        """Publish aggregated joint states and robot states."""
        now = self.get_clock().now()
        current_time = now.nanoseconds / 1e9
        
        with self._state_lock:
            # Aggregate joint states
            all_joint_names = []
            all_positions = []
            all_velocities = []
            all_efforts = []
            
            for namespace in self._robot_namespaces:
                if namespace in self._joint_states:
                    js = self._joint_states[namespace]
                    
                    # Check if data is stale
                    last_update = self._last_update_times.get(f'{namespace}_joints', 0)
                    if current_time - last_update > self._timeout_threshold:
                        self.get_logger().debug(f"Stale joint data from {namespace}")
                        continue
                    
                    all_joint_names.extend(list(js.name))
                    all_positions.extend(list(js.position))
                    all_velocities.extend(list(js.velocity) if js.velocity else [0.0] * len(js.name))
                    all_efforts.extend(list(js.effort) if js.effort else [0.0] * len(js.name))
            
            # Publish aggregated joint states
            if all_joint_names:
                aggregated_js = JointState()
                aggregated_js.header.stamp = now.to_msg()
                aggregated_js.header.frame_id = 'base_link'
                aggregated_js.name = all_joint_names
                aggregated_js.position = all_positions
                aggregated_js.velocity = all_velocities
                aggregated_js.effort = all_efforts
                
                self._joint_states_publisher.publish(aggregated_js)
            
            # Publish V2 robot state
            robot_state = RobotState()
            robot_state.header.stamp = now.to_msg()
            robot_state.header.frame_id = 'base_link'
            robot_state.num_robots = len(self._robot_namespaces)
            robot_state.robot_namespaces = self._robot_namespaces
            robot_state.joint_positions = all_positions
            robot_state.joint_velocities = all_velocities
            robot_state.joint_names = all_joint_names
            
            # Add arm states
            arm_states_list = []
            for namespace in self._robot_namespaces:
                if namespace in self._arm_states:
                    arm_states_list.append(self._arm_states[namespace])
            robot_state.arm_states = arm_states_list
            
            # Add gripper states
            gripper_states_list = []
            for namespace in self._robot_namespaces:
                if namespace in self._gripper_states:
                    gripper_states_list.append(self._gripper_states[namespace])
            robot_state.gripper_states = gripper_states_list
            
            self._robot_state_publisher.publish(robot_state)
            
            # Publish legacy PolymetisRobotState for backward compatibility
            # Use first arm state if available
            if self._arm_states and self._gripper_states:
                self._publish_legacy_robot_state()
    
    def _publish_legacy_robot_state(self):
        """Publish legacy PolymetisRobotState for backward compatibility."""
        now = self.get_clock().now()
        
        # Get first arm state
        arm_state = None
        for namespace in self._robot_namespaces:
            if namespace in self._arm_states:
                arm_state = self._arm_states[namespace]
                break
        
        # Get first gripper state
        gripper_state = None
        for namespace in self._robot_namespaces:
            if namespace in self._gripper_states:
                gripper_state = self._gripper_states[namespace]
                break
        
        if arm_state is None:
            return
        
        # Create legacy PolymetisRobotState message
        msg = PolymetisRobotState()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = 'base_link'
        
        # Copy arm state fields
        msg.joint_positions = list(arm_state.joint_positions)
        msg.joint_velocities = list(arm_state.joint_velocities)
        msg.joint_torques_computed = list(arm_state.joint_torques_computed)
        msg.prev_joint_torques_computed = list(arm_state.prev_joint_torques_computed)
        msg.prev_joint_torques_computed_safened = list(arm_state.prev_joint_torques_computed_safened)
        msg.motor_torques_measured = list(arm_state.motor_torques_measured)
        msg.ee_position = list(arm_state.ee_position)
        msg.ee_quaternion = list(arm_state.ee_quaternion)
        msg.ee_euler = list(arm_state.ee_euler)
        msg.prev_controller_latency_ms = arm_state.prev_controller_latency_ms
        msg.prev_command_successful = arm_state.prev_command_successful
        msg.is_running_policy = arm_state.is_running_policy
        msg.polymetis_timestamp_ns = arm_state.polymetis_timestamp_ns
        
        # Copy gripper state fields if available
        if gripper_state is not None:
            msg.gripper_width = gripper_state.width
            msg.gripper_position = gripper_state.position
            msg.gripper_is_grasped = gripper_state.is_grasped
            msg.gripper_is_moving = gripper_state.is_moving
            msg.gripper_prev_command_successful = gripper_state.prev_command_successful
            msg.gripper_error_code = gripper_state.error_code
        else:
            msg.gripper_width = 0.0
            msg.gripper_position = 0.0
            msg.gripper_is_grasped = False
            msg.gripper_is_moving = False
            msg.gripper_prev_command_successful = True
            msg.gripper_error_code = 0
        
        self._legacy_robot_state_publisher.publish(msg)
        
        # Also publish legacy gripper state (convert from GripperState to PolymetisGripperState)
        if gripper_state is not None:
            legacy_gripper_msg = PolymetisGripperState()
            legacy_gripper_msg.header = gripper_state.header
            legacy_gripper_msg.width = gripper_state.width
            legacy_gripper_msg.position = gripper_state.position
            legacy_gripper_msg.is_grasped = gripper_state.is_grasped
            legacy_gripper_msg.is_moving = gripper_state.is_moving
            legacy_gripper_msg.prev_command_successful = gripper_state.prev_command_successful
            legacy_gripper_msg.error_code = gripper_state.error_code
            legacy_gripper_msg.max_width = gripper_state.max_width
            legacy_gripper_msg.polymetis_timestamp_ns = gripper_state.timestamp_ns
            self._legacy_gripper_state_publisher.publish(legacy_gripper_msg)
    
    def destroy_node(self):
        """Clean up resources on shutdown."""
        super().destroy_node()


def main(args=None):
    """Main function to run the Robot State Aggregator Node."""
    from rclpy.executors import MultiThreadedExecutor
    import os
    
    rclpy.init(args=args)
    
    # Get namespaces from environment or use defaults
    namespaces_str = os.environ.get('ROBOT_NAMESPACES', 'fr3_arm,fr3_gripper')
    namespaces = [ns.strip() for ns in namespaces_str.split(',') if ns.strip()]
    
    node = RobotStateAggregatorNode(robot_namespaces=namespaces)
    
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception as e:
            print(f"Error destroying node: {e}")
        finally:
            rclpy.shutdown()


if __name__ == '__main__':
    main()

