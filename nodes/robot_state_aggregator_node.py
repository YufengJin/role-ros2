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

import copy
import threading
from typing import Dict, List, Optional
import numpy as np
from scipy.spatial.transform import Rotation as R

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import JointState

import tf2_ros
from tf2_ros import TransformException as TF2TransformException

# Import custom messages (V2)
from role_ros2.msg import (
    ArmState, GripperState, RobotState,
)

try:
    from role_ros2.misc.transformations import quat_to_euler
    QUAT_TO_EULER_AVAILABLE = True
except ImportError:
    QUAT_TO_EULER_AVAILABLE = False


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
        self.declare_parameter('target_ee_frame_id', 'base_link')

        # Get parameters
        self._robot_namespaces = list(
            self.get_parameter('robot_namespaces').get_parameter_value().string_array_value
        )
        self._publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self._timeout_threshold = self.get_parameter('timeout_threshold').get_parameter_value().double_value
        self._target_ee_frame_id = self.get_parameter(
            'target_ee_frame_id'
        ).get_parameter_value().string_value

        # TF for ee_pose transform (source frame -> target_ee_frame_id, e.g. base_link)
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._cached_transforms: Dict[str, object] = {}  # frame_id -> TransformStamped

        self.get_logger().info(
            f"Aggregating states from namespaces: {self._robot_namespaces}, "
            f"target_ee_frame_id: {self._target_ee_frame_id}"
        )
        
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

    def _get_transform_to_target(self, source_frame_id: str):
        """
        Get cached transform from source_frame_id to target_ee_frame_id.
        Returns None if frames are the same or transform lookup fails.
        """
        if source_frame_id == self._target_ee_frame_id:
            return None
        if source_frame_id in self._cached_transforms:
            return self._cached_transforms[source_frame_id]
        try:
            trans = self._tf_buffer.lookup_transform(
                self._target_ee_frame_id,
                source_frame_id,
                rclpy.time.Time(),
            )
            self._cached_transforms[source_frame_id] = trans
            return trans
        except TF2TransformException:
            return None

    def _transform_arm_state_ee_to_target(self, arm_state: ArmState) -> ArmState:
        """
        Transform arm_state ee_pose from header.frame_id to target_ee_frame_id.
        Returns a new ArmState (does not modify the original).
        Compatible with single arm (frame_id already base_link) and bimanual.
        """
        if arm_state.header.frame_id == self._target_ee_frame_id:
            return arm_state
        trans = self._get_transform_to_target(arm_state.header.frame_id)
        if trans is None:
            return arm_state
        # Copy arm_state to avoid mutating cached data
        out = copy.deepcopy(arm_state)
        pos = np.array(arm_state.ee_position, dtype=np.float64)
        quat = np.array(arm_state.ee_quaternion, dtype=np.float64)
        t = trans.transform.translation
        r = trans.transform.rotation
        t_vec = np.array([t.x, t.y, t.z])
        r_rot = R.from_quat([r.x, r.y, r.z, r.w])
        pos_new = r_rot.apply(pos) + t_vec
        r_pose = R.from_quat(quat)
        quat_new = (r_rot * r_pose).as_quat()
        out.ee_position = pos_new.tolist()
        out.ee_quaternion = quat_new.tolist()
        if QUAT_TO_EULER_AVAILABLE and len(arm_state.ee_euler) >= 3:
            out.ee_euler = quat_to_euler(quat_new).tolist()
        out.header.frame_id = self._target_ee_frame_id
        return out

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
            
            # Add arm states (transform ee_pose to target_ee_frame_id when needed)
            arm_states_list = []
            for namespace in self._robot_namespaces:
                if namespace in self._arm_states:
                    arm_state = self._transform_arm_state_ee_to_target(
                        self._arm_states[namespace]
                    )
                    arm_states_list.append(arm_state)
            robot_state.arm_states = arm_states_list
            
            # Add gripper states
            gripper_states_list = []
            for namespace in self._robot_namespaces:
                if namespace in self._gripper_states:
                    gripper_states_list.append(self._gripper_states[namespace])
            robot_state.gripper_states = gripper_states_list
            
            self._robot_state_publisher.publish(robot_state)

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

