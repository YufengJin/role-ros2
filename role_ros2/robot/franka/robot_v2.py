# ROBOT SPECIFIC IMPORTS
"""
FrankaRobotV2 - ROS2 Interface for V2 Franka Robot Nodes

This class provides a Python interface for the V2 architecture which separates
arm and gripper into different ROS2 nodes:
- franka_robot_interface_node: Arm control (namespace: /{arm_namespace})
- franka_gripper_interface_node: Gripper control (namespace: /{gripper_namespace})

Key Differences from V1 (robot.py):
- Separate arm and gripper subscriptions and publishers
- Namespace-based topics: /{namespace}/joint_states, /{namespace}/arm_state, etc.
- Separate control methods for arm and gripper
- Support for multiple robots via different namespaces

Usage:
    # Default namespaces
    robot = FrankaRobotV2()
    
    # Custom namespaces
    robot = FrankaRobotV2(arm_namespace='robot1_arm', gripper_namespace='robot1_gripper')
    
    # Using shared node
    robot = FrankaRobotV2(node=existing_node)

Author: Role-ROS2 Team
"""

import time
import threading
from collections import deque
from typing import Optional, Tuple, List, Dict, Any

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# ROS2 message imports (V2 messages)
from role_ros2.msg import (
    ArmState, GripperState, ControllerStatus,
    JointPositionCommand, JointVelocityCommand,
    CartesianPositionCommand, CartesianVelocityCommand,
    GripperCommand
)
from role_ros2.srv import (
    Reset, StartCartesianImpedance, StartJointImpedance, StartJointVelocity,
    TerminatePolicy, MoveToJointPositions, MoveToEEPose,
    SolveIK, ComputeFK, ComputeTimeToGo,
    GripperGoto, GripperGrasp
)
from std_srvs.srv import Trigger

# UTILITY SPECIFIC IMPORTS
from role_ros2.misc.transformations import add_poses, euler_to_quat, pose_diff, quat_to_euler
from role_ros2.misc.ros2_utils import get_ros_time_ns
from role_ros2.robot_ik.robot_ik_solver import RobotIKSolver
from role_ros2.robot.base_robot import BaseRobot


class FrankaRobotV2(BaseRobot):
    """
    ROS2-based robot interface for V2 architecture (separate arm and gripper nodes).
    
    This class communicates with:
    - franka_robot_interface_node for arm control
    - franka_gripper_interface_node for gripper control
    
    Key Features:
    - Separate arm and gripper state subscriptions
    - Namespace-based topics for multi-robot support
    - Separate command publishers for different control modes
    - Full service support for blocking operations
    """
    
    def __init__(
        self,
        node: Optional[Node] = None,
        arm_namespace: str = "fr3_arm",
        gripper_namespace: str = "fr3_gripper"
    ):
        """
        Initialize the V2 robot interface.
        
        Args:
            node: Optional ROS2 node. If None, creates a new node.
            arm_namespace: Namespace for arm interface node (default: "fr3_arm")
            gripper_namespace: Namespace for gripper interface node (default: "fr3_gripper")
        """
        self._arm_namespace = arm_namespace
        self._gripper_namespace = gripper_namespace
        
        # Create or use provided node
        if node is None:
            if not rclpy.ok():
                rclpy.init()
            self._node = Node('franka_robot_v2_interface')
            self._own_node = True
        else:
            self._node = node
            self._own_node = False
        
        # Initialize IK solver
        self._ik_solver = RobotIKSolver()
        
        # Create ReentrantCallbackGroup for parallel callback execution
        self._cb_group = ReentrantCallbackGroup()
        
        # ==================== ARM STATE STORAGE ====================
        self._arm_state_lock = threading.Lock()
        self._arm_data_snapshot: Optional[Tuple[ArmState, int, int]] = None
        
        # Arm state cache for timestamp-based lookup
        self._arm_state_cache: deque = deque(maxlen=100)
        self._arm_state_cache_lock = threading.Lock()
        self._cache_timestamps_ns: np.ndarray = np.zeros(100, dtype=np.int64)
        
        # ==================== GRIPPER STATE STORAGE ====================
        self._gripper_state_lock = threading.Lock()
        self._gripper_data_snapshot: Optional[Tuple[GripperState, int]] = None
        self._max_gripper_width = 0.08  # Default for Franka hand
        
        # Latency logging counter
        self._latency_log_counter = 0
        
        # ==================== ARM SUBSCRIBERS ====================
        self._arm_state_sub = self._node.create_subscription(
            ArmState,
            f'/{arm_namespace}/arm_state',
            self._arm_state_callback,
            10,
            callback_group=self._cb_group
        )
        
        # ==================== GRIPPER SUBSCRIBERS ====================
        self._gripper_state_sub = self._node.create_subscription(
            GripperState,
            f'/{gripper_namespace}/gripper_state',
            self._gripper_state_callback,
            1,
            callback_group=self._cb_group
        )
        
        # ==================== ARM PUBLISHERS ====================
        self._joint_pos_cmd_pub = self._node.create_publisher(
            JointPositionCommand,
            f'/{arm_namespace}/joint_position_controller/command',
            10
        )
        
        self._joint_vel_cmd_pub = self._node.create_publisher(
            JointVelocityCommand,
            f'/{arm_namespace}/joint_velocity_controller/command',
            10
        )
        
        self._cartesian_pos_cmd_pub = self._node.create_publisher(
            CartesianPositionCommand,
            f'/{arm_namespace}/cartesian_position_controller/command',
            10
        )
        
        self._cartesian_vel_cmd_pub = self._node.create_publisher(
            CartesianVelocityCommand,
            f'/{arm_namespace}/cartesian_velocity_controller/command',
            10
        )
        
        # ==================== GRIPPER PUBLISHER ====================
        self._gripper_cmd_pub = self._node.create_publisher(
            GripperCommand,
            f'/{gripper_namespace}/gripper/command',
            10
        )
        
        # ==================== ARM SERVICE CLIENTS ====================
        self._arm_reset_client = self._node.create_client(
            Reset, f'/{arm_namespace}/reset'
        )
        
        self._start_cartesian_impedance_client = self._node.create_client(
            StartCartesianImpedance, f'/{arm_namespace}/start_cartesian_impedance'
        )
        
        self._start_joint_impedance_client = self._node.create_client(
            StartJointImpedance, f'/{arm_namespace}/start_joint_impedance'
        )
        
        self._start_joint_velocity_client = self._node.create_client(
            StartJointVelocity, f'/{arm_namespace}/start_joint_velocity'
        )
        
        self._terminate_policy_client = self._node.create_client(
            TerminatePolicy, f'/{arm_namespace}/terminate_policy'
        )
        
        self._move_to_joint_positions_client = self._node.create_client(
            MoveToJointPositions, f'/{arm_namespace}/move_to_joint_positions'
        )
        
        self._move_to_ee_pose_client = self._node.create_client(
            MoveToEEPose, f'/{arm_namespace}/move_to_ee_pose'
        )
        
        self._solve_ik_client = self._node.create_client(
            SolveIK, f'/{arm_namespace}/solve_ik'
        )
        
        self._compute_fk_client = self._node.create_client(
            ComputeFK, f'/{arm_namespace}/compute_fk'
        )
        
        self._compute_time_to_go_client = self._node.create_client(
            ComputeTimeToGo, f'/{arm_namespace}/compute_time_to_go'
        )
        
        # ==================== GRIPPER SERVICE CLIENTS ====================
        self._gripper_goto_client = self._node.create_client(
            GripperGoto, f'/{gripper_namespace}/gripper/goto'
        )
        
        self._gripper_grasp_client = self._node.create_client(
            GripperGrasp, f'/{gripper_namespace}/gripper/grasp'
        )
        
        self._gripper_open_client = self._node.create_client(
            Trigger, f'/{gripper_namespace}/gripper/open'
        )
        
        self._gripper_close_client = self._node.create_client(
            Trigger, f'/{gripper_namespace}/gripper/close'
        )
        
        # Wait for essential services
        self._node.get_logger().info(f"Waiting for V2 services (arm: /{arm_namespace}, gripper: /{gripper_namespace})...")
        if not self._arm_reset_client.wait_for_service(timeout_sec=5.0):
            self._node.get_logger().warn(f"Arm reset service not available at /{arm_namespace}/reset")
        
        # Spin thread management
        self._executor = None
        self._spin_thread = None
        
        if self._own_node:
            from rclpy.executors import MultiThreadedExecutor
            self._executor = MultiThreadedExecutor()
            self._executor.add_node(self._node)
            self._spin_thread = threading.Thread(target=self._spin_executor, daemon=True)
            self._spin_thread.start()
            self._node.get_logger().info("FrankaRobotV2: Own node created with spin thread")
        else:
            self._node.get_logger().debug("FrankaRobotV2: Using shared node - external executor handles spinning")
        
        # Wait for initial state
        self._wait_for_state(timeout=5.0)
        
        self._node.get_logger().info(
            f"FrankaRobotV2 interface initialized "
            f"(arm: /{arm_namespace}, gripper: /{gripper_namespace})"
        )
    
    # ==================== CALLBACKS ====================
    
    def _arm_state_callback(self, msg: ArmState):
        """Callback for arm state updates."""
        received_time_ns = get_ros_time_ns(self._node)
        
        pub_stamp = msg.header.stamp
        pub_timestamp_ns = int(pub_stamp.sec * 1_000_000_000 + pub_stamp.nanosec)
        
        snapshot = (msg, pub_timestamp_ns, received_time_ns)
        
        with self._arm_state_lock:
            self._arm_data_snapshot = snapshot
        
        with self._arm_state_cache_lock:
            self._arm_state_cache.append((pub_timestamp_ns, msg, received_time_ns))
            cache_len = len(self._arm_state_cache)
            if cache_len <= len(self._cache_timestamps_ns):
                self._cache_timestamps_ns[cache_len - 1] = pub_timestamp_ns
    
    def _gripper_state_callback(self, msg: GripperState):
        """Callback for gripper state updates."""
        received_time_ns = get_ros_time_ns(self._node)
        snapshot = (msg, received_time_ns)
        
        with self._gripper_state_lock:
            self._gripper_data_snapshot = snapshot
            if hasattr(msg, 'max_width') and msg.max_width > 0:
                self._max_gripper_width = msg.max_width
    
    def _wait_for_state(self, timeout: float = 5.0):
        """Wait for initial state to be received."""
        start_time = time.time()
        while time.time() - start_time < timeout:
            with self._arm_state_lock:
                arm_ready = self._arm_data_snapshot is not None
            with self._gripper_state_lock:
                gripper_ready = self._gripper_data_snapshot is not None
            if arm_ready and gripper_ready:
                return
            time.sleep(0.1)
        self._node.get_logger().warn("Timeout waiting for initial V2 robot state")
    
    # ==================== ARM STATE GETTERS ====================
    
    def _get_arm_state(self) -> Optional[ArmState]:
        """Get current arm state (thread-safe)."""
        with self._arm_state_lock:
            snapshot = self._arm_data_snapshot
        return snapshot[0] if snapshot is not None else None
    
    def _get_gripper_state(self) -> Optional[GripperState]:
        """Get current gripper state (thread-safe)."""
        with self._gripper_state_lock:
            snapshot = self._gripper_data_snapshot
        return snapshot[0] if snapshot is not None else None
    
    def get_joint_positions(self) -> List[float]:
        """Get current joint positions."""
        arm_state = self._get_arm_state()
        if arm_state is not None:
            return list(arm_state.joint_positions)
        return [0.0] * 7
    
    def get_joint_velocities(self) -> List[float]:
        """Get current joint velocities."""
        arm_state = self._get_arm_state()
        if arm_state is not None:
            return list(arm_state.joint_velocities)
        return [0.0] * 7
    
    def get_ee_pose(self) -> List[float]:
        """Get current end-effector pose [x, y, z, roll, pitch, yaw]."""
        arm_state = self._get_arm_state()
        if arm_state is not None:
            return list(arm_state.ee_position) + list(arm_state.ee_euler)
        return [0.0] * 6
    
    def get_ee_position(self) -> List[float]:
        """Get current end-effector position [x, y, z]."""
        arm_state = self._get_arm_state()
        if arm_state is not None:
            return list(arm_state.ee_position)
        return [0.0] * 3
    
    def get_ee_quaternion(self) -> List[float]:
        """Get current end-effector quaternion [x, y, z, w]."""
        arm_state = self._get_arm_state()
        if arm_state is not None:
            return list(arm_state.ee_quaternion)
        return [0.0, 0.0, 0.0, 1.0]
    
    # ==================== GRIPPER STATE GETTERS ====================
    
    def get_gripper_position(self) -> float:
        """Get current gripper position (normalized: 0=closed, 1=open)."""
        gripper_state = self._get_gripper_state()
        if gripper_state is not None:
            return float(gripper_state.position)
        return 0.0
    
    def get_gripper_width(self) -> float:
        """Get current gripper width in meters."""
        gripper_state = self._get_gripper_state()
        if gripper_state is not None:
            return float(gripper_state.width)
        return 0.0
    
    def get_gripper_state(self) -> float:
        """Alias for get_gripper_position() for compatibility."""
        return self.get_gripper_position()
    
    def is_gripper_grasped(self) -> bool:
        """Check if gripper has grasped an object."""
        gripper_state = self._get_gripper_state()
        if gripper_state is not None:
            return bool(gripper_state.is_grasped)
        return False
    
    # ==================== ROBOT STATE (COMBINED) ====================
    
    _DEFAULT_STATE = (
        {
            "cartesian_position": [0.0] * 6,
            "gripper_position": 0.0,
            "joint_positions": [0.0] * 7,
            "joint_velocities": [0.0] * 7,
            "joint_torques_computed": [0.0] * 7,
            "prev_joint_torques_computed": [0.0] * 7,
            "prev_joint_torques_computed_safened": [0.0] * 7,
            "motor_torques_measured": [0.0] * 7,
            "prev_controller_latency_ms": 0.0,
            "prev_command_successful": False,
        },
        {
            "robot_polymetis_t": 0,
            "robot_pub_t": 0,
            "robot_sub_t": 0,
            "robot_end_t": 0
        }
    )
    
    def get_robot_state(self) -> Tuple[Dict[str, Any], Dict[str, Any]]:
        """
        Get comprehensive robot state (arm + gripper).
        
        Returns:
            tuple: (state_dict, timestamp_dict)
        """
        # Get snapshots
        with self._arm_state_lock:
            arm_snapshot = self._arm_data_snapshot
        with self._gripper_state_lock:
            gripper_snapshot = self._gripper_data_snapshot
        
        if arm_snapshot is None or gripper_snapshot is None:
            return self._DEFAULT_STATE[0].copy(), self._DEFAULT_STATE[1].copy()
        
        arm_state, pub_timestamp_ns, received_timestamp_ns = arm_snapshot
        gripper_state, _ = gripper_snapshot
        
        processing_start_timestamp_ns = get_ros_time_ns(self._node)
        
        cartesian_position = list(arm_state.ee_position) + list(arm_state.ee_euler)
        
        state_dict = {
            "cartesian_position": cartesian_position,
            "gripper_position": float(gripper_state.position),
            "joint_positions": list(arm_state.joint_positions),
            "joint_velocities": list(arm_state.joint_velocities),
            "joint_torques_computed": list(arm_state.joint_torques_computed),
            "prev_joint_torques_computed": list(arm_state.prev_joint_torques_computed),
            "prev_joint_torques_computed_safened": list(arm_state.prev_joint_torques_computed_safened),
            "motor_torques_measured": list(arm_state.motor_torques_measured),
            "prev_controller_latency_ms": arm_state.prev_controller_latency_ms,
            "prev_command_successful": arm_state.prev_command_successful,
        }
        
        polymetis_timestamp_ns = arm_state.polymetis_timestamp_ns
        end_timestamp_ns = get_ros_time_ns(self._node)
        
        timestamp_dict = {
            "robot_polymetis_t": int(polymetis_timestamp_ns),
            "robot_pub_t": int(pub_timestamp_ns),
            "robot_sub_t": int(received_timestamp_ns),
            "robot_end_t": int(end_timestamp_ns),
        }
        
        return state_dict, timestamp_dict
    
    def get_arm_state_raw(self) -> Optional[ArmState]:
        """Get raw ArmState message for direct access."""
        return self._get_arm_state()
    
    def get_gripper_state_raw(self) -> Optional[GripperState]:
        """Get raw GripperState message for direct access."""
        return self._get_gripper_state()
    
    # ==================== ARM CONTROL ====================
    
    def reset(self, randomize=False, wait_for_completion=True, wait_time_sec=30.0, open_gripper=True):
        """
        Reset robot to home position.
        
        Steps:
        1. Open gripper (blocking) - optional
        2. Move arm to home position
        
        Args:
            randomize: If True, add random cartesian noise to reset position
            wait_for_completion: If True, wait for reset to complete
            wait_time_sec: Time to wait for reset completion
            open_gripper: If True, open gripper before resetting arm (default: True)
        """
        import inspect
        caller_info = inspect.stack()[1]
        caller_name = caller_info.filename.split('/')[-1] + ':' + str(caller_info.lineno)
        
        self._node.get_logger().info("=" * 70)
        self._node.get_logger().info(f"🔄 FrankaRobotV2.reset() called from: {caller_name}")
        self._node.get_logger().info(f"   Parameters: randomize={randomize}, wait_for_completion={wait_for_completion}, open_gripper={open_gripper}")
        
        # Step 1: Open gripper first
        if open_gripper:
            self._node.get_logger().info("   Step 1: Opening gripper...")
            gripper_success = self.gripper_open()
            if gripper_success:
                self._node.get_logger().info("   ✅ Gripper opened")
            else:
                self._node.get_logger().warn("   ⚠️ Gripper open failed or service unavailable")
        
        # Step 2: Reset arm to home position
        self._node.get_logger().info("   Step 2: Resetting arm to home position...")
        
        request = Reset.Request()
        request.randomize = randomize
        future = self._arm_reset_client.call_async(request)
        
        self._node.get_logger().info("   Waiting for arm reset service...")
        
        timeout_sec = 5.0
        start_time = time.time()
        while not future.done() and (time.time() - start_time) < timeout_sec:
            time.sleep(0.001)
        
        if future.done():
            response = future.result()
            if response.success:
                self._node.get_logger().info(f"   ✅ Arm reset accepted: {response.message}")
                if wait_for_completion:
                    self._node.get_logger().info(f"   ⏳ Waiting {wait_time_sec}s for reset to complete...")
                    time.sleep(wait_time_sec)
                    self._node.get_logger().info("   ✅ Reset wait complete")
            else:
                self._node.get_logger().error(f"   ❌ Arm reset failed: {response.message}")
        else:
            self._node.get_logger().error("   ❌ Reset service call timed out")
        
        self._node.get_logger().info("=" * 70)
    
    def home(self):
        """Move robot to home position (alias for reset)."""
        self.reset()
    
    def update_command(self, command, action_space="cartesian_velocity", gripper_action_space=None, blocking=False):
        """
        Update robot command (arm + gripper).
        
        Args:
            command: Command array (7 for arm + 1 for gripper, or 6 for cartesian + 1 gripper)
            action_space: "cartesian_position", "joint_position", "cartesian_velocity", "joint_velocity"
            gripper_action_space: "position" or "velocity" (optional)
            blocking: Whether to wait for command completion
        """
        action_dict = self.create_action_dict(command, action_space=action_space, gripper_action_space=gripper_action_space)
        
        # Send arm command
        self.update_joints(action_dict["joint_position"], velocity=False, blocking=blocking)
        
        # Send gripper command
        self.update_gripper(action_dict["gripper_position"], velocity=False, blocking=blocking)
        
        return action_dict
    
    def update_joints(self, command, velocity=False, blocking=False, cartesian_noise=None):
        """
        Update joint positions/velocities via V2 topics.
        
        Args:
            command: Joint command (7 DOF)
            velocity: If True, command is velocity; if False, command is position
            blocking: Whether to wait for completion
            cartesian_noise: Optional cartesian noise (not supported)
        """
        if cartesian_noise is not None:
            self._node.get_logger().warn("cartesian_noise not supported via V2 interface")
        
        if blocking and not velocity:
            return self._update_joints_blocking(command)
        else:
            return self._update_joints_non_blocking(command, velocity)
    
    def _update_joints_blocking(self, command) -> bool:
        """Blocking joint update using service."""
        if not self._move_to_joint_positions_client.wait_for_service(timeout_sec=1.0):
            self._node.get_logger().error("MoveToJointPositions service not available")
            return False
        
        request = MoveToJointPositions.Request()
        request.joint_positions = list(command)
        request.time_to_go = 0.0  # Auto-calculate
        
        future = self._move_to_joint_positions_client.call_async(request)
        timeout_sec = 10.0
        start_time = time.time()
        while not future.done() and (time.time() - start_time) < timeout_sec:
            time.sleep(0.001)
        
        if future.done():
            response = future.result()
            if response.success:
                self._node.get_logger().debug("Blocking move to joint positions completed")
                return True
            else:
                self._node.get_logger().error(f"Move to joint positions failed: {response.message}")
                return False
        else:
            self._node.get_logger().error("Move to joint positions service call timed out")
            return False
    
    def _update_joints_non_blocking(self, command, velocity):
        """Non-blocking joint update using topic."""
        if velocity:
            msg = JointVelocityCommand()
            msg.header.stamp = self._node.get_clock().now().to_msg()
            msg.velocities = list(command)
            self._joint_vel_cmd_pub.publish(msg)
        else:
            msg = JointPositionCommand()
            msg.header.stamp = self._node.get_clock().now().to_msg()
            msg.positions = list(command)
            msg.blocking = False
            self._joint_pos_cmd_pub.publish(msg)
    
    def update_pose(self, command, velocity=False, blocking=False):
        """
        Update end-effector pose.
        
        Args:
            command: Cartesian command [x, y, z, roll, pitch, yaw]
            velocity: If True, command is velocity; if False, command is position
            blocking: Whether to wait for completion
        """
        if blocking and not velocity:
            return self._update_pose_blocking(command)
        else:
            return self._update_pose_non_blocking(command, velocity)
    
    def _update_pose_blocking(self, command) -> bool:
        """Blocking pose update using service."""
        if not self._move_to_ee_pose_client.wait_for_service(timeout_sec=1.0):
            self._node.get_logger().error("MoveToEEPose service not available")
            return False
        
        pos = np.array(command[:3])
        quat = euler_to_quat(command[3:6])
        
        request = MoveToEEPose.Request()
        request.position = pos.tolist()
        request.orientation = quat.tolist()
        request.time_to_go = 0.0
        
        arm_state = self._get_arm_state()
        if arm_state is not None:
            request.q0 = list(arm_state.joint_positions)
        else:
            request.q0 = [0.0] * 7
        
        future = self._move_to_ee_pose_client.call_async(request)
        timeout_sec = 10.0
        start_time = time.time()
        while not future.done() and (time.time() - start_time) < timeout_sec:
            time.sleep(0.001)
        
        if future.done():
            response = future.result()
            if response.success:
                self._node.get_logger().debug("Blocking move to EE pose completed")
                return True
            else:
                self._node.get_logger().error(f"Move to EE pose failed: {response.message}")
                return False
        else:
            self._node.get_logger().error("Move to EE pose service call timed out")
            return False
    
    def _update_pose_non_blocking(self, command, velocity):
        """Non-blocking pose update using topic."""
        if not velocity:
            # Convert position to velocity
            curr_pose = self.get_ee_pose()
            cartesian_delta = pose_diff(command, curr_pose)
            command = self._ik_solver.cartesian_delta_to_velocity(cartesian_delta)
            velocity = True
        
        if velocity:
            msg = CartesianVelocityCommand()
            msg.header.stamp = self._node.get_clock().now().to_msg()
            msg.linear_velocity = list(command[:3])
            msg.angular_velocity = list(command[3:6]) if len(command) >= 6 else [0.0, 0.0, 0.0]
            self._cartesian_vel_cmd_pub.publish(msg)
        else:
            pos = command[:3]
            quat = euler_to_quat(command[3:6])
            
            msg = CartesianPositionCommand()
            msg.header.stamp = self._node.get_clock().now().to_msg()
            msg.position = list(pos)
            msg.orientation = list(quat)
            msg.blocking = False
            self._cartesian_pos_cmd_pub.publish(msg)
    
    # ==================== GRIPPER CONTROL ====================
    
    def update_gripper(self, command, velocity=True, blocking=False):
        """
        Update gripper position.
        
        Args:
            command: Gripper command (0=closed, 1=open for position; -1 to 1 for velocity)
            velocity: If True, command is velocity; if False, command is position
            blocking: Whether to wait for completion
        """
        if velocity:
            current_pos = self.get_gripper_position()
            gripper_delta = self._ik_solver.gripper_velocity_to_delta(command)
            command = gripper_delta + current_pos
        
        command = float(np.clip(command, 0, 1))
        
        # Convert normalized position (0=closed, 1=open) to width
        # command=0 -> width=0 (closed), command=1 -> width=max (open)
        width = self._max_gripper_width * command
        
        msg = GripperCommand()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.width = width
        msg.blocking = blocking
        
        self._gripper_cmd_pub.publish(msg)
    
    def gripper_goto(self, width: float, speed: float = 0.15, force: float = 0.1, blocking: bool = True) -> bool:
        """
        Move gripper to specified width using service.
        
        Args:
            width: Target width in meters
            speed: Movement speed (m/s)
            force: Grip force (normalized 0-1)
            blocking: Whether to wait for completion
        
        Returns:
            bool: True if successful
        """
        if not self._gripper_goto_client.wait_for_service(timeout_sec=1.0):
            self._node.get_logger().error("Gripper goto service not available")
            return False
        
        request = GripperGoto.Request()
        request.width = float(width)
        request.speed = float(speed)
        request.force = float(force)
        request.blocking = blocking
        
        future = self._gripper_goto_client.call_async(request)
        timeout_sec = 10.0
        start_time = time.time()
        while not future.done() and (time.time() - start_time) < timeout_sec:
            time.sleep(0.001)
        
        if future.done():
            response = future.result()
            return response.success
        return False
    
    def gripper_grasp(self, speed: float = 0.15, force: float = 0.5, blocking: bool = True) -> Tuple[bool, bool]:
        """
        Close gripper to grasp an object.
        
        Args:
            speed: Movement speed (m/s)
            force: Grip force (normalized 0-1)
            blocking: Whether to wait for completion
        
        Returns:
            tuple: (success, is_grasped)
        """
        if not self._gripper_grasp_client.wait_for_service(timeout_sec=1.0):
            self._node.get_logger().error("Gripper grasp service not available")
            return False, False
        
        request = GripperGrasp.Request()
        request.speed = float(speed)
        request.force = float(force)
        request.blocking = blocking
        
        future = self._gripper_grasp_client.call_async(request)
        timeout_sec = 10.0
        start_time = time.time()
        while not future.done() and (time.time() - start_time) < timeout_sec:
            time.sleep(0.001)
        
        if future.done():
            response = future.result()
            return response.success, response.is_grasped
        return False, False
    
    def gripper_open(self) -> bool:
        """Open gripper fully."""
        if not self._gripper_open_client.wait_for_service(timeout_sec=1.0):
            self._node.get_logger().error("Gripper open service not available")
            return False
        
        request = Trigger.Request()
        future = self._gripper_open_client.call_async(request)
        timeout_sec = 10.0
        start_time = time.time()
        while not future.done() and (time.time() - start_time) < timeout_sec:
            time.sleep(0.001)
        
        if future.done():
            response = future.result()
            return response.success
        return False
    
    def gripper_close(self) -> bool:
        """Close gripper fully."""
        if not self._gripper_close_client.wait_for_service(timeout_sec=1.0):
            self._node.get_logger().error("Gripper close service not available")
            return False
        
        request = Trigger.Request()
        future = self._gripper_close_client.call_async(request)
        timeout_sec = 10.0
        start_time = time.time()
        while not future.done() and (time.time() - start_time) < timeout_sec:
            time.sleep(0.001)
        
        if future.done():
            response = future.result()
            return response.success
        return False
    
    
    # ==================== CONTROLLER MANAGEMENT ====================
    
    def start_cartesian_impedance(self, kx=None, kxd=None) -> bool:
        """Start cartesian impedance controller."""
        if not self._start_cartesian_impedance_client.wait_for_service(timeout_sec=1.0):
            self._node.get_logger().warn("StartCartesianImpedance service not available")
            return False
        
        request = StartCartesianImpedance.Request()
        request.kx = list(kx) if kx is not None else []
        request.kxd = list(kxd) if kxd is not None else []
        
        future = self._start_cartesian_impedance_client.call_async(request)
        timeout_sec = 5.0
        start_time = time.time()
        while not future.done() and (time.time() - start_time) < timeout_sec:
            time.sleep(0.001)
        
        if future.done():
            response = future.result()
            if response.success:
                self._node.get_logger().info("Cartesian impedance started")
                return True
            else:
                self._node.get_logger().error(f"Failed to start cartesian impedance: {response.message}")
                return False
        else:
            self._node.get_logger().error("Start cartesian impedance service call timed out")
            return False
    
    def start_joint_impedance(self, kq=None, kqd=None) -> bool:
        """Start joint impedance controller."""
        if not self._start_joint_impedance_client.wait_for_service(timeout_sec=1.0):
            self._node.get_logger().warn("StartJointImpedance service not available")
            return False
        
        request = StartJointImpedance.Request()
        request.kq = list(kq) if kq is not None else []
        request.kqd = list(kqd) if kqd is not None else []
        
        future = self._start_joint_impedance_client.call_async(request)
        timeout_sec = 5.0
        start_time = time.time()
        while not future.done() and (time.time() - start_time) < timeout_sec:
            time.sleep(0.001)
        
        if future.done():
            response = future.result()
            if response.success:
                self._node.get_logger().info("Joint impedance started")
                return True
            else:
                self._node.get_logger().error(f"Failed to start joint impedance: {response.message}")
                return False
        else:
            self._node.get_logger().error("Start joint impedance service call timed out")
            return False
    
    def terminate_current_policy(self) -> bool:
        """Terminate current policy."""
        if not self._terminate_policy_client.wait_for_service(timeout_sec=1.0):
            self._node.get_logger().warn("TerminatePolicy service not available")
            return False
        
        request = TerminatePolicy.Request()
        future = self._terminate_policy_client.call_async(request)
        timeout_sec = 5.0
        start_time = time.time()
        while not future.done() and (time.time() - start_time) < timeout_sec:
            time.sleep(0.001)
        
        if future.done():
            response = future.result()
            if response.success:
                self._node.get_logger().info("Policy terminated")
                return True
            else:
                self._node.get_logger().error(f"Failed to terminate policy: {response.message}")
                return False
        else:
            self._node.get_logger().error("Terminate policy service call timed out")
            return False
    
    # ==================== IK/FK COMPUTATION ====================
    
    def solve_inverse_kinematics(self, position, orientation, q0=None, tolerance=1e-3):
        """Solve inverse kinematics."""
        if not self._solve_ik_client.wait_for_service(timeout_sec=1.0):
            self._node.get_logger().warn("SolveIK service not available")
            return None, False
        
        request = SolveIK.Request()
        request.position = list(position)[:3]
        
        if len(orientation) == 3:
            quat = euler_to_quat(orientation)
        else:
            quat = orientation
        request.orientation = list(quat)[:4]
        
        if q0 is None:
            arm_state = self._get_arm_state()
            if arm_state is not None:
                request.q0 = list(arm_state.joint_positions)
            else:
                request.q0 = [0.0] * 7
        else:
            request.q0 = list(q0)
        
        request.tolerance = tolerance
        
        future = self._solve_ik_client.call_async(request)
        timeout_sec = 5.0
        start_time = time.time()
        while not future.done() and (time.time() - start_time) < timeout_sec:
            time.sleep(0.001)
        
        if future.done():
            response = future.result()
            if response.success:
                return response.joint_positions, True
            else:
                self._node.get_logger().warn(f"IK solution not found: {response.message}")
                return response.joint_positions, False
        else:
            self._node.get_logger().error("Solve IK service call timed out")
            return None, False
    
    def compute_forward_kinematics(self, joint_positions):
        """Compute forward kinematics."""
        if not self._compute_fk_client.wait_for_service(timeout_sec=1.0):
            self._node.get_logger().warn("ComputeFK service not available")
            return None, None
        
        request = ComputeFK.Request()
        request.joint_positions = list(joint_positions)
        
        future = self._compute_fk_client.call_async(request)
        timeout_sec = 5.0
        start_time = time.time()
        while not future.done() and (time.time() - start_time) < timeout_sec:
            time.sleep(0.001)
        
        if future.done():
            response = future.result()
            if response.success:
                return response.position, response.orientation
            else:
                self._node.get_logger().error(f"FK computation failed: {response.message}")
                return None, None
        else:
            self._node.get_logger().error("Compute FK service call timed out")
            return None, None
    
    # ==================== ACTION DICT HELPER ====================
    
    def create_action_dict(self, action, action_space="cartesian_velocity", gripper_action_space=None, robot_state=None):
        """
        Create action dictionary from action array.
        
        Args:
            action: Action array
            action_space: "cartesian_position", "joint_position", "cartesian_velocity", "joint_velocity"
            gripper_action_space: "position" or "velocity" (optional)
            robot_state: Optional robot state dict (if None, uses current state)
        """
        assert action_space in ["cartesian_position", "joint_position", "cartesian_velocity", "joint_velocity"]
        
        if robot_state is None:
            robot_state = self.get_robot_state()[0]
        
        action_dict = {"robot_state": robot_state}
        velocity = "velocity" in action_space
        
        if gripper_action_space is None:
            gripper_action_space = "velocity" if velocity else "position"
        assert gripper_action_space in ["velocity", "position"]
        
        # Process gripper
        if gripper_action_space == "velocity":
            action_dict["gripper_velocity"] = action[-1]
            gripper_delta = self._ik_solver.gripper_velocity_to_delta(action[-1])
            gripper_position = robot_state["gripper_position"] + gripper_delta
            action_dict["gripper_position"] = float(np.clip(gripper_position, 0, 1))
        else:
            action_dict["gripper_position"] = float(np.clip(action[-1], 0, 1))
            gripper_delta = action_dict["gripper_position"] - robot_state["gripper_position"]
            gripper_velocity = self._ik_solver.gripper_delta_to_velocity(gripper_delta)
            action_dict["gripper_delta"] = gripper_velocity
        
        # Process arm
        if "cartesian" in action_space:
            if velocity:
                action_dict["cartesian_velocity"] = action[:-1]
                cartesian_delta = self._ik_solver.cartesian_velocity_to_delta(action[:-1])
                action_dict["cartesian_position"] = add_poses(
                    cartesian_delta, robot_state["cartesian_position"]
                ).tolist()
            else:
                action_dict["cartesian_position"] = action[:-1]
                cartesian_delta = pose_diff(action[:-1], robot_state["cartesian_position"])
                cartesian_velocity = self._ik_solver.cartesian_delta_to_velocity(cartesian_delta)
                action_dict["cartesian_velocity"] = cartesian_velocity.tolist()
            
            action_dict["joint_velocity"] = self._ik_solver.cartesian_velocity_to_joint_velocity(
                action_dict["cartesian_velocity"], robot_state=robot_state
            ).tolist()
            joint_delta = self._ik_solver.joint_velocity_to_delta(action_dict["joint_velocity"])
            action_dict["joint_position"] = (joint_delta + np.array(robot_state["joint_positions"])).tolist()
        
        if "joint" in action_space:
            if velocity:
                action_dict["joint_velocity"] = action[:-1]
                joint_delta = self._ik_solver.joint_velocity_to_delta(action[:-1])
                action_dict["joint_position"] = (joint_delta + np.array(robot_state["joint_positions"])).tolist()
            else:
                action_dict["joint_position"] = action[:-1]
                joint_delta = np.array(action[:-1]) - np.array(robot_state["joint_positions"])
                joint_velocity = self._ik_solver.joint_delta_to_velocity(joint_delta)
                action_dict["joint_velocity"] = joint_velocity.tolist()
        
        return action_dict
    
    # ==================== UTILITY METHODS ====================
    
    def _spin_executor(self):
        """Background thread method that spins the executor."""
        try:
            self._executor.spin()
        except Exception as e:
            if rclpy.ok():
                self._node.get_logger().error(f"Error in FrankaRobotV2 spin thread: {e}")
    
    def shutdown(self):
        """Clean up resources."""
        if not self._own_node:
            self._node.get_logger().debug("FrankaRobotV2: Using shared node - skipping shutdown")
            return
        
        self._node.get_logger().info("FrankaRobotV2: Shutting down...")
        
        if self._executor is not None:
            try:
                self._executor.shutdown(timeout_sec=2.0)
            except Exception as e:
                self._node.get_logger().warn(f"Error shutting down executor: {e}")
        
        if self._spin_thread is not None and self._spin_thread.is_alive():
            try:
                self._spin_thread.join(timeout=3.0)
            except Exception as e:
                self._node.get_logger().warn(f"Error joining spin thread: {e}")
        
        if self._node is not None:
            try:
                self._node.destroy_node()
            except Exception as e:
                self._node.get_logger().warn(f"Error destroying node: {e}")
        
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass
        
        self._node.get_logger().info("FrankaRobotV2: Shutdown complete")
    
    @property
    def arm_namespace(self) -> str:
        """Get arm namespace."""
        return self._arm_namespace
    
    @property
    def gripper_namespace(self) -> str:
        """Get gripper namespace."""
        return self._gripper_namespace
