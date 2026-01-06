# ROBOT SPECIFIC IMPORTS
import time
import threading
from typing import Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# ROS2 message imports
from role_ros2.msg import (
    PolymetisRobotCommand, PolymetisRobotState, PolymetisGripperState,
    GripperCommand
)
from role_ros2.srv import (
    Reset, StartCartesianImpedance, StartJointImpedance,
    TerminatePolicy, MoveToJointPositions, MoveToEEPose,
    SolveIK, ComputeFK, ComputeTimeToGo
)

# UTILITY SPECIFIC IMPORTS
from role_ros2.misc.transformations import add_poses, euler_to_quat, pose_diff, quat_to_euler
from role_ros2.misc.ros2_utils import get_ros_time_ns
from role_ros2.robot_ik.robot_ik_solver import RobotIKSolver
from role_ros2.robot.base_robot import BaseRobot


class FrankaRobot(BaseRobot):
    """
    ROS2-based robot interface that communicates with polymetis_bridge_node.
    
    This class replaces direct Polymetis calls with ROS2 topic communication:
    - Subscribes to /polymetis/robot_state and /polymetis/gripper_state for state
    - Publishes to /polymetis/robot_command for control commands
    - Uses service /polymetis/reset for robot initialization (home is alias for reset)
    """
    
    def __init__(self, node: Optional[Node] = None):
        """
        Initialize the robot interface.
        
        Args:
            node: Optional ROS2 node. If None, creates a new node.
        """
        # Create or use provided node
        if node is None:
            if not rclpy.ok():
                rclpy.init()
            self._node = Node('franka_robot_interface')
            self._own_node = True
        else:
            self._node = node
            self._own_node = False
        
        # Initialize IK solver
        self._ik_solver = RobotIKSolver()
        
        # Create ReentrantCallbackGroup for parallel callback execution
        # This allows robot state and gripper state callbacks to run concurrently
        # with camera callbacks, eliminating serial execution bottleneck
        self._cb_group = ReentrantCallbackGroup()
        
        # State storage (updated from subscribers)
        # Use atomic snapshots: (message, pub_timestamp_ns, received_time_ns)
        # This ensures data consistency - all three values belong to the same message
        self._robot_state_lock = threading.Lock()
        self._robot_data_snapshot: Optional[Tuple[PolymetisRobotState, int, int]] = None
        self._gripper_data_snapshot: Optional[Tuple[PolymetisGripperState, int]] = None
        self._max_gripper_width = 0.08  # Default for Franka hand
        
        # Latency logging counter (for lazy evaluation)
        self._latency_log_counter = 0
        
        # Subscribers for robot state (default RELIABLE QoS)
        # Queue size = 10 allows buffering multiple messages
        # This helps if get_robot_state() is called less frequently than messages arrive
        # However, we still only use the latest message, so this mainly helps with
        # ensuring messages are not dropped if processing is slow
        # Use ReentrantCallbackGroup for parallel execution
        self._robot_state_sub = self._node.create_subscription(
            PolymetisRobotState,
            '/polymetis/robot_state',
            self._robot_state_callback,
            10,  # Queue size: buffer up to 10 messages (but we only use latest)
            callback_group=self._cb_group
        )
        
        self._gripper_state_sub = self._node.create_subscription(
            PolymetisGripperState,
            '/polymetis/gripper_state',
            self._gripper_state_callback,
            1,
            callback_group=self._cb_group
        )
        
        # Publisher for robot commands (default RELIABLE QoS)
        self._command_pub = self._node.create_publisher(
            PolymetisRobotCommand,
            '/polymetis/robot_command',
            10
        )
        
        # Service client for reset
        self._reset_client = self._node.create_client(Reset, '/polymetis/reset')
        
        # Service clients for controller management
        self._start_cartesian_impedance_client = self._node.create_client(
            StartCartesianImpedance, '/polymetis/arm/start_cartesian_impedance'
        )
        self._start_joint_impedance_client = self._node.create_client(
            StartJointImpedance, '/polymetis/arm/start_joint_impedance'
        )
        self._terminate_policy_client = self._node.create_client(
            TerminatePolicy, '/polymetis/arm/terminate_policy'
        )
        
        # Service clients for motion control
        self._move_to_joint_positions_client = self._node.create_client(
            MoveToJointPositions, '/polymetis/arm/move_to_joint_positions'
        )
        self._move_to_ee_pose_client = self._node.create_client(
            MoveToEEPose, '/polymetis/arm/move_to_ee_pose'
        )
        
        # Service clients for computation
        self._solve_ik_client = self._node.create_client(
            SolveIK, '/polymetis/arm/solve_ik'
        )
        self._compute_fk_client = self._node.create_client(
            ComputeFK, '/polymetis/arm/compute_fk'
        )
        self._compute_time_to_go_client = self._node.create_client(
            ComputeTimeToGo, '/polymetis/arm/compute_time_to_go'
        )
        
        # Publisher for gripper commands (default RELIABLE QoS)
        self._gripper_cmd_pub = self._node.create_publisher(
            GripperCommand,
            '/polymetis/gripper/command',
            10
        )
        
        # Wait for essential services to be available
        self._node.get_logger().info("Waiting for polymetis services...")
        if not self._reset_client.wait_for_service(timeout_sec=5.0):
            self._node.get_logger().warn("Reset service not available")
        
        # Spin thread management based on node ownership
        self._executor = None
        self._spin_thread = None
        
        if self._own_node:
            # Own node: create executor and spin thread
            from rclpy.executors import MultiThreadedExecutor
            self._executor = MultiThreadedExecutor()
            self._executor.add_node(self._node)
            self._spin_thread = threading.Thread(target=self._spin_executor, daemon=True)
            self._spin_thread.start()
            self._node.get_logger().info("FrankaRobot: Own node created with spin thread")
        else:
            # Shared node: external executor handles spinning (e.g., RobotEnv's MultiThreadedExecutor)
            self._node.get_logger().debug("FrankaRobot: Using shared node - external executor handles spinning")
        
        # Wait for initial state
        self._wait_for_state(timeout=5.0)
        
        self._node.get_logger().info("FrankaRobot interface initialized")
    
    def _robot_state_callback(self, msg: PolymetisRobotState):
        """
        Callback for robot state updates.
        
        Creates an atomic snapshot: (message, pub_timestamp_ns, received_time_ns)
        This ensures all three values belong to the exact same message.
        """
        # Record ROS time when message is received
        received_time_ns = get_ros_time_ns(self._node)
        
        # Extract message header timestamp (pub_t) from the SAME message
        pub_stamp = msg.header.stamp
        pub_timestamp_ns = int(pub_stamp.sec * 1_000_000_000 + pub_stamp.nanosec)
        
        # Create atomic snapshot: (message, pub_t, sub_t)
        # All three values are from the same message, ensuring data consistency
        snapshot = (msg, pub_timestamp_ns, received_time_ns)
        
        with self._robot_state_lock:
            self._robot_data_snapshot = snapshot
    
    def _gripper_state_callback(self, msg: PolymetisGripperState):
        """
        Callback for gripper state updates.
        
        Creates an atomic snapshot: (message, received_time_ns)
        """
        # Record ROS time when message is received
        received_time_ns = get_ros_time_ns(self._node)
        
        # Create atomic snapshot: (message, received_time_ns)
        snapshot = (msg, received_time_ns)
        
        with self._robot_state_lock:
            self._gripper_data_snapshot = snapshot
            if hasattr(msg, 'max_width') and msg.max_width > 0:
                self._max_gripper_width = msg.max_width
    
    def _wait_for_state(self, timeout: float = 5.0):
        """
        Wait for initial state to be received.
        
        Background spin thread handles callbacks, so we just need to wait.
        """
        start_time = time.time()
        while time.time() - start_time < timeout:
            with self._robot_state_lock:
                if self._robot_data_snapshot is not None and self._gripper_data_snapshot is not None:
                    return
            time.sleep(0.1)
        self._node.get_logger().warn("Timeout waiting for initial robot state")
    
    def _get_current_state(self):
        """
        Get current robot state (thread-safe).
        
        Returns:
            tuple: (robot_state_msg, gripper_state_msg) or (None, None)
        """
        with self._robot_state_lock:
            robot_snapshot = self._robot_data_snapshot
            gripper_snapshot = self._gripper_data_snapshot
        
        # Unpack snapshots (outside lock to minimize hold time)
        robot_state = robot_snapshot[0] if robot_snapshot is not None else None
        gripper_state = gripper_snapshot[0] if gripper_snapshot is not None else None
        
        return robot_state, gripper_state
    
    def reset(self, randomize=False, wait_for_completion=True, wait_time_sec=30.0):
        """
        Reset robot to home position.
        
        Args:
            randomize: If True, add random cartesian noise to reset position (matching robot_env.py behavior)
            wait_for_completion: If True, wait for reset to complete (default: True)
            wait_time_sec: Time to wait for reset completion (default: 30.0s)
                          Reset moves at 0.1 rad/s, typical displacement is 1-2 rad, so ~10-20s
                          Adding buffer for gripper close time.
        
        Note: The reset service in polymetis_bridge_node executes asynchronously.
              The service call returns immediately with "accepted", then executes in background.
              If wait_for_completion=True, this method will wait for the robot to reach home position.
        """
        import inspect
        caller_info = inspect.stack()[1]
        caller_name = caller_info.filename.split('/')[-1] + ':' + str(caller_info.lineno)
        
        self._node.get_logger().info("=" * 70)
        self._node.get_logger().info(f"🔄 FrankaRobot.reset() called from: {caller_name}")
        self._node.get_logger().info(f"   Parameters: randomize={randomize}, wait_for_completion={wait_for_completion}, wait_time_sec={wait_time_sec}")
        
        request = Reset.Request()
        request.randomize = randomize
        future = self._reset_client.call_async(request)
        
        self._node.get_logger().info("   Waiting for reset service to accept request (timeout: 5.0s)...")
        
        # Wait for service to accept the request (non-blocking - let background executor handle it)
        # Cannot use rclpy.spin_until_future_complete() because node is already being spun by background thread
        timeout_sec = 5.0
        start_time = time.time()
        while not future.done() and (time.time() - start_time) < timeout_sec:
            time.sleep(0.001)  # Yield to background executor thread
        
        if future.done():
            response = future.result()
            if response.success:
                self._node.get_logger().info(f"✅ Robot reset command accepted: {response.message}")
                
                # Wait for reset to complete (reset executes in background thread)
                if wait_for_completion:
                    self._node.get_logger().info(f"⏳ Waiting {wait_time_sec}s for reset to complete...")
                    self._node.get_logger().info(f"   (Reset executes in background thread in polymetis_bridge_node)")
                    time.sleep(wait_time_sec)
                    self._node.get_logger().info("✅ Reset wait complete")
                else:
                    self._node.get_logger().info("ℹ️  Reset command sent (not waiting for completion)")
            else:
                self._node.get_logger().error(f"❌ Robot reset failed: {response.message}")
        else:
            self._node.get_logger().error("❌ Reset service call timed out (5.0s)")
        
        self._node.get_logger().info("=" * 70)
    
    def home(self):
        """Move robot to home position (alias for reset)."""
        # Home is the same as reset, so just call reset
        self.reset()
    
    def update_command(self, command, action_space="cartesian_velocity", gripper_action_space=None, blocking=False):
        """
        Update robot command (arm + gripper).
        
        Convert all control modes (cartesian/joint position/velocity) to joint position control.
        This matches the behavior of droid/droid/franka/robot.py.
        
        Args:
            command: Command array (7 for arm + 1 for gripper, or 6 for cartesian + 1 gripper)
            action_space: "cartesian_position", "joint_position", "cartesian_velocity", "joint_velocity"
            gripper_action_space: "position" or "velocity" (optional)
            blocking: Whether to wait for command completion
        """
        action_dict = self.create_action_dict(command, action_space=action_space, gripper_action_space=gripper_action_space)

        # Always convert to joint position control (matching droid implementation)
        self.update_joints(action_dict["joint_position"], velocity=False, blocking=blocking)
        self.update_gripper(action_dict["gripper_position"], velocity=False, blocking=blocking)

        return action_dict
    
    def update_pose(self, command, velocity=False, blocking=False):
        """
        Update end-effector pose.
        
        Args:
            command: Cartesian command [x, y, z, roll, pitch, yaw]
            velocity: If True, command is velocity; if False, command is position
            blocking: Whether to wait for completion
        
        Design:
            - Non-blocking: Uses Topic (/polymetis/robot_command) for low-latency control
            - Blocking: Uses Service (/polymetis/arm/move_to_ee_pose) to ensure completion
        """
        if blocking and not velocity:
            # BLOCKING MODE: Use Service (ensures completion)
            return self._update_pose_blocking(command)
        else:
            # NON-BLOCKING MODE: Use Topic (low latency)
            return self._update_pose_non_blocking(command, velocity)
    
    def _update_pose_blocking(self, command):
        """
        Blocking pose update using Service.
        
        Args:
            command: Cartesian command [x, y, z, roll, pitch, yaw]
        
        Returns:
            bool: True if successful, False otherwise
        """
        if not self._move_to_ee_pose_client.wait_for_service(timeout_sec=1.0):
            self._node.get_logger().error("MoveToEEPose service not available")
            return False
        
        pos = np.array(command[:3])
        quat = euler_to_quat(command[3:6])
        
        request = MoveToEEPose.Request()
        request.position = pos.tolist()
        request.orientation = quat.tolist()
        request.time_to_go = 0.0  # Auto-calculate
        
        # Get current joints for IK initial guess
        robot_state, _ = self._get_current_state()
        if robot_state is not None:
            request.q0 = list(robot_state.joint_positions)
        else:
            request.q0 = [0.0] * 7
        
        future = self._move_to_ee_pose_client.call_async(request)
        # Wait for service response (non-blocking - let background executor handle it)
        timeout_sec = 10.0
        start_time = time.time()
        while not future.done() and (time.time() - start_time) < timeout_sec:
            time.sleep(0.001)  # Yield to background executor thread
        
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
        """
        Non-blocking pose update using Topic.
        
        Args:
            command: Cartesian command [x, y, z, roll, pitch, yaw]
            velocity: If True, command is velocity; if False, command is position
        """
        if not velocity:
            # Convert position to velocity for smooth non-blocking control
            curr_pose = self.get_ee_pose()
            cartesian_delta = pose_diff(command, curr_pose)
            command = self._ik_solver.cartesian_delta_to_velocity(cartesian_delta)
            velocity = True  # After conversion, we're sending velocity
        
        # Publish non-blocking command via Topic
        msg = PolymetisRobotCommand()
        msg.action_space = "cartesian_velocity" if velocity else "cartesian_position"
        msg.command = list(command) + [0.0]  # Add gripper (no change)
        msg.blocking = False  # Topic commands are always non-blocking
        
        self._command_pub.publish(msg)
    
    def update_joints(self, command, velocity=False, blocking=False, cartesian_noise=None):
        """
        Update joint positions/velocities.
        
        Args:
            command: Joint command (7 DOF)
            velocity: If True, command is velocity; if False, command is position
            blocking: Whether to wait for completion
            cartesian_noise: Optional cartesian noise for randomization (not supported via ROS2)
        
        Design:
            - Non-blocking: Uses Topic (/polymetis/robot_command) for low-latency control
            - Blocking: Uses Service (/polymetis/arm/move_to_joint_positions) to ensure completion
        """
        if cartesian_noise is not None:
            self._node.get_logger().warn("cartesian_noise not supported via ROS2 interface")
        
        if blocking and not velocity:
            # BLOCKING MODE: Use Service (ensures completion)
            return self._update_joints_blocking(command)
        else:
            # NON-BLOCKING MODE: Use Topic (low latency)
            return self._update_joints_non_blocking(command, velocity, cartesian_noise)
    
    def _update_joints_blocking(self, command):
        """
        Blocking joint update using Service.
        
        Args:
            command: Joint command (7 DOF)
        
        Returns:
            bool: True if successful, False otherwise
        """
        if not self._move_to_joint_positions_client.wait_for_service(timeout_sec=1.0):
            self._node.get_logger().error("MoveToJointPositions service not available")
            return False
        
        request = MoveToJointPositions.Request()
        request.joint_positions = list(command)
        request.time_to_go = 0.0  # Auto-calculate
        
        future = self._move_to_joint_positions_client.call_async(request)
        # Wait for service response (non-blocking - let background executor handle it)
        timeout_sec = 10.0
        start_time = time.time()
        while not future.done() and (time.time() - start_time) < timeout_sec:
            time.sleep(0.001)  # Yield to background executor thread
        
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
    
    def _update_joints_non_blocking(self, command, velocity, cartesian_noise=None):
        """
        Non-blocking joint update using Topic.
        
        Args:
            command: Joint command (7 DOF)
            velocity: If True, command is velocity; if False, command is position
            cartesian_noise: Optional cartesian noise (not supported)
        """
        # Publish non-blocking command via Topic
        msg = PolymetisRobotCommand()
        msg.action_space = "joint_velocity" if velocity else "joint_position"
        msg.command = list(command) + [self.get_gripper_position()]  # Add gripper (no change)
        msg.blocking = False  # Topic commands are always non-blocking
        
        if cartesian_noise is not None:
            msg.cartesian_noise = list(cartesian_noise)
        
        self._command_pub.publish(msg)
    
    def update_gripper(self, command, velocity=True, blocking=False):
        """
        Update gripper position.
        
        Args:
            command: Gripper command (0=closed, 1=open for position; -1 to 1 for velocity)
            velocity: If True, command is velocity; if False, command is position
            blocking: Whether to wait for completion
        
        Design:
            - Non-blocking: Uses Topic (/polymetis/gripper/command) for low-latency control
            - Blocking: Currently uses Topic with blocking flag (bridge handles in thread)
            Note: For true blocking, consider adding a Service in the future
        """
        if velocity:
            current_pos = self.get_gripper_position()
            gripper_delta = self._ik_solver.gripper_velocity_to_delta(command)
            command = gripper_delta + current_pos
        
        command = float(np.clip(command, 0, 1))
        
        # Convert normalized position (0=closed, 1=open) to width (0=closed, max_width=open)
        width = self._max_gripper_width * (1 - command)
        
        # Publish gripper command using GripperCommand topic
        # Note: Bridge node handles blocking in a separate thread
        msg = GripperCommand()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.width = width
        msg.blocking = blocking  # Bridge will handle blocking in thread if needed
        
        self._gripper_cmd_pub.publish(msg)
    
    def get_joint_positions(self):
        """Get current joint positions."""
        robot_state, _ = self._get_current_state()
        if robot_state is not None:
            return list(robot_state.joint_positions)
        return [0.0] * 7
    
    def get_joint_velocities(self):
        """Get current joint velocities."""
        robot_state, _ = self._get_current_state()
        if robot_state is not None:
            return list(robot_state.joint_velocities)
        return [0.0] * 7
    
    def get_gripper_position(self):
        """Get current gripper position (normalized: 0=closed, 1=open)."""
        _, gripper_state = self._get_current_state()
        if gripper_state is not None:
            return float(gripper_state.position)
        return 0.0
    
    def get_gripper_state(self):
        """Alias for get_gripper_position() for compatibility with ServerInterface."""
        return self.get_gripper_position()
    
    def get_ee_pose(self):
        """Get current end-effector pose [x, y, z, roll, pitch, yaw]."""
        robot_state, _ = self._get_current_state()
        if robot_state is not None:
            # Return position + euler angles
            return list(robot_state.ee_position) + list(robot_state.ee_euler)
        return [0.0] * 6
    
    # Class-level constant for default state (avoid creating new objects on every call)
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
    
    def get_robot_state(self):
        """
        Get comprehensive robot state.
        
        Optimized for performance:
        - Minimal lock hold time (only copies snapshot reference)
        - Lazy latency calculation (only when logging)
        - Uses atomic snapshots for data consistency
        
        Returns:
            tuple: (state_dict, timestamp_dict)
                timestamp_dict contains:
                - robot_polymetis_t: Polymetis timestamp (nanoseconds)
                - robot_pub_t: Message header timestamp (nanoseconds)
                - robot_sub_t: Message received/subscription time (nanoseconds, ROS time)
                - robot_end_t: Processing end time (nanoseconds, ROS time)
        """
        # CRITICAL SECTION: Only copy snapshot references (minimal lock time)
        with self._robot_state_lock:
            robot_snapshot = self._robot_data_snapshot
            gripper_snapshot = self._gripper_data_snapshot
        
        # Fast path: return default if no data available
        if robot_snapshot is None or gripper_snapshot is None:
            return self._DEFAULT_STATE[0].copy(), self._DEFAULT_STATE[1].copy()
        
        # Unpack atomic snapshot (all values from the same message)
        robot_state, pub_timestamp_ns, received_timestamp_ns = robot_snapshot
        gripper_state, _ = gripper_snapshot
        
        # Record processing start time (for latency calculation, done lazily)
        processing_start_timestamp_ns = get_ros_time_ns(self._node)
        
        # Build state dict from ROS2 messages (outside lock)
        cartesian_position = list(robot_state.ee_position) + list(robot_state.ee_euler)
        
        state_dict = {
            "cartesian_position": cartesian_position,
            "gripper_position": float(gripper_state.position),
            "joint_positions": list(robot_state.joint_positions),
            "joint_velocities": list(robot_state.joint_velocities),
            "joint_torques_computed": list(robot_state.joint_torques_computed),
            "prev_joint_torques_computed": list(robot_state.prev_joint_torques_computed),
            "prev_joint_torques_computed_safened": list(robot_state.prev_joint_torques_computed_safened),
            "motor_torques_measured": list(robot_state.motor_torques_measured),
            "prev_controller_latency_ms": robot_state.prev_controller_latency_ms,
            "prev_command_successful": robot_state.prev_command_successful,
        }
        
        # Extract timestamps from snapshot (guaranteed to match)
        polymetis_timestamp_ns = robot_state.polymetis_timestamp_ns
        
        # robot_end_t: Processing end time
        end_timestamp_ns = get_ros_time_ns(self._node)
        
        # Build timestamp dict (all from the same atomic snapshot)
        timestamp_dict = {
            "robot_polymetis_t": int(polymetis_timestamp_ns),
            "robot_pub_t": int(pub_timestamp_ns),  # From snapshot
            "robot_sub_t": int(received_timestamp_ns),  # From snapshot (same message)
            "robot_end_t": int(end_timestamp_ns),
        }
        
        # Lazy latency calculation (only when logging is needed)
        self._latency_log_counter += 1
        if self._latency_log_counter % 50 == 0:
            self._check_and_log_latency(
                pub_timestamp_ns,
                received_timestamp_ns,
                processing_start_timestamp_ns,
                end_timestamp_ns
            )
        
        return state_dict, timestamp_dict
    
    def _check_and_log_latency(
        self,
        pub_timestamp_ns: int,
        received_timestamp_ns: int,
        processing_start_timestamp_ns: int,
        end_timestamp_ns: int
    ) -> None:
        """
        Check and log latency metrics (lazy evaluation).
        
        This method is only called periodically (every 50 calls) to avoid
        unnecessary CPU overhead from latency calculations.
        
        Args:
            pub_timestamp_ns: Message publication timestamp
            received_timestamp_ns: Message reception timestamp
            processing_start_timestamp_ns: Processing start timestamp
            end_timestamp_ns: Processing end timestamp
        """
        # Calculate processing time
        processing_time_ns = end_timestamp_ns - processing_start_timestamp_ns
        processing_time_ms = processing_time_ns / 1e6
        
        # Calculate pub_to_sub latency (network + ROS2 processing)
        if pub_timestamp_ns > 0 and received_timestamp_ns > 0:
            pub_to_sub_ns = received_timestamp_ns - pub_timestamp_ns
            pub_to_sub_ms = pub_to_sub_ns / 1e6
        else:
            pub_to_sub_ms = 0.0
        
        # Calculate sub_to_end latency (data staleness)
        if received_timestamp_ns > 0:
            sub_to_end_ns = end_timestamp_ns - received_timestamp_ns
            sub_to_end_ms = sub_to_end_ns / 1e6
        else:
            sub_to_end_ms = 0.0
        
        # Calculate message age
        if pub_timestamp_ns > 0:
            message_age_ns = end_timestamp_ns - pub_timestamp_ns
            message_age_ms = message_age_ns / 1e6
        else:
            message_age_ms = 0.0
        
        # Log warnings for large latencies
        if pub_to_sub_ms > 100.0:
            self._node.get_logger().warn(
                f"⚠️  Large pub_to_sub latency: {pub_to_sub_ms:.2f} ms. "
                f"This may indicate network/ROS2 processing delay. "
                f"Expected: < 50ms for local communication."
            )
        
        if sub_to_end_ms > 100.0:
            self._node.get_logger().warn(
                f"⚠️  Large sub_to_end latency: {sub_to_end_ms:.2f} ms. "
                f"Message age: {message_age_ms:.2f} ms. "
                f"This indicates get_robot_state() is called less frequently than messages arrive, "
                f"or there's processing delay. "
                f"Expected: < 50ms if called at >= 20Hz with 50Hz message rate."
            )
        
        if processing_time_ms > 10.0:
            self._node.get_logger().warn(
                f"⚠️  Large processing time: {processing_time_ms:.3f} ms. "
                f"This may indicate a performance issue in get_robot_state(). "
                f"Expected: < 1ms."
            )
    
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
    
    def start_cartesian_impedance(self, kx=None, kxd=None):
        """
        Start cartesian impedance controller.
        
        Args:
            kx: Optional cartesian position gains [x, y, z, roll, pitch, yaw]
            kxd: Optional cartesian velocity gains [x, y, z, roll, pitch, yaw]
        """
        if not self._start_cartesian_impedance_client.wait_for_service(timeout_sec=1.0):
            self._node.get_logger().warn("StartCartesianImpedance service not available")
            return False
        
        request = StartCartesianImpedance.Request()
        request.kx = list(kx) if kx is not None else []
        request.kxd = list(kxd) if kxd is not None else []
        
        future = self._start_cartesian_impedance_client.call_async(request)
        # Wait for service response (non-blocking - let background executor handle it)
        timeout_sec = 5.0
        start_time = time.time()
        while not future.done() and (time.time() - start_time) < timeout_sec:
            time.sleep(0.001)  # Yield to background executor thread
        
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
    
    def start_joint_impedance(self, kq=None, kqd=None):
        """
        Start joint impedance controller.
        
        Args:
            kq: Optional joint position gains (7 DOF)
            kqd: Optional joint velocity gains (7 DOF)
        """
        if not self._start_joint_impedance_client.wait_for_service(timeout_sec=1.0):
            self._node.get_logger().warn("StartJointImpedance service not available")
            return False
        
        request = StartJointImpedance.Request()
        request.kq = list(kq) if kq is not None else []
        request.kqd = list(kqd) if kqd is not None else []
        
        future = self._start_joint_impedance_client.call_async(request)
        # Wait for service response (non-blocking - let background executor handle it)
        timeout_sec = 5.0
        start_time = time.time()
        while not future.done() and (time.time() - start_time) < timeout_sec:
            time.sleep(0.001)  # Yield to background executor thread
        
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
    
    def terminate_current_policy(self):
        """Terminate current policy."""
        if not self._terminate_policy_client.wait_for_service(timeout_sec=1.0):
            self._node.get_logger().warn("TerminatePolicy service not available")
            return False
        
        request = TerminatePolicy.Request()
        future = self._terminate_policy_client.call_async(request)
        # Wait for service response (non-blocking - let background executor handle it)
        timeout_sec = 5.0
        start_time = time.time()
        while not future.done() and (time.time() - start_time) < timeout_sec:
            time.sleep(0.001)  # Yield to background executor thread
        
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
    
    def solve_inverse_kinematics(self, position, orientation, q0=None, tolerance=1e-3):
        """
        Solve inverse kinematics.
        
        Args:
            position: Desired position [x, y, z]
            orientation: Desired orientation as quaternion [x, y, z, w] or euler [roll, pitch, yaw]
            q0: Initial joint guess (7 DOF), if None uses current joints
            tolerance: IK solution tolerance
        
        Returns:
            tuple: (joint_positions, success)
        """
        if not self._solve_ik_client.wait_for_service(timeout_sec=1.0):
            self._node.get_logger().warn("SolveIK service not available")
            return None, False
        
        request = SolveIK.Request()
        request.position = list(position) if len(position) == 3 else position[:3]
        
        # Convert orientation to quaternion if needed
        if len(orientation) == 3:
            quat = euler_to_quat(orientation)
        else:
            quat = orientation
        request.orientation = list(quat) if len(quat) == 4 else quat[:4]
        
        if q0 is None:
            robot_state, _ = self._get_current_state()
            if robot_state is not None:
                request.q0 = list(robot_state.joint_positions)
            else:
                request.q0 = [0.0] * 7
        else:
            request.q0 = list(q0)
        
        request.tolerance = tolerance
        
        future = self._solve_ik_client.call_async(request)
        # Wait for service response (non-blocking - let background executor handle it)
        timeout_sec = 5.0
        start_time = time.time()
        while not future.done() and (time.time() - start_time) < timeout_sec:
            time.sleep(0.001)  # Yield to background executor thread
        
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
        """
        Compute forward kinematics.
        
        Args:
            joint_positions: Joint positions (7 DOF)
        
        Returns:
            tuple: (position [x, y, z], orientation [x, y, z, w] quaternion)
        """
        if not self._compute_fk_client.wait_for_service(timeout_sec=1.0):
            self._node.get_logger().warn("ComputeFK service not available")
            return None, None
        
        request = ComputeFK.Request()
        request.joint_positions = list(joint_positions)
        
        future = self._compute_fk_client.call_async(request)
        # Wait for service response (non-blocking - let background executor handle it)
        timeout_sec = 5.0
        start_time = time.time()
        while not future.done() and (time.time() - start_time) < timeout_sec:
            time.sleep(0.001)  # Yield to background executor thread
        
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
    
    def compute_time_to_go(self, desired_joint_positions):
        """
        Compute adaptive time to go.
        
        Args:
            desired_joint_positions: Target joint positions (7 DOF)
        
        Returns:
            float: Time to go in seconds
        """
        if not self._compute_time_to_go_client.wait_for_service(timeout_sec=1.0):
            self._node.get_logger().warn("ComputeTimeToGo service not available")
            return 2.0  # Default fallback
        
        request = ComputeTimeToGo.Request()
        request.desired_joint_positions = list(desired_joint_positions)
        
        future = self._compute_time_to_go_client.call_async(request)
        # Wait for service response (non-blocking - let background executor handle it)
        timeout_sec = 5.0
        start_time = time.time()
        while not future.done() and (time.time() - start_time) < timeout_sec:
            time.sleep(0.001)  # Yield to background executor thread
        
        if future.done():
            response = future.result()
            if response.success:
                return response.time_to_go
            else:
                self._node.get_logger().error(f"Time to go computation failed: {response.message}")
                return 2.0  # Default fallback
        else:
            self._node.get_logger().error("Compute time to go service call timed out")
            return 2.0  # Default fallback
    
    def _spin_executor(self):
        """
        Background thread method that spins the executor.
        Only used when own_node=True.
        """
        try:
            self._executor.spin()
        except Exception as e:
            if rclpy.ok():
                self._node.get_logger().error(f"Error in FrankaRobot spin thread: {e}")
    
    def shutdown(self):
        """
        Clean up resources.
        
        If own_node=True: stops executor, spin thread, and destroys node.
        If own_node=False: does nothing (node is managed by external executor).
        """
        if not self._own_node:
            # Shared node: external executor manages cleanup
            self._node.get_logger().debug("FrankaRobot: Using shared node - skipping shutdown")
            return
        
        # Own node: full cleanup
        self._node.get_logger().info("FrankaRobot: Shutting down...")
        
        # Step 1: Shutdown executor (stops spin thread)
        if self._executor is not None:
            try:
                self._executor.shutdown(timeout_sec=2.0)
            except Exception as e:
                self._node.get_logger().warn(f"Error shutting down executor: {e}")
        
        # Step 2: Join spin thread
        if self._spin_thread is not None and self._spin_thread.is_alive():
            try:
                self._spin_thread.join(timeout=3.0)
            except Exception as e:
                self._node.get_logger().warn(f"Error joining spin thread: {e}")
        
        # Step 3: Destroy node
        if self._node is not None:
            try:
                self._node.destroy_node()
            except Exception as e:
                self._node.get_logger().warn(f"Error destroying node: {e}")
        
        # Step 4: Shutdown rclpy if we initialized it
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception as e:
                pass  # May already be shutdown
        
        self._node.get_logger().info("FrankaRobot: Shutdown complete")
