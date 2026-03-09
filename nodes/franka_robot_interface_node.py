#!/usr/bin/env python3
"""
Franka Robot Interface Node - ROS 2 Driver for Polymetis Robot Arm

This node provides a ROS 2 interface for the Franka robot arm via Polymetis.
It handles arm control only; gripper is handled by a separate node.

Features:
- Publishers:
  - /{namespace}/joint_states: Joint states for tf tree
  - /{namespace}/arm_state: Comprehensive arm state
  - /{namespace}/ee_pose: End-effector pose
  - /{namespace}/controller_status: Controller status
- Subscribers:
  - /{namespace}/joint_position_controller/command: Joint position commands
  - /{namespace}/joint_velocity_controller/command: Joint velocity commands
  - /{namespace}/cartesian_position_controller/command: Cartesian position commands
  - /{namespace}/cartesian_velocity_controller/command: Cartesian velocity commands
- Services:
  - /{namespace}/reset: Reset robot to home position
  - /{namespace}/move_to_joint_positions: Blocking joint position control
  - /{namespace}/move_to_ee_pose: Blocking end-effector pose control
  - /{namespace}/start_joint_impedance: Start joint impedance controller
  - /{namespace}/start_cartesian_impedance: Start cartesian impedance controller
  - /{namespace}/start_joint_velocity: Start joint velocity controller
  - /{namespace}/terminate_policy: Terminate current policy
  - /{namespace}/solve_ik: Solve inverse kinematics
  - /{namespace}/compute_fk: Compute forward kinematics

Author: Role-ROS2 Team
"""

import time
import threading
import os
import sys
import traceback
import subprocess
import signal
from typing import Optional, List, Tuple
import numpy as np
from scipy.spatial.transform import Rotation as R

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

# Try to import Polymetis and PyTorch (graceful fallback if not available)
try:
    import torch
    TORCH_AVAILABLE = True
except ImportError:
    TORCH_AVAILABLE = False
    print("Warning: PyTorch not available. Mock mode will be used.")

try:
    from polymetis import RobotInterface
    POLYMETIS_AVAILABLE = True
except ImportError:
    POLYMETIS_AVAILABLE = False
    print("Warning: Polymetis not available. Using MockRobotInterface.")

# Import custom messages
from role_ros2.msg import (
    ArmState, ControllerStatus,
    JointPositionCommand, JointVelocityCommand,
    CartesianPositionCommand, CartesianVelocityCommand
)

# Import services
from role_ros2.srv import (
    Reset, StartCartesianImpedance, StartJointImpedance, StartJointVelocity,
    TerminatePolicy, MoveToJointPositions, MoveToEEPose,
    SolveIK, ComputeFK, ComputeTimeToGo
)

# Import IK solver and transformations
try:
    from role_ros2.robot_ik.robot_ik_solver import RobotIKSolver
    from role_ros2.misc.transformations import add_poses, euler_to_quat, quat_to_euler
    IK_SOLVER_AVAILABLE = True
except ImportError as e:
    IK_SOLVER_AVAILABLE = False
    print(f"Warning: RobotIKSolver not available: {e}.")


# ============================================================================
# HYPERPARAMETERS - Control max velocities and speeds
# ============================================================================

# Joint velocity limits (rad/s)
MAX_JOINT_DELTA = 0.2  # Maximum joint position delta per command (rad)
MAX_JOINT_VELOCITY = 1.0  # Maximum joint velocity for blocking movements (rad/s)

# Cartesian velocity limits
MAX_CARTESIAN_LIN_DELTA = 0.075  # Maximum linear position delta per command (m)
MAX_CARTESIAN_ROT_DELTA = 0.15  # Maximum angular position delta per command (rad)

# Control frequency (Hz)
CONTROL_HZ = 15.0

# Time-to-go calculation parameters
MIN_TIME_TO_GO = 0.5  # Minimum time for movement (seconds)
MAX_TIME_TO_GO = 4.0  # Maximum time for movement (seconds)
MIN_TIME_TO_GO_SLOW = 5.0  # Minimum time for slow/safe movements (seconds)
MAX_TIME_TO_GO_SLOW = 40.0  # Maximum time for slow/safe movements (seconds)
MAX_VELOCITY_FOR_TIME_CALC = 1.0  # Maximum velocity for time calculation (rad/s)
MAX_VELOCITY_SLOW = 0.1  # Maximum velocity for slow/safe movements (rad/s)

# ============================================================================


class MockRobotInterface:
    """
    Mock implementation of Polymetis RobotInterface for testing without hardware.
    
    Mimics the behavior of polymetis.RobotInterface, allowing development and testing
    without physical robot hardware.
    """
    
    def __init__(self, num_dofs: int = 7, update_rate: float = 50.0):
        """
        Initialize mock robot interface.
        
        Args:
            num_dofs: Number of degrees of freedom (default: 7 for Franka arm)
            update_rate: Update rate in Hz for state simulation
        """
        self.num_dofs = num_dofs
        self.update_rate = update_rate
        self.dt = 1.0 / update_rate
        
        # Franka rest pose (home position)
        rest_pose = [
            -0.13935425877571106,
            -0.020481698215007782,
            -0.05201413854956627,
            -2.0691256523132324,
            0.05058913677930832,
            2.0028650760650635,
            -0.9167874455451965
        ]
        
        # Current state
        if TORCH_AVAILABLE:
            self._joint_positions = torch.tensor(rest_pose[:num_dofs], dtype=torch.float32)
            self._joint_velocities = torch.zeros(num_dofs, dtype=torch.float32)
            self._kp = torch.ones(num_dofs, dtype=torch.float32) * 40.0
            self._kd = torch.ones(num_dofs, dtype=torch.float32) * 2.0
            self._target_joint_positions = torch.tensor(rest_pose[:num_dofs], dtype=torch.float32)
            self._target_joint_velocities = torch.zeros(num_dofs, dtype=torch.float32)
        else:
            self._joint_positions = np.array(rest_pose[:num_dofs], dtype=np.float32)
            self._joint_velocities = np.zeros(num_dofs)
            self._kp = np.ones(num_dofs) * 40.0
            self._kd = np.ones(num_dofs) * 2.0
            self._target_joint_positions = np.array(rest_pose[:num_dofs], dtype=np.float32)
            self._target_joint_velocities = np.zeros(num_dofs)
        
        self._is_running_policy = True
        self._last_update_time = time.time()
        self._state_lock = threading.Lock()
        
        # Motion parameters
        self._max_velocity = 1.0  # rad/s
        self._position_gain = 5.0
        self._velocity_gain = 1.0  # Lower = less damping (was 2.0)
    
    def update_state(self, dt: Optional[float] = None):
        """Update robot state based on target positions/velocities."""
        if not self._is_running_policy:
            return
        
        with self._state_lock:
            if dt is None:
                current_time = time.time()
                dt = min(current_time - self._last_update_time, 0.1)
                self._last_update_time = current_time
            
            if TORCH_AVAILABLE:
                pos_error = self._target_joint_positions - self._joint_positions
                desired_velocity = self._position_gain * pos_error + self._target_joint_velocities
                
                vel_norm = torch.norm(desired_velocity)
                if vel_norm > self._max_velocity:
                    desired_velocity = desired_velocity * (self._max_velocity / vel_norm)
                
                vel_error = desired_velocity - self._joint_velocities
                self._joint_velocities += self._velocity_gain * vel_error * dt
                self._joint_positions += self._joint_velocities * dt
            else:
                pos_error = self._target_joint_positions - self._joint_positions
                desired_velocity = self._position_gain * pos_error + self._target_joint_velocities
                
                vel_norm = np.linalg.norm(desired_velocity)
                if vel_norm > self._max_velocity:
                    desired_velocity = desired_velocity * (self._max_velocity / vel_norm)
                
                vel_error = desired_velocity - self._joint_velocities
                self._joint_velocities += self._velocity_gain * vel_error * dt
                self._joint_positions += self._joint_velocities * dt
    
    def get_joint_positions(self):
        """Get current joint positions."""
        with self._state_lock:
            if TORCH_AVAILABLE:
                return self._joint_positions.clone() if isinstance(self._joint_positions, torch.Tensor) else self._joint_positions.copy()
            return self._joint_positions.copy()
    
    def get_joint_velocities(self):
        """Get current joint velocities."""
        with self._state_lock:
            if TORCH_AVAILABLE:
                return self._joint_velocities.clone() if isinstance(self._joint_velocities, torch.Tensor) else self._joint_velocities.copy()
            return self._joint_velocities.copy()
    
    def get_robot_state(self):
        """Get robot state (mock implementation)."""
        with self._state_lock:
            if TORCH_AVAILABLE:
                positions = self._joint_positions.clone() if isinstance(self._joint_positions, torch.Tensor) else self._joint_positions.copy()
                velocities = self._joint_velocities.clone() if isinstance(self._joint_velocities, torch.Tensor) else self._joint_velocities.copy()
            else:
                positions = self._joint_positions.copy()
                velocities = self._joint_velocities.copy()
        
        class MockRobotState:
            def __init__(self, positions, velocities):
                self.joint_positions = positions
                self.joint_velocities = velocities
                self.joint_torques_computed = torch.zeros(7) if TORCH_AVAILABLE else np.zeros(7)
                self.prev_joint_torques_computed = torch.zeros(7) if TORCH_AVAILABLE else np.zeros(7)
                self.prev_joint_torques_computed_safened = torch.zeros(7) if TORCH_AVAILABLE else np.zeros(7)
                self.motor_torques_measured = torch.zeros(7) if TORCH_AVAILABLE else np.zeros(7)
                self.prev_controller_latency_ms = 0.0
                self.prev_command_successful = True
                self.timestamp = type('obj', (object,), {'seconds': 0, 'nanos': 0})()
        
        return MockRobotState(positions, velocities)
    
    def is_running_policy(self) -> bool:
        """Check if a policy is currently running."""
        return self._is_running_policy
    
    def start_joint_impedance(self, Kq=None, Kqd=None):
        """Start joint impedance control mode."""
        if Kq is not None:
            self._kp = torch.tensor(Kq, dtype=torch.float32) if TORCH_AVAILABLE else np.array(Kq)
        if Kqd is not None:
            self._kd = torch.tensor(Kqd, dtype=torch.float32) if TORCH_AVAILABLE else np.array(Kqd)
        self._is_running_policy = True
    
    def start_cartesian_impedance(self, Kx=None, Kxd=None):
        """Start cartesian impedance control mode."""
        self._is_running_policy = True
    
    def start_joint_velocity_control(self, qd_init=None):
        """Start joint velocity control mode."""
        if qd_init is not None:
            if TORCH_AVAILABLE:
                self._target_joint_velocities = torch.tensor(qd_init, dtype=torch.float32)
            else:
                self._target_joint_velocities = np.array(qd_init)
        self._is_running_policy = True
    
    def update_desired_joint_positions(self, positions):
        """Update desired joint positions (non-blocking)."""
        with self._state_lock:
            if TORCH_AVAILABLE:
                if isinstance(positions, torch.Tensor):
                    self._target_joint_positions = positions.clone()
                else:
                    self._target_joint_positions = torch.tensor(positions, dtype=torch.float32)
            else:
                self._target_joint_positions = np.array(positions)
    
    def update_desired_joint_velocities(self, velocities):
        """Update desired joint velocities (non-blocking)."""
        with self._state_lock:
            if TORCH_AVAILABLE:
                if isinstance(velocities, torch.Tensor):
                    self._target_joint_velocities = velocities.clone()
                else:
                    self._target_joint_velocities = torch.tensor(velocities, dtype=torch.float32)
            else:
                self._target_joint_velocities = np.array(velocities)
    
    def move_to_joint_positions(self, positions, time_to_go: float = 2.0):
        """Move to joint positions (blocking)."""
        with self._state_lock:
            if TORCH_AVAILABLE:
                if isinstance(positions, torch.Tensor):
                    self._target_joint_positions = positions.clone()
                else:
                    self._target_joint_positions = torch.tensor(positions, dtype=torch.float32)
            else:
                self._target_joint_positions = np.array(positions)
        
        start_time = time.time()
        dt = 1.0 / self.update_rate
        while time.time() - start_time < time_to_go:
            self.update_state(dt=dt)
            with self._state_lock:
                if TORCH_AVAILABLE:
                    error = torch.norm(self._target_joint_positions - self._joint_positions).item()
                else:
                    error = np.linalg.norm(self._target_joint_positions - self._joint_positions)
            if error < 0.01:
                break
            time.sleep(0.01)
    
    def terminate_current_policy(self):
        """Terminate current policy."""
        self._is_running_policy = False
    
    def solve_inverse_kinematics(self, pos, quat, curr_joints, tol=None):
        """Solve inverse kinematics (mock implementation)."""
        if TORCH_AVAILABLE:
            if isinstance(curr_joints, torch.Tensor):
                return curr_joints.clone(), True
            return torch.tensor(curr_joints, dtype=torch.float32), True
        return np.array(curr_joints), True
    
    def get_ee_pose(self):
        """Get end-effector pose (mock implementation)."""
        with self._state_lock:
            if TORCH_AVAILABLE:
                joints = self._joint_positions.cpu().numpy() if isinstance(self._joint_positions, torch.Tensor) else self._joint_positions.copy()
            else:
                joints = self._joint_positions.copy()
        
        # Simplified FK for mock
        x = 0.3 + 0.1 * joints[0] + 0.1 * joints[1]
        y = 0.0 + 0.1 * joints[0] - 0.1 * joints[2]
        z = 0.5 + 0.1 * joints[1] + 0.1 * joints[3]
        
        roll, pitch, yaw = joints[3], joints[4], joints[5]
        
        cy, sy = np.cos(yaw * 0.5), np.sin(yaw * 0.5)
        cp, sp = np.cos(pitch * 0.5), np.sin(pitch * 0.5)
        cr, sr = np.cos(roll * 0.5), np.sin(roll * 0.5)
        
        quat = np.array([
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy
        ])
        
        if TORCH_AVAILABLE:
            pos = torch.tensor([x, y, z], dtype=torch.float32)
            quat = torch.tensor(quat, dtype=torch.float32)
        else:
            pos = np.array([x, y, z])
        
        return pos, quat


class FrankaRobotInterfaceNode(Node):
    """
    ROS 2 Node that provides interface for Franka robot arm via Polymetis.
    
    This node handles arm control only; gripper is handled by a separate node.
    """
    
    def __init__(self, use_mock: bool = False, ip_address: str = "localhost", namespace: str = ""):
        """
        Initialize the Franka Robot Interface Node.
        
        Args:
            use_mock: If True, use MockRobotInterface instead of real hardware
            ip_address: IP address of Polymetis server
            namespace: ROS namespace for topics/services (e.g., "fr3_arm")
        """
        # Use namespace parameter or default
        node_name = 'franka_robot_interface_node'
        super().__init__(node_name, namespace=namespace)
        
        # Declare parameters (joint names are passed from launch; launch reads from config)
        self.declare_parameter('use_mock', use_mock)
        self.declare_parameter('ip_address', ip_address)
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('namespace', namespace)
        self.declare_parameter('arm_joint_names', [])
        self.declare_parameter('polymetis_port', 50051)
        self.declare_parameter('ee_frame_id', 'base_link')

        # Get parameters (ROS 2: declare then get; launch must pass arm_joint_names from config)
        use_mock = self.get_parameter('use_mock').get_parameter_value().bool_value
        ip_address = self.get_parameter('ip_address').get_parameter_value().string_value
        polymetis_port = self.get_parameter('polymetis_port').get_parameter_value().integer_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self._namespace = self.get_parameter('namespace').get_parameter_value().string_value
        self.arm_joint_names = list(
            self.get_parameter('arm_joint_names').get_parameter_value().string_array_value
        )
        self._ee_frame_id = self.get_parameter('ee_frame_id').get_parameter_value().string_value
        if not self.arm_joint_names:
            raise ValueError(
                "arm_joint_names is empty. Launch file must pass arm_joints from config "
                "(franka_robot_config.yaml or bimanual_franka_robot_config.yaml)."
            )
        self.get_logger().info(
            f"Using {len(self.arm_joint_names)} arm joint names: {self.arm_joint_names}, "
            f"ee_frame_id: {self._ee_frame_id}"
        )
        
        # Initialize robot interface
        self._robot: Optional[RobotInterface] = None
        
        try:
            if use_mock or not POLYMETIS_AVAILABLE:
                self.get_logger().info("Using MockRobotInterface (no hardware)")
                self._robot = MockRobotInterface(num_dofs=7, update_rate=self.publish_rate)
            else:
                self.get_logger().info(f"Connecting to Polymetis server at {ip_address}:{polymetis_port}")
                self._robot = RobotInterface(ip_address=ip_address, port=polymetis_port)
                self.get_logger().info("Successfully connected to Polymetis server")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to Polymetis: {e}. Using mock interface.")
            self._robot = MockRobotInterface(num_dofs=7, update_rate=self.publish_rate)
        
        # Initialize IK solver
        self._ik_solver = None
        self._ik_solver_lock = threading.Lock()
        if IK_SOLVER_AVAILABLE:
            try:
                self._ik_solver = RobotIKSolver()
                self.get_logger().info("RobotIKSolver initialized successfully")
            except Exception as e:
                self.get_logger().warning(f"RobotIKSolver initialization failed: {e}")
                self._ik_solver = None
        
        # State storage and thread safety
        self._command_lock = threading.Lock()
        self._controller_mode = "none"
        self._current_kp = None
        self._current_kd = None
        self._controller_not_loaded = False
        
        # Default reset/home joint positions
        self._reset_joints = np.array([0, -1 / 5 * np.pi, 0, -4 / 5 * np.pi, 0, 3 / 5 * np.pi, 0.0])
        
        # Callback groups
        self._timer_callback_group = ReentrantCallbackGroup()
        self._service_callback_group = MutuallyExclusiveCallbackGroup()
        
        # ===== Publishers =====
        self._joint_state_publisher = self.create_publisher(
            JointState, 'joint_states', 10
        )
        
        self._arm_state_publisher = self.create_publisher(
            ArmState, 'arm_state', 10
        )
        
        self._ee_pose_publisher = self.create_publisher(
            PoseStamped, 'ee_pose', 10
        )
        
        self._controller_status_publisher = self.create_publisher(
            ControllerStatus, 'controller_status', 10
        )
        
        # ===== Subscribers =====
        self._joint_pos_cmd_subscriber = self.create_subscription(
            JointPositionCommand,
            'joint_position_controller/command',
            self._joint_position_command_callback,
            10
        )
        
        self._joint_vel_cmd_subscriber = self.create_subscription(
            JointVelocityCommand,
            'joint_velocity_controller/command',
            self._joint_velocity_command_callback,
            10
        )
        
        self._cartesian_pos_cmd_subscriber = self.create_subscription(
            CartesianPositionCommand,
            'cartesian_position_controller/command',
            self._cartesian_position_command_callback,
            10
        )
        
        self._cartesian_vel_cmd_subscriber = self.create_subscription(
            CartesianVelocityCommand,
            'cartesian_velocity_controller/command',
            self._cartesian_velocity_command_callback,
            10
        )
        
        # ===== Services =====
        self._reset_service = self.create_service(
            Reset, 'reset', self._reset_service_callback,
            callback_group=self._service_callback_group
        )
        
        self._move_to_joint_positions_service = self.create_service(
            MoveToJointPositions, 'move_to_joint_positions',
            self._move_to_joint_positions_callback,
            callback_group=self._service_callback_group
        )
        
        self._move_to_ee_pose_service = self.create_service(
            MoveToEEPose, 'move_to_ee_pose',
            self._move_to_ee_pose_callback,
            callback_group=self._service_callback_group
        )
        
        self._start_joint_impedance_service = self.create_service(
            StartJointImpedance, 'start_joint_impedance',
            self._start_joint_impedance_callback,
            callback_group=self._service_callback_group
        )
        
        self._start_cartesian_impedance_service = self.create_service(
            StartCartesianImpedance, 'start_cartesian_impedance',
            self._start_cartesian_impedance_callback,
            callback_group=self._service_callback_group
        )
        
        self._start_joint_velocity_service = self.create_service(
            StartJointVelocity, 'start_joint_velocity',
            self._start_joint_velocity_callback,
            callback_group=self._service_callback_group
        )
        
        self._terminate_policy_service = self.create_service(
            TerminatePolicy, 'terminate_policy',
            self._terminate_policy_callback,
            callback_group=self._service_callback_group
        )
        
        self._solve_ik_service = self.create_service(
            SolveIK, 'solve_ik',
            self._solve_ik_callback,
            callback_group=self._service_callback_group
        )
        
        self._compute_fk_service = self.create_service(
            ComputeFK, 'compute_fk',
            self._compute_fk_callback,
            callback_group=self._service_callback_group
        )
        
        self._compute_time_to_go_service = self.create_service(
            ComputeTimeToGo, 'compute_time_to_go',
            self._compute_time_to_go_callback,
            callback_group=self._service_callback_group
        )
        
        # Timer for publishing states
        timer_period = 1.0 / self.publish_rate
        self._state_timer = self.create_timer(
            timer_period, self._publish_states,
            callback_group=self._timer_callback_group
        )
        
        # Start impedance controller
        self._start_impedance_controller()
        
        self.get_logger().info(
            f'FrankaRobotInterfaceNode initialized. '
            f'Namespace: {self._namespace or "(default)"}, '
            f'Publishing at {self.publish_rate} Hz'
        )
    
    def _start_impedance_controller(self):
        """Start the impedance controller for continuous control."""
        try:
            if not isinstance(self._robot, MockRobotInterface):
                if not self._robot.is_running_policy():
                    self._robot.start_joint_impedance()
                    self._controller_mode = "joint_impedance"
        except Exception as e:
            self.get_logger().warn(f"Failed to start impedance controller: {e}")
    
    def _compute_forward_kinematics(self, joint_positions) -> Tuple[np.ndarray, np.ndarray]:
        """
        Compute forward kinematics for end-effector pose.
        
        Args:
            joint_positions: Joint positions (7 DOF)
            
        Returns:
            tuple: (position [x, y, z], quaternion [x, y, z, w])
        """
        try:
            if TORCH_AVAILABLE and hasattr(self._robot, 'robot_model'):
                if isinstance(joint_positions, torch.Tensor):
                    joint_pos_tensor = joint_positions
                else:
                    joint_pos_tensor = torch.Tensor(joint_positions)
                pos, quat = self._robot.robot_model.forward_kinematics(joint_pos_tensor)
                if isinstance(pos, torch.Tensor):
                    pos = pos.cpu().numpy()
                if isinstance(quat, torch.Tensor):
                    quat = quat.cpu().numpy()
                return pos, quat
            elif isinstance(self._robot, MockRobotInterface) and self._ik_solver is not None:
                with self._ik_solver_lock:
                    joint_pos = np.array(joint_positions)
                    joint_vel = np.zeros(7)
                    self._ik_solver._arm.update_state(self._ik_solver._physics, joint_pos, joint_vel)
                    
                    link8_body = self._ik_solver._arm.mjcf_model.find("body", "panda_link8")
                    if link8_body is not None:
                        body_bind = self._ik_solver._physics.bind(link8_body)
                        pos = body_bind.xpos.copy()
                        quat_mat = body_bind.xmat.copy().reshape(3, 3)
                        quat_rot = R.from_matrix(quat_mat)
                        quat = quat_rot.as_quat()
                    else:
                        site_bind = self._ik_solver._physics.bind(self._ik_solver._arm.wrist_site)
                        wrist_pos = site_bind.xpos.copy()
                        wrist_quat_mat = site_bind.xmat.copy().reshape(3, 3)
                        hand_rot = R.from_euler('z', -0.785398163397, degrees=False)
                        wrist_rot = R.from_matrix(wrist_quat_mat)
                        link8_rot = wrist_rot * hand_rot.inv()
                        pos = wrist_pos.copy()
                        quat = link8_rot.as_quat()
                
                return pos, quat
            else:
                pos, quat = self._robot.get_ee_pose()
                if isinstance(pos, torch.Tensor):
                    pos = pos.cpu().numpy()
                if isinstance(quat, torch.Tensor):
                    quat = quat.cpu().numpy()
                return pos, quat
        except Exception as e:
            self.get_logger().warn(f"Failed to compute forward kinematics: {e}")
            return np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.0, 0.0, 1.0])
    
    def _publish_states(self):
        """Publish joint states, arm state, EE pose, and controller status."""
        # Update mock state if using mock interface
        if isinstance(self._robot, MockRobotInterface):
            dt = 1.0 / self.publish_rate
            self._robot.update_state(dt=dt)
        
        self._publish_joint_states()
        self._publish_arm_state()
        self._publish_ee_pose()
        self._publish_controller_status()
    
    def _publish_joint_states(self):
        """Publish joint states."""
        try:
            joint_positions = self._robot.get_joint_positions()
            joint_velocities = self._robot.get_joint_velocities()
            
            if TORCH_AVAILABLE and isinstance(joint_positions, torch.Tensor):
                joint_positions = joint_positions.cpu().numpy()
            if TORCH_AVAILABLE and isinstance(joint_velocities, torch.Tensor):
                joint_velocities = joint_velocities.cpu().numpy()
            
            joint_positions = joint_positions.tolist() if hasattr(joint_positions, 'tolist') else list(joint_positions)
            joint_velocities = joint_velocities.tolist() if hasattr(joint_velocities, 'tolist') else list(joint_velocities)
            
            msg = JointState()
            now = self.get_clock().now()
            msg.header.stamp = now.to_msg()
            msg.header.frame_id = self._ee_frame_id
            msg.name = list(self.arm_joint_names)
            msg.position = joint_positions[:len(self.arm_joint_names)]
            msg.velocity = joint_velocities[:len(self.arm_joint_names)]
            msg.effort = [0.0] * len(self.arm_joint_names)
            
            self._joint_state_publisher.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Error publishing joint states: {e}")
    
    def _publish_arm_state(self):
        """Publish comprehensive arm state."""
        try:
            data_ros_time = self.get_clock().now()
            data_timestamp_ns = data_ros_time.nanoseconds
            
            robot_state = self._robot.get_robot_state()
            pos, quat = self._compute_forward_kinematics(robot_state.joint_positions)
            
            def to_list(x):
                if TORCH_AVAILABLE and isinstance(x, torch.Tensor):
                    return x.cpu().numpy().tolist()
                elif hasattr(x, 'tolist'):
                    return x.tolist()
                return list(x)
            
            msg = ArmState()
            msg.header.stamp = data_ros_time.to_msg()
            msg.header.frame_id = self._ee_frame_id
            
            msg.joint_positions = to_list(robot_state.joint_positions)
            msg.joint_velocities = to_list(robot_state.joint_velocities)
            msg.joint_torques_computed = to_list(robot_state.joint_torques_computed)
            msg.prev_joint_torques_computed = to_list(robot_state.prev_joint_torques_computed)
            msg.prev_joint_torques_computed_safened = to_list(robot_state.prev_joint_torques_computed_safened)
            msg.motor_torques_measured = to_list(robot_state.motor_torques_measured)
            
            ee_pos_list = pos.tolist() if hasattr(pos, 'tolist') else list(pos)
            ee_quat_list = quat.tolist() if hasattr(quat, 'tolist') else list(quat)
            try:
                euler = quat_to_euler(quat) if IK_SOLVER_AVAILABLE else np.zeros(3)
                ee_euler_list = euler.tolist() if hasattr(euler, 'tolist') else list(euler)
            except Exception:
                ee_euler_list = [0.0, 0.0, 0.0]

            msg.ee_position = ee_pos_list
            msg.ee_quaternion = ee_quat_list
            msg.ee_euler = ee_euler_list
            msg.ee_position_local = ee_pos_list[:]
            msg.ee_quaternion_local = ee_quat_list[:]
            msg.ee_euler_local = ee_euler_list[:]
            
            msg.prev_controller_latency_ms = float(robot_state.prev_controller_latency_ms)
            msg.prev_command_successful = bool(robot_state.prev_command_successful)
            msg.is_running_policy = bool(self._robot.is_running_policy() if hasattr(self._robot, 'is_running_policy') else False)
            msg.controller_mode = self._controller_mode
            
            if self._current_kp is not None:
                msg.kp = to_list(self._current_kp)
            else:
                msg.kp = []
            
            if self._current_kd is not None:
                msg.kd = to_list(self._current_kd)
            else:
                msg.kd = []
            
            msg.polymetis_timestamp_ns = int(data_timestamp_ns)
            
            self._arm_state_publisher.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Error publishing arm state: {e}\n{traceback.format_exc()}")
    
    def _publish_ee_pose(self):
        """Publish end-effector pose."""
        try:
            joint_positions = self._robot.get_joint_positions()
            pos, quat = self._compute_forward_kinematics(joint_positions)
            pos = pos.tolist() if hasattr(pos, 'tolist') else list(pos)
            quat = quat.tolist() if hasattr(quat, 'tolist') else list(quat)
            
            msg = PoseStamped()
            now = self.get_clock().now()
            msg.header.stamp = now.to_msg()
            msg.header.frame_id = self._ee_frame_id
            
            msg.pose.position.x = float(pos[0])
            msg.pose.position.y = float(pos[1])
            msg.pose.position.z = float(pos[2])
            msg.pose.orientation.x = float(quat[0])
            msg.pose.orientation.y = float(quat[1])
            msg.pose.orientation.z = float(quat[2])
            msg.pose.orientation.w = float(quat[3])
            
            self._ee_pose_publisher.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Error publishing EE pose: {e}")
    
    def _publish_controller_status(self):
        """Publish controller status."""
        try:
            msg = ControllerStatus()
            now = self.get_clock().now()
            msg.header.stamp = now.to_msg()
            msg.header.frame_id = self._ee_frame_id
            
            msg.is_running_policy = bool(self._robot.is_running_policy())
            msg.controller_mode = self._controller_mode
            
            if self._current_kp is not None:
                if TORCH_AVAILABLE and isinstance(self._current_kp, torch.Tensor):
                    msg.kp = self._current_kp.cpu().numpy().tolist()
                else:
                    msg.kp = list(self._current_kp)
            else:
                msg.kp = []
            
            if self._current_kd is not None:
                if TORCH_AVAILABLE and isinstance(self._current_kd, torch.Tensor):
                    msg.kd = self._current_kd.cpu().numpy().tolist()
                else:
                    msg.kd = list(self._current_kd)
            else:
                msg.kd = []
            
            self._controller_status_publisher.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Error publishing controller status: {e}")
    
    def _joint_position_command_callback(self, msg: JointPositionCommand):
        """Callback for joint position commands."""
        if self._controller_not_loaded:
            return
        
        try:
            positions = list(msg.positions)
            
            if len(positions) != 7:
                self.get_logger().error(f"Invalid joint positions length: {len(positions)}, expected 7")
                return
            
            if not TORCH_AVAILABLE:
                self.get_logger().error("PyTorch not available")
                return
            
            pos_tensor = torch.tensor(positions, dtype=torch.float32)
            
            if msg.blocking:
                # Blocking mode: run in separate thread
                def blocking_move():
                    try:
                        if self._robot.is_running_policy():
                            self._robot.terminate_current_policy()
                        
                        time_to_go = msg.time_to_go if msg.time_to_go > 0 else 2.0
                        self._robot.move_to_joint_positions(pos_tensor, time_to_go=time_to_go)
                        self._robot.start_cartesian_impedance()
                        self._controller_mode = "cartesian_impedance"
                    except Exception as e:
                        self.get_logger().error(f"Blocking move error: {e}")
                
                threading.Thread(target=blocking_move, daemon=True).start()
            else:
                # Non-blocking mode: update desired positions
                def helper_non_blocking():
                    try:
                        if not self._robot.is_running_policy():
                            self._controller_not_loaded = True
                            self._robot.start_cartesian_impedance()
                            self._controller_mode = "cartesian_impedance"
                            
                            timeout = time.time() + 5
                            while not self._robot.is_running_policy():
                                time.sleep(0.01)
                                if time.time() > timeout:
                                    self._robot.start_cartesian_impedance()
                                    timeout = time.time() + 5
                            
                            self._controller_not_loaded = False
                        
                        self._robot.update_desired_joint_positions(pos_tensor)
                    except Exception as e:
                        self.get_logger().error(f"Non-blocking command error: {e}")
                        self._controller_not_loaded = False
                
                threading.Thread(target=helper_non_blocking, daemon=True).start()
        except Exception as e:
            self.get_logger().error(f"Error processing joint position command: {e}")
    
    def _joint_velocity_command_callback(self, msg: JointVelocityCommand):
        """Callback for joint velocity commands."""
        if self._controller_not_loaded:
            return
        
        try:
            velocities = list(msg.velocities)
            
            if len(velocities) != 7:
                self.get_logger().error(f"Invalid joint velocities length: {len(velocities)}, expected 7")
                return
            
            if not TORCH_AVAILABLE:
                self.get_logger().error("PyTorch not available")
                return
            
            vel_tensor = torch.tensor(velocities, dtype=torch.float32)
            
            def update_velocity():
                try:
                    if not self._robot.is_running_policy() or self._controller_mode != "joint_velocity":
                        self._controller_not_loaded = True
                        self._robot.start_joint_velocity_control(vel_tensor)
                        self._controller_mode = "joint_velocity"
                        
                        timeout = time.time() + 5
                        while not self._robot.is_running_policy():
                            time.sleep(0.01)
                            if time.time() > timeout:
                                self._robot.start_joint_velocity_control(vel_tensor)
                                timeout = time.time() + 5
                        
                        self._controller_not_loaded = False
                    else:
                        self._robot.update_desired_joint_velocities(vel_tensor)
                except Exception as e:
                    self.get_logger().error(f"Joint velocity command error: {e}")
                    self._controller_not_loaded = False
            
            threading.Thread(target=update_velocity, daemon=True).start()
        except Exception as e:
            self.get_logger().error(f"Error processing joint velocity command: {e}")
    
    def _cartesian_position_command_callback(self, msg: CartesianPositionCommand):
        """Callback for cartesian position commands."""
        if self._controller_not_loaded:
            return
        
        try:
            position = list(msg.position)
            orientation = list(msg.orientation)
            
            if len(position) != 3 or len(orientation) != 4:
                self.get_logger().error(f"Invalid cartesian command: position={len(position)}, orientation={len(orientation)}")
                return
            
            if not TORCH_AVAILABLE:
                self.get_logger().error("PyTorch not available")
                return
            
            # Solve IK to get joint positions
            pos_tensor = torch.tensor(position, dtype=torch.float32)
            quat_tensor = torch.tensor(orientation, dtype=torch.float32)
            curr_joints = self._robot.get_joint_positions()
            
            desired_joints, ik_success = self._robot.solve_inverse_kinematics(
                pos_tensor, quat_tensor, curr_joints
            )
            
            if not ik_success:
                self.get_logger().warn("IK solution not found for cartesian command")
                return
            
            if msg.blocking:
                def blocking_move():
                    try:
                        if self._robot.is_running_policy():
                            self._robot.terminate_current_policy()
                        
                        time_to_go = msg.time_to_go if msg.time_to_go > 0 else 2.0
                        self._robot.move_to_joint_positions(desired_joints, time_to_go=time_to_go)
                        self._robot.start_cartesian_impedance()
                        self._controller_mode = "cartesian_impedance"
                    except Exception as e:
                        self.get_logger().error(f"Blocking cartesian move error: {e}")
                
                threading.Thread(target=blocking_move, daemon=True).start()
            else:
                def helper_non_blocking():
                    try:
                        if not self._robot.is_running_policy():
                            self._controller_not_loaded = True
                            self._robot.start_cartesian_impedance()
                            self._controller_mode = "cartesian_impedance"
                            
                            timeout = time.time() + 5
                            while not self._robot.is_running_policy():
                                time.sleep(0.01)
                                if time.time() > timeout:
                                    self._robot.start_cartesian_impedance()
                                    timeout = time.time() + 5
                            
                            self._controller_not_loaded = False
                        
                        self._robot.update_desired_joint_positions(desired_joints)
                    except Exception as e:
                        self.get_logger().error(f"Non-blocking cartesian command error: {e}")
                        self._controller_not_loaded = False
                
                threading.Thread(target=helper_non_blocking, daemon=True).start()
        except Exception as e:
            self.get_logger().error(f"Error processing cartesian position command: {e}")
    
    def _cartesian_velocity_command_callback(self, msg: CartesianVelocityCommand):
        """Callback for cartesian velocity commands."""
        if self._controller_not_loaded:
            return
        
        try:
            linear_vel = list(msg.linear_velocity)
            angular_vel = list(msg.angular_velocity)
            
            if len(linear_vel) != 3 or len(angular_vel) != 3:
                self.get_logger().error(f"Invalid cartesian velocity: linear={len(linear_vel)}, angular={len(angular_vel)}")
                return
            
            if not TORCH_AVAILABLE or self._ik_solver is None:
                self.get_logger().error("PyTorch or IK solver not available for cartesian velocity")
                return
            
            # Convert cartesian velocity to joint velocity using Jacobian
            # For simplicity, we use IK solver's velocity IK method if available
            # Otherwise, fall back to position-based control
            self.get_logger().warn("Cartesian velocity control: using position-based approximation")
            
            # Get current pose and apply velocity delta
            joint_pos = self._robot.get_joint_positions()
            pos, quat = self._compute_forward_kinematics(joint_pos)
            
            dt = 1.0 / CONTROL_HZ
            new_pos = pos + np.array(linear_vel) * dt
            
            # For angular velocity, convert to quaternion delta (simplified)
            # This is an approximation
            new_pos_tensor = torch.tensor(new_pos, dtype=torch.float32)
            quat_tensor = torch.tensor(quat, dtype=torch.float32)
            
            desired_joints, ik_success = self._robot.solve_inverse_kinematics(
                new_pos_tensor, quat_tensor, joint_pos
            )
            
            if ik_success:
                self._robot.update_desired_joint_positions(desired_joints)
        except Exception as e:
            self.get_logger().error(f"Error processing cartesian velocity command: {e}")
    
    def _reset_robot(self, randomize: bool = False):
        """Reset robot to home position."""
        try:
            self.get_logger().info(f"Resetting robot to home position (randomize={randomize})...")
            
            target_joints = self._reset_joints.copy()
            
            if TORCH_AVAILABLE:
                target_joints_tensor = torch.tensor(target_joints, dtype=torch.float32)
                if not isinstance(self._robot, MockRobotInterface):
                    if self._robot.is_running_policy():
                        self._robot.terminate_current_policy()
                    
                    curr_joints = self._robot.get_joint_positions()
                    displacement = torch.abs(target_joints_tensor - curr_joints)
                    max_displacement = torch.max(displacement).item()
                    time_to_go = min(MAX_TIME_TO_GO_SLOW, max(MIN_TIME_TO_GO_SLOW, max_displacement / MAX_VELOCITY_SLOW))
                    
                    self._robot.move_to_joint_positions(target_joints_tensor, time_to_go=time_to_go)
                    self._robot.start_joint_impedance()
                    self._controller_mode = "joint_impedance"
                else:
                    self._robot.move_to_joint_positions(target_joints_tensor, time_to_go=5.0)
            
            self.get_logger().info("Robot reset to home position successfully")
        except Exception as e:
            self.get_logger().error(f"Reset error: {e}\n{traceback.format_exc()}")
    
    def _reset_service_callback(self, request, response):
        """Service callback for reset."""
        try:
            randomize = getattr(request, 'randomize', False)
            
            def reset_thread():
                try:
                    self._reset_robot(randomize=randomize)
                except Exception as e:
                    self.get_logger().error(f"Reset thread error: {e}")
            
            threading.Thread(target=reset_thread, daemon=True).start()
            
            response.success = True
            response.message = "Reset command accepted"
        except Exception as e:
            response.success = False
            response.message = f"Reset failed: {str(e)}"
        
        return response
    
    def _move_to_joint_positions_callback(self, request, response):
        """Service callback for moving to joint positions (BLOCKING)."""
        try:
            if len(request.joint_positions) != 7:
                response.success = False
                response.message = f"Invalid length: {len(request.joint_positions)}, expected 7"
                return response
            
            if not TORCH_AVAILABLE:
                response.success = False
                response.message = "PyTorch not available"
                return response
            
            pos_tensor = torch.tensor(request.joint_positions, dtype=torch.float32)
            
            if isinstance(self._robot, MockRobotInterface):
                time_to_go = request.time_to_go if request.time_to_go > 0 else 2.0
                self._robot.move_to_joint_positions(pos_tensor, time_to_go=time_to_go)
                response.success = True
                response.message = "Moved to joint positions"
                response.final_joint_positions = request.joint_positions
            else:
                if self._robot.is_running_policy():
                    self._robot.terminate_current_policy()
                
                if request.time_to_go > 0:
                    time_to_go = request.time_to_go
                else:
                    curr_pos = self._robot.get_joint_positions()
                    displacement = torch.abs(pos_tensor - curr_pos)
                    max_displacement = torch.max(displacement).item()
                    time_to_go = min(MAX_TIME_TO_GO, max(MIN_TIME_TO_GO, max_displacement / MAX_VELOCITY_FOR_TIME_CALC))
                
                self._robot.move_to_joint_positions(pos_tensor, time_to_go=time_to_go)
                self._robot.start_cartesian_impedance()
                self._controller_mode = "cartesian_impedance"
                
                final_pos = self._robot.get_joint_positions()
                if isinstance(final_pos, torch.Tensor):
                    response.final_joint_positions = final_pos.cpu().numpy().tolist()
                else:
                    response.final_joint_positions = list(final_pos)
                
                response.success = True
                response.message = f"Moved to joint positions (time_to_go={time_to_go:.1f}s)"
        except Exception as e:
            response.success = False
            response.message = f"Failed: {str(e)}"
            response.final_joint_positions = []
        
        return response
    
    def _move_to_ee_pose_callback(self, request, response):
        """Service callback for moving to end-effector pose (BLOCKING)."""
        try:
            if len(request.position) != 3 or len(request.orientation) != 4:
                response.success = False
                response.message = "Invalid position/orientation length"
                return response
            
            if isinstance(self._robot, MockRobotInterface):
                response.success = True
                response.message = "Mock: Moved to EE pose"
                response.final_joint_positions = [0.0] * 7
                return response
            
            if not TORCH_AVAILABLE:
                response.success = False
                response.message = "PyTorch not available"
                return response
            
            pos = torch.tensor(request.position, dtype=torch.float32)
            quat = torch.tensor(request.orientation, dtype=torch.float32)
            
            q0 = torch.tensor(request.q0, dtype=torch.float32) if len(request.q0) == 7 else self._robot.get_joint_positions()
            
            desired_joints, ik_success = self._robot.solve_inverse_kinematics(pos, quat, q0)
            
            if not ik_success:
                response.success = False
                response.message = "IK solution not found"
                response.final_joint_positions = []
                return response
            
            if self._robot.is_running_policy():
                self._robot.terminate_current_policy()
            
            if request.time_to_go > 0:
                time_to_go = request.time_to_go
            else:
                curr_pos = self._robot.get_joint_positions()
                displacement = torch.abs(desired_joints - curr_pos)
                max_displacement = torch.max(displacement).item()
                time_to_go = min(MAX_TIME_TO_GO, max(MIN_TIME_TO_GO, max_displacement / MAX_VELOCITY_FOR_TIME_CALC))
            
            self._robot.move_to_joint_positions(desired_joints, time_to_go=time_to_go)
            self._robot.start_cartesian_impedance()
            self._controller_mode = "cartesian_impedance"
            
            final_pos = self._robot.get_joint_positions()
            if isinstance(final_pos, torch.Tensor):
                response.final_joint_positions = final_pos.cpu().numpy().tolist()
            else:
                response.final_joint_positions = list(final_pos)
            
            response.success = True
            response.message = f"Moved to EE pose (time_to_go={time_to_go:.1f}s)"
        except Exception as e:
            response.success = False
            response.message = f"Failed: {str(e)}"
            response.final_joint_positions = []
        
        return response
    
    def _start_joint_impedance_callback(self, request, response):
        """Service callback for starting joint impedance controller."""
        try:
            if isinstance(self._robot, MockRobotInterface):
                self._controller_mode = "joint_impedance"
                response.success = True
                response.message = "Mock: Joint impedance started"
            else:
                if TORCH_AVAILABLE:
                    kq = torch.tensor(request.kq, dtype=torch.float32) if len(request.kq) > 0 else None
                    kqd = torch.tensor(request.kqd, dtype=torch.float32) if len(request.kqd) > 0 else None
                    
                    if self._robot.is_running_policy():
                        self._robot.terminate_current_policy()
                    
                    if kq is not None and kqd is not None:
                        self._robot.start_joint_impedance(Kq=kq, Kqd=kqd)
                        self._current_kp = kq
                        self._current_kd = kqd
                    else:
                        self._robot.start_joint_impedance()
                    
                    self._controller_mode = "joint_impedance"
                    response.success = True
                    response.message = "Joint impedance controller started"
                else:
                    response.success = False
                    response.message = "PyTorch not available"
        except Exception as e:
            response.success = False
            response.message = f"Failed: {str(e)}"
        
        return response
    
    def _start_cartesian_impedance_callback(self, request, response):
        """Service callback for starting cartesian impedance controller."""
        try:
            if isinstance(self._robot, MockRobotInterface):
                self._controller_mode = "cartesian_impedance"
                response.success = True
                response.message = "Mock: Cartesian impedance started"
            else:
                if TORCH_AVAILABLE:
                    kx = torch.tensor(request.kx, dtype=torch.float32) if len(request.kx) > 0 else None
                    kxd = torch.tensor(request.kxd, dtype=torch.float32) if len(request.kxd) > 0 else None
                    
                    if self._robot.is_running_policy():
                        self._robot.terminate_current_policy()
                    
                    if kx is not None and kxd is not None:
                        self._robot.start_cartesian_impedance(Kx=kx, Kxd=kxd)
                        self._current_kp = kx
                        self._current_kd = kxd
                    else:
                        self._robot.start_cartesian_impedance()
                    
                    self._controller_mode = "cartesian_impedance"
                    response.success = True
                    response.message = "Cartesian impedance controller started"
                else:
                    response.success = False
                    response.message = "PyTorch not available"
        except Exception as e:
            response.success = False
            response.message = f"Failed: {str(e)}"
        
        return response
    
    def _start_joint_velocity_callback(self, request, response):
        """Service callback for starting joint velocity controller."""
        try:
            if isinstance(self._robot, MockRobotInterface):
                self._controller_mode = "joint_velocity"
                response.success = True
                response.message = "Mock: Joint velocity started"
            else:
                if TORCH_AVAILABLE:
                    qd_init = torch.tensor(request.qd_init, dtype=torch.float32) if len(request.qd_init) > 0 else None
                    
                    if self._robot.is_running_policy():
                        self._robot.terminate_current_policy()
                    
                    self._robot.start_joint_velocity_control(qd_init=qd_init)
                    self._controller_mode = "joint_velocity"
                    response.success = True
                    response.message = "Joint velocity controller started"
                else:
                    response.success = False
                    response.message = "PyTorch not available"
        except Exception as e:
            response.success = False
            response.message = f"Failed: {str(e)}"
        
        return response
    
    def _terminate_policy_callback(self, request, response):
        """Service callback for terminating current policy."""
        try:
            if isinstance(self._robot, MockRobotInterface):
                self._robot.terminate_current_policy()
                response.success = True
                response.message = "Mock: Policy terminated"
            else:
                if self._robot.is_running_policy():
                    self._robot.terminate_current_policy()
                    self._controller_mode = "none"
                    response.success = True
                    response.message = "Policy terminated"
                else:
                    response.success = True
                    response.message = "No policy running"
        except Exception as e:
            response.success = False
            response.message = f"Failed: {str(e)}"
        
        return response
    
    def _solve_ik_callback(self, request, response):
        """Service callback for solving inverse kinematics."""
        try:
            if len(request.position) != 3 or len(request.orientation) != 4 or len(request.q0) != 7:
                response.success = False
                response.message = "Invalid input lengths"
                return response
            
            if isinstance(self._robot, MockRobotInterface):
                response.success = True
                response.message = "Mock: IK solved"
                response.joint_positions = request.q0
                response.error = 0.0
            else:
                if TORCH_AVAILABLE:
                    pos = torch.tensor(request.position, dtype=torch.float32)
                    quat = torch.tensor(request.orientation, dtype=torch.float32)
                    q0 = torch.tensor(request.q0, dtype=torch.float32)
                    tolerance = request.tolerance if request.tolerance > 0 else 1e-3
                    
                    joint_pos, success = self._robot.solve_inverse_kinematics(pos, quat, q0, tol=tolerance)
                    
                    if isinstance(joint_pos, torch.Tensor):
                        response.joint_positions = joint_pos.cpu().numpy().tolist()
                    else:
                        response.joint_positions = list(joint_pos)
                    
                    response.success = bool(success)
                    response.message = "IK solved" if success else "IK solution not found"
                    response.error = 0.0
                else:
                    response.success = False
                    response.message = "PyTorch not available"
                    response.joint_positions = []
                    response.error = 1.0
        except Exception as e:
            response.success = False
            response.message = f"Failed: {str(e)}"
            response.joint_positions = []
            response.error = 1.0
        
        return response
    
    def _compute_fk_callback(self, request, response):
        """Service callback for computing forward kinematics."""
        try:
            if len(request.joint_positions) != 7:
                response.success = False
                response.message = "Invalid joint positions length"
                return response
            
            pos, quat = self._compute_forward_kinematics(request.joint_positions)
            response.position = pos.tolist() if hasattr(pos, 'tolist') else list(pos)
            response.orientation = quat.tolist() if hasattr(quat, 'tolist') else list(quat)
            response.success = True
            response.message = "FK computed successfully"
        except Exception as e:
            response.success = False
            response.message = f"Failed: {str(e)}"
        
        return response
    
    def _compute_time_to_go_callback(self, request, response):
        """Service callback for computing adaptive time to go."""
        try:
            if len(request.desired_joint_positions) != 7:
                response.success = False
                response.message = "Invalid joint positions length"
                return response
            
            if isinstance(self._robot, MockRobotInterface):
                response.success = True
                response.time_to_go = 2.0
            else:
                if TORCH_AVAILABLE:
                    desired_pos = torch.tensor(request.desired_joint_positions, dtype=torch.float32)
                    curr_pos = self._robot.get_joint_positions()
                    displacement = desired_pos - curr_pos
                    max_displacement = torch.max(torch.abs(displacement)).item()
                    response.time_to_go = min(MAX_TIME_TO_GO, max(MIN_TIME_TO_GO, max_displacement / MAX_VELOCITY_FOR_TIME_CALC))
                    response.success = True
                else:
                    desired_pos = np.array(request.desired_joint_positions)
                    curr_pos = np.array(self._robot.get_joint_positions())
                    displacement = np.abs(desired_pos - curr_pos)
                    max_displacement = np.max(displacement)
                    response.time_to_go = min(MAX_TIME_TO_GO, max(MIN_TIME_TO_GO, max_displacement / MAX_VELOCITY_FOR_TIME_CALC))
                    response.success = True
        except Exception as e:
            response.success = False
            response.message = f"Failed: {str(e)}"
            response.time_to_go = 0.0
        
        return response
    
    def destroy_node(self):
        """Clean up resources on shutdown."""
        try:
            if self._robot and not isinstance(self._robot, MockRobotInterface):
                if self._robot.is_running_policy():
                    self._robot.terminate_current_policy()
        except Exception as e:
            self.get_logger().warn(f"Error during node destruction: {e}")
        finally:
            super().destroy_node()


def main(args=None):
    """Main function to run the Franka Robot Interface Node."""
    from rclpy.executors import MultiThreadedExecutor
    
    rclpy.init(args=args)
    
    # Get namespace from command line or environment
    namespace = os.environ.get('ROBOT_NAMESPACE', 'fr3_arm')
    
    node = FrankaRobotInterfaceNode(
        use_mock=not POLYMETIS_AVAILABLE,
        namespace=namespace
    )
    
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

