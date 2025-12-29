#!/usr/bin/env python3
"""
Polymetis Bridge Node - ROS 2 Driver for Polymetis Framework

This node bridges the Polymetis Python SDK (RobotInterface and GripperInterface) with ROS 2.
It serves as a hardware interface replacement for franka_ros.

Features:
- Publisher for /joint_states (arm+gripper) for tf tree creation
- Publishers under /polymetis/ namespace:
  - /polymetis/robot_state: Comprehensive robot state (similar to robot.py get_robot_state())
  - /polymetis/gripper_state: Gripper state information
- Subscribers:
  - /polymetis_cmd: Low-level impedance control (position, velocity, torque, Kp, Kd)
  - /polymetis/robot_command: High-level control commands (cartesian/joint position/velocity)
- Mock interfaces for testing without hardware
- Proper PyTorch tensor conversion for Polymetis API
- IK solver integration for high-level control

Author: Role-ROS2 Team
"""

import time
import threading
import os
import sys
import yaml
import traceback
import subprocess
import signal
from pathlib import Path
from typing import Optional, List
import numpy as np

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
    from polymetis import RobotInterface, GripperInterface
    POLYMETIS_AVAILABLE = True
except ImportError:
    POLYMETIS_AVAILABLE = False
    print("Warning: Polymetis not available. Using MockRobotInterface and MockGripperInterface.")

# Import custom messages
from role_ros2.msg import (
    PolymetisCommand, PolymetisRobotState, PolymetisRobotCommand, 
    PolymetisGripperState, GripperCommand, ControllerStatus
)

# Import services
from role_ros2.srv import (
    Reset, StartCartesianImpedance, StartJointImpedance,
    TerminatePolicy, MoveToJointPositions, MoveToEEPose,
    SolveIK, ComputeFK, ComputeTimeToGo
)

# Import IK solver and transformations
try:
    from role_ros2.robot_ik.robot_ik_solver import RobotIKSolver
    from role_ros2.misc.transformations import add_poses, euler_to_quat, pose_diff, quat_to_euler
    IK_SOLVER_AVAILABLE = True
except ImportError as e:
    IK_SOLVER_AVAILABLE = False
    print(f"Warning: RobotIKSolver not available: {e}. High-level control commands will not work.")


def load_joint_names_from_config(config_file=None):
    """
    Load joint names from config/franka_robot_config.yaml.
    
    If file not found or joint names don't match arm_id, raise error.
    
    Args:
        config_file: Path to config file. If None, use ROS2 package share directory.
    
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
    
    # Extract joint names with type checking
    arm_joints = config.get('arm_joints', [])
    gripper_joints = config.get('gripper_joints', [])
    arm_id = config.get('arm_id', '')
    
    # Validate types
    if not isinstance(arm_joints, list):
        raise ValueError(
            f"arm_joints must be a list, got {type(arm_joints).__name__}: {arm_joints}"
        )
    if not isinstance(gripper_joints, list):
        raise ValueError(
            f"gripper_joints must be a list, got {type(gripper_joints).__name__}: {gripper_joints}"
        )
    if not isinstance(arm_id, str):
        raise ValueError(
            f"arm_id must be a string, got {type(arm_id).__name__}: {arm_id}"
        )
    
    # Validate
    if not arm_id:
        raise ValueError("arm_id is missing or empty in franka_robot_config.yaml")
    
    if not arm_joints:
        raise ValueError("arm_joints is empty or missing in franka_robot_config.yaml")
    
    if not gripper_joints:
        raise ValueError("gripper_joints is empty or missing in franka_robot_config.yaml")
    
    # Validate joint name format (must match arm_id prefix)
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
    
    return arm_joints, gripper_joints


class MockRobotInterface:
    """
    Mock implementation of Polymetis RobotInterface for testing without hardware.
    
    Mimics the behavior of polymetis.RobotInterface, allowing development and testing
    without physical robot hardware.
    """
    
    def __init__(self, num_dofs: int = 7):
        """
        Initialize mock robot interface.
        
        Args:
            num_dofs: Number of degrees of freedom (default: 7 for Franka arm)
        """
        self.num_dofs = num_dofs
        if TORCH_AVAILABLE:
            self._joint_positions = torch.zeros(num_dofs, dtype=torch.float32)
            self._joint_velocities = torch.zeros(num_dofs, dtype=torch.float32)
            self._kp = torch.ones(num_dofs, dtype=torch.float32) * 40.0
            self._kd = torch.ones(num_dofs, dtype=torch.float32) * 4.0
        else:
            self._joint_positions = np.zeros(num_dofs)
            self._joint_velocities = np.zeros(num_dofs)
            self._kp = np.ones(num_dofs) * 40.0
            self._kd = np.ones(num_dofs) * 4.0
        
        self._is_running_policy = False
    
    def send_torch_policy(
        self,
        q: Optional[List[float]] = None,
        qd: Optional[List[float]] = None,
        Kq: Optional[List[float]] = None,
        Kqd: Optional[List[float]] = None,
        tau: Optional[List[float]] = None
    ):
        """
        Send impedance control command (mock implementation).
        
        In real RobotInterface, this would create and send a TorchScript policy.
        For mock, we directly update the internal state.
        
        Args:
            q: Desired joint positions (7 DOF) in radians
            qd: Desired joint velocities (7 DOF) in rad/s
            Kq: Position gains (stiffness) for each joint
            Kqd: Velocity gains (damping) for each joint
            tau: Desired joint torques (7 DOF) in Nm
        """
        if TORCH_AVAILABLE:
            if q is not None:
                self._joint_positions = torch.tensor(q, dtype=torch.float32)
            if qd is not None:
                self._joint_velocities = torch.tensor(qd, dtype=torch.float32)
            if Kq is not None:
                self._kp = torch.tensor(Kq, dtype=torch.float32)
            if Kqd is not None:
                self._kd = torch.tensor(Kqd, dtype=torch.float32)
        else:
            if q is not None:
                self._joint_positions = np.array(q)
            if qd is not None:
                self._joint_velocities = np.array(qd)
            if Kq is not None:
                self._kp = np.array(Kq)
            if Kqd is not None:
                self._kd = np.array(Kqd)
        
        self._is_running_policy = True
    
    def get_joint_positions(self):
        """Get current joint positions."""
        return self._joint_positions
    
    def get_joint_velocities(self):
        """Get current joint velocities."""
        return self._joint_velocities
    
    def get_robot_state(self):
        """Get robot state (mock implementation)."""
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
        
        return MockRobotState(self._joint_positions, self._joint_velocities)
    
    def is_running_policy(self) -> bool:
        """Check if a policy is currently running."""
        return self._is_running_policy
    
    def start_joint_impedance(self, Kq=None, Kqd=None):
        """Start joint impedance control mode."""
        if Kq is not None:
            if TORCH_AVAILABLE:
                self._kp = torch.tensor(Kq, dtype=torch.float32)
            else:
                self._kp = np.array(Kq)
        if Kqd is not None:
            if TORCH_AVAILABLE:
                self._kd = torch.tensor(Kqd, dtype=torch.float32)
            else:
                self._kd = np.array(Kqd)
        self._is_running_policy = True
    
    def update_desired_joint_positions(self, positions):
        """Update desired joint positions (non-blocking)."""
        if TORCH_AVAILABLE:
            if isinstance(positions, torch.Tensor):
                self._joint_positions = positions.clone()
            else:
                self._joint_positions = torch.tensor(positions, dtype=torch.float32)
        else:
            self._joint_positions = np.array(positions)
    
    def move_to_joint_positions(self, positions, time_to_go: float = 2.0):
        """Move to joint positions (blocking)."""
        if TORCH_AVAILABLE:
            if isinstance(positions, torch.Tensor):
                self._joint_positions = positions.clone()
            else:
                self._joint_positions = torch.tensor(positions, dtype=torch.float32)
        else:
            self._joint_positions = np.array(positions)
        time.sleep(time_to_go)
    
    def terminate_current_policy(self):
        """Terminate current policy."""
        self._is_running_policy = False
    
    def solve_inverse_kinematics(self, pos, quat, curr_joints):
        """Solve inverse kinematics (mock implementation)."""
        # Simple mock: return current joints (real IK would compute desired joints)
        if TORCH_AVAILABLE:
            if isinstance(curr_joints, torch.Tensor):
                return curr_joints.clone()
            else:
                return torch.tensor(curr_joints, dtype=torch.float32)
        else:
            return np.array(curr_joints)
    
    def get_ee_pose(self):
        """Get end-effector pose (mock implementation)."""
        # Return default pose
        if TORCH_AVAILABLE:
            pos = torch.zeros(3, dtype=torch.float32)
            quat = torch.tensor([0.0, 0.0, 0.0, 1.0], dtype=torch.float32)  # [x, y, z, w]
        else:
            pos = np.zeros(3)
            quat = np.array([0.0, 0.0, 0.0, 1.0])
        return pos, quat


class MockGripperInterface:
    """
    Mock implementation of Polymetis GripperInterface for testing without hardware.
    """
    
    def __init__(self):
        """Initialize mock gripper interface."""
        self._width = 0.08  # Max width in meters
        self._is_moving = False
        self.metadata = type('obj', (object,), {'max_width': 0.08})()
    
    def goto(self, width: float, speed: float = 0.05, force: float = 0.1, blocking: bool = False):
        """
        Move gripper to specified width.
        
        Args:
            width: Target width in meters (0=closed, max_width=open)
            speed: Movement speed (0.0-1.0)
            force: Gripper force (0.0-1.0)
            blocking: Whether to wait for completion
        """
        self._width = max(0.0, min(width, self.metadata.max_width))
        self._is_moving = True
        if blocking:
            time.sleep(0.5)  # Simulate movement time
        self._is_moving = False
    
    def get_state(self):
        """Get gripper state."""
        class MockGripperState:
            def __init__(self, width, is_moving):
                self.width = width
                self.is_moving = is_moving
                self.is_grasped = width < 0.01
                self.prev_command_successful = True
                self.error_code = 0
        
        return MockGripperState(self._width, self._is_moving)


class PolymetisCombinedNode(Node):
    """
    ROS 2 Node that bridges Polymetis Python SDK with ROS 2.
    
    This node:
    - Subscribes to /polymetis_cmd for impedance control commands
    - Publishes /joint_states at high frequency (50Hz+)
    - Handles both Arm and Gripper control
    - Converts ROS messages to PyTorch tensors for Polymetis API
    """
    
    def __init__(self, use_mock: bool = False, ip_address: str = "localhost"):
        """
        Initialize the Polymetis Combined Node.
        
        Args:
            use_mock: If True, use MockRobotInterface instead of real hardware
            ip_address: IP address of Polymetis server (default: "localhost")
        """
        super().__init__('polymetis_combined_node')
        
        # Declare parameters
        self.declare_parameter('use_mock', use_mock)
        self.declare_parameter('ip_address', ip_address)
        self.declare_parameter('publish_rate', 50.0)  # Hz
        self.declare_parameter('auto_launch_controller', True)  # Auto-launch robot/gripper servers
        self.declare_parameter('sudo_password', '')  # Sudo password for launching servers (empty = no sudo)
        self.declare_parameter('robot_ip', '172.17.0.2')  # Robot IP for launch scripts
        self.declare_parameter('auto_reset_on_startup', False)  # Auto-reset robot on startup
        self.declare_parameter('auto_reset_delay', 5.0)  # Delay before auto-reset (seconds)
        
        # Load joint names from config file (required, will raise error if not found or invalid)
        try:
            arm_joints, gripper_joints = load_joint_names_from_config()
            self.get_logger().info(
                f"Loaded joint names from config: {len(arm_joints)} arm, {len(gripper_joints)} gripper"
            )
        except (FileNotFoundError, ValueError) as e:
            # Use both print and logger for better error visibility
            error_msg = f"Failed to load joint names: {e}"
            print(f"[ERROR] {error_msg}", file=sys.stderr)
            try:
                self.get_logger().error(error_msg)
            except:
                pass  # Logger might not be available in some cases
            raise  # Re-raise error to prevent node from starting with invalid config
        
        self.declare_parameter('arm_joint_names', arm_joints)
        self.declare_parameter('gripper_joint_names', gripper_joints)
        
        # Get parameters (ROS parameters override config file)
        use_mock = self.get_parameter('use_mock').get_parameter_value().bool_value
        ip_address = self.get_parameter('ip_address').get_parameter_value().string_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.arm_joint_names = self.get_parameter('arm_joint_names').get_parameter_value().string_array_value
        self.gripper_joint_names = self.get_parameter('gripper_joint_names').get_parameter_value().string_array_value
        self.all_joint_names = self.arm_joint_names + self.gripper_joint_names
        auto_launch_controller = self.get_parameter('auto_launch_controller').get_parameter_value().bool_value
        sudo_password = self.get_parameter('sudo_password').get_parameter_value().string_value
        robot_ip = self.get_parameter('robot_ip').get_parameter_value().string_value
        
        # Store for cleanup
        self._robot_process = None
        self._gripper_process = None
        self._server_launched = False
        self._sudo_password = sudo_password
        self._robot_ip = robot_ip
        
        self.get_logger().info(f"Using joint names - Arm: {self.arm_joint_names}")
        self.get_logger().info(f"Using joint names - Gripper: {self.gripper_joint_names}")
        
        # Launch robot and gripper servers if needed (before connecting)
        if not use_mock and POLYMETIS_AVAILABLE and auto_launch_controller:
            try:
                self._launch_controller()
            except Exception as e:
                self.get_logger().error(f"Failed to launch controller: {e}. Continuing anyway...")
        
        # Initialize robot and gripper interfaces
        self._robot: Optional[RobotInterface] = None
        self._gripper: Optional[GripperInterface] = None
        
        try:
            if use_mock or not POLYMETIS_AVAILABLE:
                self.get_logger().info("Using MockRobotInterface and MockGripperInterface (no hardware)")
                self._robot = MockRobotInterface(num_dofs=7)
                self._gripper = MockGripperInterface()
            else:
                self.get_logger().info(f"Connecting to Polymetis server at {ip_address}")
                # Wait a bit for servers to start if we just launched them
                if self._server_launched:
                    self.get_logger().info("Waiting for Polymetis servers to initialize...")
                    time.sleep(5)
                self._robot = RobotInterface(ip_address=ip_address)
                self._gripper = GripperInterface(ip_address=ip_address)
                self.get_logger().info("Successfully connected to Polymetis server")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to Polymetis: {e}. Using mock interfaces.")
            self._robot = MockRobotInterface(num_dofs=7)
            self._gripper = MockGripperInterface()
        
        # Initialize IK solver if available
        self._ik_solver = None
        if IK_SOLVER_AVAILABLE:
            try:
                self._ik_solver = RobotIKSolver()
                self.get_logger().info("RobotIKSolver initialized successfully")
            except Exception as e:
                self.get_logger().warn(f"Failed to initialize RobotIKSolver: {e}")
        
        # Get max gripper width
        try:
            self._max_gripper_width = self._gripper.metadata.max_width
        except AttributeError:
            self._max_gripper_width = 0.08  # Default for Franka hand
        
        # State storage and thread safety
        self._command_lock = threading.Lock()
        self._last_command_time = None
        
        # Callback groups for parallel execution with MultiThreadedExecutor
        # ReentrantCallbackGroup: allows timer to run in parallel with services
        # MutuallyExclusiveCallbackGroup: blocking services execute one at a time
        self._timer_callback_group = ReentrantCallbackGroup()
        self._service_callback_group = MutuallyExclusiveCallbackGroup()
        
        # Subscriber for impedance commands (low-level)
        self._cmd_subscriber = self.create_subscription(
            PolymetisCommand,
            '/polymetis_cmd',
            self._command_callback,
            10
        )
        
        # Subscriber for high-level control commands (similar to update_command)
        self._robot_cmd_subscriber = self.create_subscription(
            PolymetisRobotCommand,
            '/polymetis/robot_command',
            self._robot_command_callback,
            10
        )
        
        # Publisher for joint states (combines arm + gripper) - for tf tree
        self._joint_state_publisher = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )
        
        # Publishers under /polymetis/ namespace
        self._robot_state_publisher = self.create_publisher(
            PolymetisRobotState,
            '/polymetis/robot_state',
            10
        )
        
        self._gripper_state_publisher = self.create_publisher(
            PolymetisGripperState,
            '/polymetis/gripper_state',
            10
        )
        
        # New publishers for additional state information
        self._ee_pose_publisher = self.create_publisher(
            PoseStamped,
            '/polymetis/arm/ee_pose',
            10
        )
        
        self._controller_status_publisher = self.create_publisher(
            ControllerStatus,
            '/polymetis/controller/status',
            10
        )
        
        # Subscriber for gripper commands
        self._gripper_cmd_subscriber = self.create_subscription(
            GripperCommand,
            '/polymetis/gripper/command',
            self._gripper_command_callback,
            10
        )
        
        # Timer for publishing joint states and robot state at high frequency
        # Uses ReentrantCallbackGroup so it can run in parallel with blocking services
        timer_period = 1.0 / self.publish_rate  # Convert Hz to seconds
        self._state_timer = self.create_timer(
            timer_period, 
            self._publish_states,
            callback_group=self._timer_callback_group
        )
        
        # Service server for reset (uses separate callback group for blocking operations)
        self._reset_service = self.create_service(
            Reset, 
            '/polymetis/reset', 
            self._reset_service_callback,
            callback_group=self._service_callback_group
        )
        
        # Service servers for controller management
        self._start_cartesian_impedance_service = self.create_service(
            StartCartesianImpedance,
            '/polymetis/arm/start_cartesian_impedance',
            self._start_cartesian_impedance_callback,
            callback_group=self._service_callback_group
        )
        
        self._start_joint_impedance_service = self.create_service(
            StartJointImpedance,
            '/polymetis/arm/start_joint_impedance',
            self._start_joint_impedance_callback,
            callback_group=self._service_callback_group
        )
        
        self._terminate_policy_service = self.create_service(
            TerminatePolicy,
            '/polymetis/arm/terminate_policy',
            self._terminate_policy_callback,
            callback_group=self._service_callback_group
        )
        
        # Service servers for motion control (BLOCKING - uses separate callback group)
        self._move_to_joint_positions_service = self.create_service(
            MoveToJointPositions,
            '/polymetis/arm/move_to_joint_positions',
            self._move_to_joint_positions_callback,
            callback_group=self._service_callback_group
        )
        
        self._move_to_ee_pose_service = self.create_service(
            MoveToEEPose,
            '/polymetis/arm/move_to_ee_pose',
            self._move_to_ee_pose_callback,
            callback_group=self._service_callback_group
        )
        
        # Service servers for computation (fast, non-blocking)
        self._solve_ik_service = self.create_service(
            SolveIK,
            '/polymetis/arm/solve_ik',
            self._solve_ik_callback,
            callback_group=self._service_callback_group
        )
        
        self._compute_fk_service = self.create_service(
            ComputeFK,
            '/polymetis/arm/compute_fk',
            self._compute_fk_callback,
            callback_group=self._service_callback_group
        )
        
        self._compute_time_to_go_service = self.create_service(
            ComputeTimeToGo,
            '/polymetis/arm/compute_time_to_go',
            self._compute_time_to_go_callback,
            callback_group=self._service_callback_group
        )
        
        # Store controller mode for status publishing
        self._controller_mode = "none"
        self._current_kp = None
        self._current_kd = None
        
        # Controller management for non-blocking mode
        self._controller_not_loaded = False
        self._controller_ensure_thread = None
        self._controller_ensure_lock = threading.Lock()
        
        # Default reset/home joint positions (same as robot_env.py)
        self._reset_joints = np.array([0, -1 / 5 * np.pi, 0, -4 / 5 * np.pi, 0, 3 / 5 * np.pi, 0.0])
        
        # Start impedance controller for continuous control
        self._start_impedance_controller()
        
        # Declare ROS2 parameters for metadata
        self.declare_parameter('gripper.max_width', self._max_gripper_width)
        self.declare_parameter('gripper.min_width', 0.0)
        self.declare_parameter('arm.num_dofs', 7)
        self.declare_parameter('arm.joint_names', self.arm_joint_names)
        
        # Try to get metadata from robot interface and set parameters
        if not isinstance(self._robot, MockRobotInterface) and hasattr(self._robot, 'metadata'):
            try:
                metadata = self._robot.metadata
                if hasattr(metadata, 'default_Kq'):
                    self.declare_parameter('arm.default_Kq', list(metadata.default_Kq))
                if hasattr(metadata, 'default_Kqd'):
                    self.declare_parameter('arm.default_Kqd', list(metadata.default_Kqd))
                if hasattr(metadata, 'rest_pose'):
                    self.declare_parameter('arm.rest_pose', list(metadata.rest_pose))
                if hasattr(metadata, 'hz'):
                    self.declare_parameter('arm.hz', metadata.hz)
            except Exception as e:
                self.get_logger().warn(f"Failed to get robot metadata: {e}")
        
        self.get_logger().info(
            f'PolymetisCombinedNode initialized. Publishing states at {self.publish_rate} Hz'
        )
        self.get_logger().info('Publishers: /joint_states, /polymetis/robot_state, /polymetis/gripper_state, /polymetis/arm/ee_pose, /polymetis/controller/status')
        self.get_logger().info('Subscribers: /polymetis_cmd, /polymetis/robot_command, /polymetis/gripper/command')
        self.get_logger().info('Services: /polymetis/reset, /polymetis/arm/start_cartesian_impedance, /polymetis/arm/start_joint_impedance, /polymetis/arm/terminate_policy, /polymetis/arm/move_to_joint_positions, /polymetis/arm/move_to_ee_pose, /polymetis/arm/solve_ik, /polymetis/arm/compute_fk, /polymetis/arm/compute_time_to_go')
        
        # Auto-reset robot on startup (delayed to avoid SIGSEGV and wait for servers)
        auto_reset_on_startup = self.get_parameter('auto_reset_on_startup').get_parameter_value().bool_value
        auto_reset_delay = self.get_parameter('auto_reset_delay').get_parameter_value().double_value
        
        if auto_reset_on_startup:
            self.get_logger().info(f"Auto-reset enabled. Will reset robot after {auto_reset_delay} seconds...")
            # Use timer to delay reset until servers are ready
            # ROS2 Foxy doesn't support oneshot=True, so we use a flag
            self._auto_reset_executed = False
            self._auto_reset_timer = self.create_timer(auto_reset_delay, self._delayed_auto_reset)
        else:
            self.get_logger().info("Auto-reset disabled. Use /polymetis/reset service to reset manually.")
    
    def _delayed_auto_reset(self):
        """
        Delayed auto-reset callback - called after servers are ready.
        
        This avoids SIGSEGV issues by:
        1. Waiting for robot/gripper servers to fully initialize
        2. Executing reset in a separate thread (non-blocking)
        """
        if self._auto_reset_executed:
            return  # Already executed, skip
        
        self._auto_reset_executed = True
        
        # Cancel timer (ROS2 Foxy compatible)
        if hasattr(self, '_auto_reset_timer') and self._auto_reset_timer is not None:
            self._auto_reset_timer.cancel()
        
        # Execute reset in separate thread to avoid blocking
        def reset_thread():
            try:
                self.get_logger().info("Executing delayed auto-reset to home position...")
                self._reset_robot()
                self.get_logger().info("Auto-reset completed successfully")
            except Exception as e:
                self.get_logger().error(f"Auto-reset failed: {e}\n{traceback.format_exc()}")
        
        threading.Thread(target=reset_thread, daemon=True).start()
    
    def _start_impedance_controller(self):
        """Start the impedance controller for continuous control."""
        try:
            if not isinstance(self._robot, MockRobotInterface):
                # Real RobotInterface: start joint impedance control
                if not self._robot.is_running_policy():
                    self._robot.start_joint_impedance()
                    self._controller_mode = "joint_impedance"
        except Exception as e:
            self.get_logger().warn(f"Failed to start impedance controller: {e}")
    
    def _ensure_controller_running_non_blocking(self):
        """
        Ensure controller is running for non-blocking commands.
        
        Similar to helper_non_blocking() in original robot.py:
        - If controller not running, start it
        - Wait for controller to be ready (with timeout and retry)
        - This should be called before update_desired_joint_positions()
        """
        if isinstance(self._robot, MockRobotInterface):
            return  # Mock doesn't need controller management
        
        with self._controller_ensure_lock:
            if not self._robot.is_running_policy():
                # Start controller in background thread to avoid blocking
                def ensure_controller_thread():
                    try:
                        self._controller_not_loaded = True
                        self._robot.start_cartesian_impedance()
                        self._controller_mode = "cartesian_impedance"
                        
                        # Wait for controller to be ready (with timeout and retry)
                        timeout = time.time() + 5
                        while not self._robot.is_running_policy():
                            time.sleep(0.01)
                            if time.time() > timeout:
                                # Retry starting controller
                                self._robot.start_cartesian_impedance()
                                timeout = time.time() + 5
                        
                        self._controller_not_loaded = False
                    except Exception as e:
                        self.get_logger().error(f"Error ensuring controller running: {e}")
                        self._controller_not_loaded = False
                
                # Only start thread if not already running
                if self._controller_ensure_thread is None or not self._controller_ensure_thread.is_alive():
                    self._controller_ensure_thread = threading.Thread(
                        target=ensure_controller_thread, 
                        daemon=True
                    )
                    self._controller_ensure_thread.start()
    
    def _command_callback(self, msg: PolymetisCommand):
        """
        Callback for PolymetisCommand messages.
        
        This function:
        1. Unpacks the ROS message
        2. Converts Python lists to PyTorch tensors
        3. Calls robot_interface.send_torch_policy() with impedance arguments
        4. Handles gripper commands if operate_gripper is True
        
        Args:
            msg: PolymetisCommand message
        """
        with self._command_lock:
            try:
                # ========== ARM CONTROL ==========
                # Extract arm command data from message
                # Message fields map to Polymetis controller arguments:
                # - joint_positions (q) -> desired joint angles
                # - joint_velocities (qd) -> velocity feedforward
                # - joint_torques (tau) -> torque control (if use_torque=True)
                # - kp (Kq) -> position gains (stiffness)
                # - kd (Kqd) -> velocity gains (damping)
                
                # Convert to lists and validate
                joint_pos = list(msg.joint_positions) if len(msg.joint_positions) > 0 else None
                joint_vel = list(msg.joint_velocities) if len(msg.joint_velocities) > 0 else None
                joint_torque = list(msg.joint_torques) if len(msg.joint_torques) > 0 else None
                kp = list(msg.kp) if len(msg.kp) > 0 else None
                kd = list(msg.kd) if len(msg.kd) > 0 else None
                
                # Validate array lengths (should be 7 for Franka arm)
                if joint_pos and len(joint_pos) != 7:
                    self.get_logger().warn(f"Invalid joint_positions length: {len(joint_pos)}, expected 7")
                    return
                if joint_vel and len(joint_vel) != 7:
                    self.get_logger().warn(f"Invalid joint_velocities length: {len(joint_vel)}, expected 7")
                    return
                if joint_torque and len(joint_torque) != 7:
                    self.get_logger().warn(f"Invalid joint_torques length: {len(joint_torque)}, expected 7")
                    return
                if kp and len(kp) != 7:
                    self.get_logger().warn(f"Invalid kp length: {len(kp)}, expected 7")
                    return
                if kd and len(kd) != 7:
                    self.get_logger().warn(f"Invalid kd length: {len(kd)}, expected 7")
                    return
                
                # Send impedance control command to robot interface
                # For MockRobotInterface, use send_torch_policy()
                if isinstance(self._robot, MockRobotInterface):
                    self._robot.send_torch_policy(
                        q=joint_pos,
                        qd=joint_vel,
                        Kq=kp,
                        Kqd=kd,
                        tau=joint_torque
                    )
                else:
                    # Real Polymetis RobotInterface implementation
                    # Update impedance gains if provided
                    if kp is not None and kd is not None:
                        if TORCH_AVAILABLE:
                            kp_tensor = torch.tensor(kp, dtype=torch.float32)
                            kd_tensor = torch.tensor(kd, dtype=torch.float32)
                            # Restart impedance controller with new gains
                            if not self._robot.is_running_policy():
                                self._robot.start_joint_impedance(Kq=kp_tensor, Kqd=kd_tensor)
                            else:
                                # Update gains (may need to restart policy)
                                self._robot.start_joint_impedance(Kq=kp_tensor, Kqd=kd_tensor)
                    
                    # Update desired joint positions
                    if joint_pos is not None and msg.use_position:
                        if TORCH_AVAILABLE:
                            pos_tensor = torch.tensor(joint_pos, dtype=torch.float32)
                            if not self._robot.is_running_policy():
                                self._robot.start_joint_impedance()
                            self._robot.update_desired_joint_positions(pos_tensor)
                    
                    # Handle blocking position control
                    if joint_pos is not None and msg.blocking and msg.use_position:
                        if TORCH_AVAILABLE:
                            pos_tensor = torch.tensor(joint_pos, dtype=torch.float32)
                            if self._robot.is_running_policy():
                                self._robot.terminate_current_policy()
                            # Calculate time to go (simple heuristic)
                            curr_pos = self._robot.get_joint_positions()
                            displacement = torch.abs(pos_tensor - curr_pos)
                            max_displacement = torch.max(displacement).item()
                            time_to_go = min(4.0, max(0.5, max_displacement / 1.0))  # 1 rad/s max velocity
                            self._robot.move_to_joint_positions(pos_tensor, time_to_go=time_to_go)
                            self._robot.start_joint_impedance()
                
                # ========== GRIPPER CONTROL ==========
                # Handle gripper command if operate_gripper is True
                # Note: Don't block the main thread - use non-blocking mode
                if msg.operate_gripper:
                    gripper_width = msg.gripper_width
                    gripper_speed = msg.gripper_speed
                    gripper_force = msg.gripper_force
                    gripper_blocking = msg.gripper_blocking
                    
                    # Call gripper interface (non-blocking in main thread)
                    # If blocking is needed, it should be handled in a separate thread
                    if gripper_blocking:
                        # Run in separate thread to avoid blocking
                        def gripper_thread():
                            self._gripper.goto(
                                width=gripper_width,
                                speed=gripper_speed,
                                force=gripper_force,
                                blocking=True
                            )
                        threading.Thread(target=gripper_thread, daemon=True).start()
                    else:
                        self._gripper.goto(
                            width=gripper_width,
                            speed=gripper_speed,
                            force=gripper_force,
                            blocking=False
                        )
                
                self._last_command_time = self.get_clock().now()
                self.get_logger().debug("Polymetis command executed successfully")
                
            except Exception as e:
                self.get_logger().error(f"Error processing command: {e}\n{traceback.format_exc()}")
    
    def _robot_command_callback(self, msg: PolymetisRobotCommand):
        """
        Callback for high-level PolymetisRobotCommand messages (NON-BLOCKING ONLY).
        
        This callback handles ONLY non-blocking commands via Topic.
        Blocking commands should use Services instead.
        
        Similar to update_command() in robot.py, this handles:
        - cartesian_position/velocity (non-blocking)
        - joint_position/velocity (non-blocking)
        - gripper position/velocity (non-blocking)
        - action space conversion via IK solver
        
        Args:
            msg: PolymetisRobotCommand message (blocking flag is ignored - always non-blocking)
        """
        if self._ik_solver is None:
            self.get_logger().error("RobotIKSolver not available. Cannot process high-level commands.")
            return
        
        # Topic commands are ALWAYS non-blocking
        # If blocking is needed, use Service instead
        if msg.blocking:
            self.get_logger().warn("Blocking command received via Topic. Use Service for blocking operations.")
        
        try:
            # Get current robot state
            robot_state_dict, _ = self._get_robot_state()
            
            # Extract command parameters
            action_space = msg.action_space
            gripper_action_space = msg.gripper_action_space if msg.gripper_action_space else None
            command = list(msg.command)
            
            # Validate action space
            if action_space not in ["cartesian_position", "joint_position", "cartesian_velocity", "joint_velocity"]:
                self.get_logger().error(f"Invalid action_space: {action_space}")
                return
            
            # Determine gripper action space if not specified
            velocity = "velocity" in action_space
            if gripper_action_space is None:
                gripper_action_space = "velocity" if velocity else "position"
            
            # Process gripper command (non-blocking)
            if len(command) > 0:
                gripper_cmd = command[-1]
                
                if gripper_action_space == "velocity":
                    gripper_delta = self._ik_solver.gripper_velocity_to_delta(gripper_cmd)
                    gripper_position = robot_state_dict["gripper_position"] + gripper_delta
                    gripper_position = float(np.clip(gripper_position, 0, 1))
                else:
                    gripper_position = float(np.clip(gripper_cmd, 0, 1))
                
                # Update gripper (non-blocking)
                gripper_width = self._max_gripper_width * (1 - gripper_position)
                self._gripper.goto(width=gripper_width, speed=0.05, force=0.1, blocking=False)
            
            # Process arm command (non-blocking)
            if "cartesian" in action_space:
                arm_cmd = command[:-1] if len(command) > 1 else command
                
                if velocity:
                    # Cartesian velocity -> joint velocity via IK
                    robot_state = {
                        "joint_positions": robot_state_dict["joint_positions"],
                        "joint_velocities": robot_state_dict["joint_velocities"]
                    }
                    joint_velocity = self._ik_solver.cartesian_velocity_to_joint_velocity(
                        arm_cmd, robot_state=robot_state
                    )
                    joint_delta = self._ik_solver.joint_velocity_to_delta(joint_velocity)
                    joint_position = (np.array(robot_state_dict["joint_positions"]) + joint_delta).tolist()
                else:
                    # Cartesian position -> joint position via IK
                    if TORCH_AVAILABLE:
                        pos = torch.Tensor(arm_cmd[:3])
                        quat = torch.Tensor(euler_to_quat(arm_cmd[3:6]))
                        curr_joints = torch.Tensor(robot_state_dict["joint_positions"])
                    else:
                        pos = np.array(arm_cmd[:3])
                        quat = np.array(euler_to_quat(arm_cmd[3:6]))
                        curr_joints = np.array(robot_state_dict["joint_positions"])
                    
                    desired_joints = self._robot.solve_inverse_kinematics(pos, quat, curr_joints)
                    if TORCH_AVAILABLE and isinstance(desired_joints, torch.Tensor):
                        joint_position = desired_joints.tolist()
                    elif hasattr(desired_joints, 'tolist'):
                        joint_position = desired_joints.tolist()
                    else:
                        joint_position = list(desired_joints)
                
                # Update joints (NON-BLOCKING: ensure controller running, then update desired position)
                self._ensure_controller_running_non_blocking()
                if TORCH_AVAILABLE:
                    pos_tensor = torch.tensor(joint_position, dtype=torch.float32)
                    self._robot.update_desired_joint_positions(pos_tensor)
            
            elif "joint" in action_space:
                arm_cmd = command[:-1] if len(command) > 1 else command
                
                if velocity:
                    joint_delta = self._ik_solver.joint_velocity_to_delta(arm_cmd)
                    joint_position = (np.array(robot_state_dict["joint_positions"]) + joint_delta).tolist()
                else:
                    joint_position = arm_cmd
                
                # Update joints (NON-BLOCKING: ensure controller running, then update desired position)
                self._ensure_controller_running_non_blocking()
                if TORCH_AVAILABLE:
                    pos_tensor = torch.tensor(joint_position, dtype=torch.float32)
                    self._robot.update_desired_joint_positions(pos_tensor)
            
            self.get_logger().debug(f"Processed non-blocking robot command: {action_space}")
            
        except Exception as e:
            self.get_logger().error(f"Error processing robot command: {e}\n{traceback.format_exc()}")
    
    def _get_robot_state(self):
        """
        Get comprehensive robot state similar to robot.py get_robot_state().
        
        Returns:
            tuple: (state_dict, timestamp_dict)
        """
        try:
            robot_state = self._robot.get_robot_state()
            gripper_state = self._gripper.get_state()
            
            # Get gripper position (normalized)
            gripper_position = 1 - (gripper_state.width / self._max_gripper_width)
            
            # Get cartesian position via forward kinematics
            try:
                if TORCH_AVAILABLE and hasattr(self._robot, 'robot_model'):
                    joint_pos_tensor = torch.Tensor(robot_state.joint_positions)
                    pos, quat = self._robot.robot_model.forward_kinematics(joint_pos_tensor)
                    cartesian_position = pos.tolist() + quat_to_euler(quat.numpy()).tolist()
                    ee_quat = quat.numpy().tolist()  # [x, y, z, w]
                else:
                    # Fallback: use get_ee_pose if available
                    try:
                        pos, quat = self._robot.get_ee_pose()
                        if TORCH_AVAILABLE and isinstance(quat, torch.Tensor):
                            cartesian_position = pos.tolist() + quat_to_euler(quat.numpy()).tolist()
                            ee_quat = quat.numpy().tolist()
                        else:
                            cartesian_position = pos.tolist() + quat_to_euler(quat).tolist()
                            ee_quat = quat.tolist() if hasattr(quat, 'tolist') else list(quat)
                    except:
                        cartesian_position = [0.0] * 6
                        ee_quat = [0.0, 0.0, 0.0, 1.0]
            except Exception as e:
                self.get_logger().warn(f"Failed to compute forward kinematics: {e}")
                cartesian_position = [0.0] * 6
                ee_quat = [0.0, 0.0, 0.0, 1.0]
            
            # Convert to lists
            def to_list(x):
                if TORCH_AVAILABLE and isinstance(x, torch.Tensor):
                    return x.cpu().numpy().tolist()
                elif hasattr(x, 'tolist'):
                    return x.tolist()
                else:
                    return list(x)
            
            state_dict = {
                "cartesian_position": cartesian_position,
                "gripper_position": gripper_position,
                "joint_positions": to_list(robot_state.joint_positions),
                "joint_velocities": to_list(robot_state.joint_velocities),
                "joint_torques_computed": to_list(robot_state.joint_torques_computed),
                "prev_joint_torques_computed": to_list(robot_state.prev_joint_torques_computed),
                "prev_joint_torques_computed_safened": to_list(robot_state.prev_joint_torques_computed_safened),
                "motor_torques_measured": to_list(robot_state.motor_torques_measured),
                "prev_controller_latency_ms": robot_state.prev_controller_latency_ms,
                "prev_command_successful": robot_state.prev_command_successful,
            }
            
            timestamp_dict = {
                "robot_timestamp_seconds": robot_state.timestamp.seconds if hasattr(robot_state.timestamp, 'seconds') else 0,
                "robot_timestamp_nanos": robot_state.timestamp.nanos if hasattr(robot_state.timestamp, 'nanos') else 0,
            }
            
            return state_dict, timestamp_dict
            
        except Exception as e:
            self.get_logger().error(f"Error getting robot state: {e}\n{traceback.format_exc()}")
            # Return empty state on error
            return {
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
            }, {"robot_timestamp_seconds": 0, "robot_timestamp_nanos": 0}
    
    def _publish_states(self):
        """
        Publish joint states, robot state, gripper state, EE pose, and controller status at high frequency.
        """
        now = self.get_clock().now()
        
        # Publish joint states (for tf tree)
        self._publish_joint_states()
        
        # Publish robot state
        self._publish_robot_state()
        
        # Publish gripper state
        self._publish_gripper_state()
        
        # Publish end-effector pose
        self._publish_ee_pose()
        
        # Publish controller status
        self._publish_controller_status()
    
    def _publish_joint_states(self):
        """
        Publish joint states at high frequency.
        
        Combines Arm joint states AND Gripper finger states into a single JointState message.
        """
        try:
            # Get current state from robot interface
            joint_positions = self._robot.get_joint_positions()
            joint_velocities = self._robot.get_joint_velocities()
            
            # Convert to numpy if torch tensor
            if TORCH_AVAILABLE and isinstance(joint_positions, torch.Tensor):
                joint_positions = joint_positions.cpu().numpy()
            if TORCH_AVAILABLE and isinstance(joint_velocities, torch.Tensor):
                joint_velocities = joint_velocities.cpu().numpy()
            
            # Convert to lists
            joint_positions = joint_positions.tolist() if hasattr(joint_positions, 'tolist') else list(joint_positions)
            joint_velocities = joint_velocities.tolist() if hasattr(joint_velocities, 'tolist') else list(joint_velocities)
            
            # Get gripper state
            gripper_state = self._gripper.get_state()
            gripper_width = gripper_state.width
            # Use the cached max_gripper_width (set during initialization)
            # This avoids AttributeError if GripperInterface doesn't have metadata
            max_gripper_width = self._max_gripper_width
            
            # Calculate gripper finger positions (each finger moves half the width)
            finger1_position = gripper_width / 2.0
            finger2_position = gripper_width / 2.0
            
            # Create JointState message
            joint_state_msg = JointState()
            
            # Set header with proper timestamp
            now = self.get_clock().now()
            joint_state_msg.header.stamp = now.to_msg()
            joint_state_msg.header.frame_id = 'base_link'
            
            # Set joint names (arm + gripper)
            joint_state_msg.name = list(self.all_joint_names)
            
            # Set joint positions (arm + gripper)
            joint_state_msg.position = (
                joint_positions[:len(self.arm_joint_names)] +
                [finger1_position, finger2_position]
            )
            
            # Set joint velocities (arm + gripper)
            joint_state_msg.velocity = (
                joint_velocities[:len(self.arm_joint_names)] +
                [0.0, 0.0]  # Gripper velocities (usually not available)
            )
            
            # Set joint efforts (torques) - arm only, gripper not available
            joint_state_msg.effort = [0.0] * len(self.all_joint_names)
            
            # Publish
            self._joint_state_publisher.publish(joint_state_msg)
            
        except Exception as e:
            self.get_logger().error(f"Error publishing joint states: {e}\n{traceback.format_exc()}")
    
    def _publish_robot_state(self):
        """
        Publish comprehensive robot state under /polymetis/robot_state.
        Similar to robot.py get_robot_state() output.
        """
        try:
            state_dict, timestamp_dict = self._get_robot_state()
            
            # Create PolymetisRobotState message
            msg = PolymetisRobotState()
            now = self.get_clock().now()
            msg.header.stamp = now.to_msg()
            msg.header.frame_id = 'base_link'
            
            # Joint states
            msg.joint_positions = state_dict["joint_positions"]
            msg.joint_velocities = state_dict["joint_velocities"]
            msg.joint_torques_computed = state_dict["joint_torques_computed"]
            msg.prev_joint_torques_computed = state_dict["prev_joint_torques_computed"]
            msg.prev_joint_torques_computed_safened = state_dict["prev_joint_torques_computed_safened"]
            msg.motor_torques_measured = state_dict["motor_torques_measured"]
            
            # End-effector pose
            cartesian_pos = state_dict["cartesian_position"]
            msg.ee_position = cartesian_pos[:3]
            
            # Convert euler to quaternion for ee_quaternion
            try:
                ee_euler = cartesian_pos[3:6]
                ee_quat = euler_to_quat(ee_euler)
                msg.ee_quaternion = ee_quat.tolist() if hasattr(ee_quat, 'tolist') else list(ee_quat)
                msg.ee_euler = ee_euler
            except:
                msg.ee_quaternion = [0.0, 0.0, 0.0, 1.0]
                msg.ee_euler = [0.0, 0.0, 0.0]
            
            # Gripper state
            gripper_state = self._gripper.get_state()
            msg.gripper_width = gripper_state.width
            msg.gripper_position = state_dict["gripper_position"]
            msg.gripper_is_grasped = getattr(gripper_state, 'is_grasped', False)
            msg.gripper_is_moving = getattr(gripper_state, 'is_moving', False)
            msg.gripper_prev_command_successful = getattr(gripper_state, 'prev_command_successful', True)
            msg.gripper_error_code = getattr(gripper_state, 'error_code', 0)
            
            # Controller info
            msg.prev_controller_latency_ms = state_dict["prev_controller_latency_ms"]
            msg.prev_command_successful = state_dict["prev_command_successful"]
            msg.is_running_policy = self._robot.is_running_policy() if hasattr(self._robot, 'is_running_policy') else False
            
            # Polymetis timestamp (convert to int for ROS2 message type)
            msg.polymetis_timestamp_ns = int(timestamp_dict["robot_timestamp_seconds"] * 1e9 + timestamp_dict["robot_timestamp_nanos"])
            
            # Publish
            self._robot_state_publisher.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f"Error publishing robot state: {e}\n{traceback.format_exc()}")
    
    def _publish_gripper_state(self):
        """
        Publish gripper state under /polymetis/gripper_state.
        """
        try:
            gripper_state = self._gripper.get_state()
            
            # Create PolymetisGripperState message
            msg = PolymetisGripperState()
            now = self.get_clock().now()
            msg.header.stamp = now.to_msg()
            msg.header.frame_id = 'gripper_link'
            
            msg.width = gripper_state.width
            msg.position = 1 - (gripper_state.width / self._max_gripper_width)  # Normalized
            msg.is_grasped = getattr(gripper_state, 'is_grasped', False)
            msg.is_moving = getattr(gripper_state, 'is_moving', False)
            msg.prev_command_successful = getattr(gripper_state, 'prev_command_successful', True)
            msg.error_code = getattr(gripper_state, 'error_code', 0)
            msg.max_width = self._max_gripper_width
            
            # Get timestamp if available
            if hasattr(gripper_state, 'timestamp'):
                timestamp = gripper_state.timestamp
                if hasattr(timestamp, 'seconds') and hasattr(timestamp, 'nanos'):
                    msg.polymetis_timestamp_ns = int(timestamp.seconds * 1e9 + timestamp.nanos)
                else:
                    msg.polymetis_timestamp_ns = 0
            else:
                msg.polymetis_timestamp_ns = 0
            
            # Publish
            self._gripper_state_publisher.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f"Error publishing gripper state: {e}\n{traceback.format_exc()}")
    
    def _publish_ee_pose(self):
        """
        Publish end-effector pose under /polymetis/arm/ee_pose.
        """
        try:
            if isinstance(self._robot, MockRobotInterface):
                # Mock: return default pose
                pos = [0.0, 0.0, 0.0]
                quat = [0.0, 0.0, 0.0, 1.0]
            else:
                pos, quat = self._robot.get_ee_pose()
                if TORCH_AVAILABLE and isinstance(pos, torch.Tensor):
                    pos = pos.cpu().numpy().tolist()
                if TORCH_AVAILABLE and isinstance(quat, torch.Tensor):
                    quat = quat.cpu().numpy().tolist()
                else:
                    pos = pos.tolist() if hasattr(pos, 'tolist') else list(pos)
                    quat = quat.tolist() if hasattr(quat, 'tolist') else list(quat)
            
            # Create PoseStamped message
            msg = PoseStamped()
            now = self.get_clock().now()
            msg.header.stamp = now.to_msg()
            msg.header.frame_id = 'base_link'
            
            msg.pose.position.x = float(pos[0])
            msg.pose.position.y = float(pos[1])
            msg.pose.position.z = float(pos[2])
            
            # Quaternion format: [x, y, z, w]
            msg.pose.orientation.x = float(quat[0])
            msg.pose.orientation.y = float(quat[1])
            msg.pose.orientation.z = float(quat[2])
            msg.pose.orientation.w = float(quat[3])
            
            # Publish
            self._ee_pose_publisher.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f"Error publishing EE pose: {e}\n{traceback.format_exc()}")
    
    def _publish_controller_status(self):
        """
        Publish controller status under /polymetis/controller/status.
        """
        try:
            msg = ControllerStatus()
            now = self.get_clock().now()
            msg.header.stamp = now.to_msg()
            msg.header.frame_id = 'base_link'
            
            # Get controller status
            if isinstance(self._robot, MockRobotInterface):
                msg.is_running_policy = self._robot.is_running_policy()
            else:
                msg.is_running_policy = self._robot.is_running_policy() if hasattr(self._robot, 'is_running_policy') else False
            
            msg.controller_mode = self._controller_mode
            
            # Get current gains if available
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
            
            # Publish
            self._controller_status_publisher.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f"Error publishing controller status: {e}\n{traceback.format_exc()}")
    
    def _gripper_command_callback(self, msg: GripperCommand):
        """
        Callback for GripperCommand messages.
        
        Args:
            msg: GripperCommand message
        """
        try:
            width = msg.width
            speed = msg.speed if msg.speed > 0 else 0.05
            force = msg.force if msg.force > 0 else 0.1
            blocking = msg.blocking
            
            if blocking:
                # Run in separate thread to avoid blocking
                def gripper_thread():
                    self._gripper.goto(width=width, speed=speed, force=force, blocking=True)
                threading.Thread(target=gripper_thread, daemon=True).start()
            else:
                self._gripper.goto(width=width, speed=speed, force=force, blocking=False)
            
            self.get_logger().debug(f"Gripper command: width={width}, speed={speed}, force={force}, blocking={blocking}")
            
        except Exception as e:
            self.get_logger().error(f"Error processing gripper command: {e}\n{traceback.format_exc()}")
    
    def _reset_robot(self, randomize=False):
        """
        Internal method to reset robot to home position.
        
        This safely moves the robot to a known safe position, matching robot_env.py reset logic:
        1. Close gripper first (blocking)
        2. If randomize=True, generate cartesian noise
        3. Move to reset_joints position (blocking), optionally with noise
        
        Args:
            randomize: If True, add random cartesian noise to reset position
                      (same as robot_env.py randomize parameter)
        """
        try:
            self.get_logger().info(f"Resetting robot to home position (randomize={randomize})...")
            
            # Step 1: Close gripper first (blocking) - same as robot_env.py
            self.get_logger().debug("Closing gripper...")
            self._gripper.goto(width=0.0, speed=0.05, force=0.1, blocking=True)  # Slow speed for safety
            
            # Step 2: Generate cartesian noise if randomize=True (same as robot_env.py)
            cartesian_noise = None
            if randomize:
                # Same noise range as robot_env.py
                randomize_low = np.array([-0.1, -0.2, -0.1, -0.3, -0.3, -0.3])
                randomize_high = np.array([0.1, 0.2, 0.1, 0.3, 0.3, 0.3])
                cartesian_noise = np.random.uniform(low=randomize_low, high=randomize_high)
                self.get_logger().info(f"Generated cartesian noise: {cartesian_noise}")
            
            # Step 3: Move to reset joint positions (blocking for safety)
            target_joints = self._reset_joints.copy()
            
            # Apply cartesian noise if provided (similar to robot.py add_noise_to_joints)
            if cartesian_noise is not None and TORCH_AVAILABLE and not isinstance(self._robot, MockRobotInterface):
                try:
                    target_joints = self._add_cartesian_noise_to_joints(target_joints, cartesian_noise)
                except Exception as e:
                    self.get_logger().warn(f"Failed to add cartesian noise: {e}. Using original joints.")
            
            # Move to target position
            if TORCH_AVAILABLE:
                target_joints_tensor = torch.tensor(target_joints, dtype=torch.float32)
                if not isinstance(self._robot, MockRobotInterface):
                    # Terminate any running policy
                    if self._robot.is_running_policy():
                        self._robot.terminate_current_policy()
                    
                    # Calculate time to go (slow speed for safety: 0.1 rad/s)
                    curr_joints = self._robot.get_joint_positions()
                    displacement = torch.abs(target_joints_tensor - curr_joints)
                    max_displacement = torch.max(displacement).item()
                    time_to_go = min(40.0, max(5.0, max_displacement / 0.1))  # 0.1 rad/s max velocity (10x slower)
                    
                    self.get_logger().debug(f"Moving to reset position (time_to_go={time_to_go:.2f}s)...")
                    
                    # Move to reset position (blocking)
                    self._robot.move_to_joint_positions(target_joints_tensor, time_to_go=time_to_go)
                    
                    # Restart impedance controller
                    self._robot.start_joint_impedance()
                else:
                    # Mock interface (slow speed for consistency)
                    self._robot.move_to_joint_positions(target_joints_tensor, time_to_go=20.0)
            else:
                # Fallback without torch
                target_joints_list = target_joints.tolist() if hasattr(target_joints, 'tolist') else list(target_joints)
                if not isinstance(self._robot, MockRobotInterface):
                    if self._robot.is_running_policy():
                        self._robot.terminate_current_policy()
                    curr_joints = np.array(self._robot.get_joint_positions())
                    displacement = np.abs(np.array(target_joints) - curr_joints)
                    max_displacement = np.max(displacement)
                    time_to_go = min(40.0, max(5.0, max_displacement / 0.1))  # 0.1 rad/s max velocity (10x slower)
                    self._robot.move_to_joint_positions(target_joints_list, time_to_go=time_to_go)
                    self._robot.start_joint_impedance()
                else:
                    self._robot.move_to_joint_positions(target_joints_list, time_to_go=20.0)  # Slow speed for consistency
            
            self.get_logger().info("Robot reset to home position successfully")
            
        except Exception as e:
            self.get_logger().error(f"Reset error: {e}\n{traceback.format_exc()}")
            # Don't raise exception - log and continue
    
    def _add_cartesian_noise_to_joints(self, original_joints, cartesian_noise):
        """
        Add cartesian noise to joint positions.
        
        Similar to robot.py add_noise_to_joints():
        1. Forward kinematics to get current pose
        2. Add cartesian noise to pose
        3. Inverse kinematics to get new joints
        
        Args:
            original_joints: Original joint positions (numpy array)
            cartesian_noise: Cartesian noise [x, y, z, roll, pitch, yaw]
        
        Returns:
            New joint positions with noise applied
        """
        if not TORCH_AVAILABLE or isinstance(self._robot, MockRobotInterface):
            return original_joints
        
        try:
            original_joints_tensor = torch.tensor(original_joints, dtype=torch.float32)
            
            # Forward kinematics to get current pose
            if hasattr(self._robot, 'robot_model'):
                pos, quat = self._robot.robot_model.forward_kinematics(original_joints_tensor)
                curr_pose = pos.tolist() + quat_to_euler(quat.numpy()).tolist()
            else:
                pos, quat = self._robot.get_ee_pose()
                if isinstance(quat, torch.Tensor):
                    curr_pose = pos.tolist() + quat_to_euler(quat.numpy()).tolist()
                else:
                    curr_pose = pos.tolist() + quat_to_euler(quat).tolist()
            
            # Add cartesian noise to pose
            new_pose = add_poses(cartesian_noise, curr_pose)
            
            # Inverse kinematics to get new joints
            new_pos = torch.tensor(new_pose[:3], dtype=torch.float32)
            new_quat = torch.tensor(euler_to_quat(new_pose[3:6]), dtype=torch.float32)
            
            noisy_joints, success = self._robot.solve_inverse_kinematics(new_pos, new_quat, original_joints_tensor)
            
            if success:
                if isinstance(noisy_joints, torch.Tensor):
                    return noisy_joints.numpy()
                return np.array(noisy_joints)
            else:
                self.get_logger().warn("IK failed for noisy pose, using original joints")
                return original_joints
                
        except Exception as e:
            self.get_logger().warn(f"Error adding cartesian noise: {e}")
            return original_joints
    
    def _reset_service_callback(self, request, response):
        """
        Service callback for reset - move robot to reset/home position.
        
        This safely moves the robot to a known safe position, similar to robot_env.py reset.
        Supports randomize option (matching robot_env.py behavior).
        
        Reset is executed in a separate thread to avoid blocking and SIGSEGV issues.
        """
        try:
            # Get randomize parameter from request (if available)
            randomize = getattr(request, 'randomize', False)
            
            # Execute reset in separate thread to avoid blocking service callback
            def reset_thread():
                try:
                    self._reset_robot(randomize=randomize)
                except Exception as e:
                    self.get_logger().error(f"Reset thread error: {e}\n{traceback.format_exc()}")
            
            threading.Thread(target=reset_thread, daemon=True).start()
            
            response.success = True
            response.message = "Reset command accepted, executing in background"
            if randomize:
                response.message += " (with randomization)"
            
        except Exception as e:
            response.success = False
            response.message = f"Reset failed: {str(e)}"
            self.get_logger().error(f"Reset service error: {e}\n{traceback.format_exc()}")
        
        return response
    
    def _start_cartesian_impedance_callback(self, request, response):
        """
        Service callback for starting cartesian impedance controller.
        """
        try:
            if isinstance(self._robot, MockRobotInterface):
                response.success = True
                response.message = "Mock: Cartesian impedance started"
                self._controller_mode = "cartesian_impedance"
            else:
                if TORCH_AVAILABLE:
                    kx = None
                    kxd = None
                    if len(request.kx) > 0:
                        kx = torch.tensor(request.kx, dtype=torch.float32)
                    if len(request.kxd) > 0:
                        kxd = torch.tensor(request.kxd, dtype=torch.float32)
                    
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
            response.message = f"Failed to start cartesian impedance: {str(e)}"
            self.get_logger().error(f"Error starting cartesian impedance: {e}\n{traceback.format_exc()}")
        
        return response
    
    def _start_joint_impedance_callback(self, request, response):
        """
        Service callback for starting joint impedance controller.
        """
        try:
            if isinstance(self._robot, MockRobotInterface):
                response.success = True
                response.message = "Mock: Joint impedance started"
                self._controller_mode = "joint_impedance"
            else:
                if TORCH_AVAILABLE:
                    kq = None
                    kqd = None
                    if len(request.kq) > 0:
                        kq = torch.tensor(request.kq, dtype=torch.float32)
                    if len(request.kqd) > 0:
                        kqd = torch.tensor(request.kqd, dtype=torch.float32)
                    
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
            response.message = f"Failed to start joint impedance: {str(e)}"
            self.get_logger().error(f"Error starting joint impedance: {e}\n{traceback.format_exc()}")
        
        return response
    
    def _terminate_policy_callback(self, request, response):
        """
        Service callback for terminating current policy.
        """
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
            response.message = f"Failed to terminate policy: {str(e)}"
            self.get_logger().error(f"Error terminating policy: {e}\n{traceback.format_exc()}")
        
        return response
    
    def _move_to_joint_positions_callback(self, request, response):
        """
        Service callback for moving to joint positions (BLOCKING).
        
        This is a blocking service - it waits until movement completes.
        Use MultiThreadedExecutor to allow /joint_states to continue publishing.
        
        Same pattern as robot.py update_joints(blocking=True):
        1. Terminate current policy
        2. move_to_joint_positions (blocking)
        3. Restart impedance controller
        """
        try:
            if len(request.joint_positions) != 7:
                response.success = False
                response.message = f"Invalid joint positions length: {len(request.joint_positions)}, expected 7"
                return response
            
            if isinstance(self._robot, MockRobotInterface):
                if TORCH_AVAILABLE:
                    pos_tensor = torch.tensor(request.joint_positions, dtype=torch.float32)
                else:
                    pos_tensor = request.joint_positions
                time_to_go = request.time_to_go if request.time_to_go > 0 else 2.0
                self._robot.move_to_joint_positions(pos_tensor, time_to_go=time_to_go)
                response.success = True
                response.message = "Mock: Moved to joint positions"
                response.final_joint_positions = request.joint_positions
            else:
                if TORCH_AVAILABLE:
                    pos_tensor = torch.tensor(request.joint_positions, dtype=torch.float32)
                    
                    # Step 1: Terminate current policy (same as robot.py)
                    if self._robot.is_running_policy():
                        self._robot.terminate_current_policy()
                    
                    # Calculate time to go
                    if request.time_to_go > 0:
                        time_to_go = request.time_to_go
                    else:
                        curr_pos = self._robot.get_joint_positions()
                        displacement = torch.abs(pos_tensor - curr_pos)
                        max_displacement = torch.max(displacement).item()
                        time_to_go = min(4.0, max(0.5, max_displacement / 1.0))
                    
                    # Step 2: BLOCKING move to position (same as robot.py)
                    self._robot.move_to_joint_positions(pos_tensor, time_to_go=time_to_go)
                    
                    # Step 3: Restart impedance controller (same as robot.py)
                    self._robot.start_cartesian_impedance()
                    self._controller_mode = "cartesian_impedance"
                    
                    # Get final position
                    final_pos = self._robot.get_joint_positions()
                    if isinstance(final_pos, torch.Tensor):
                        response.final_joint_positions = final_pos.cpu().numpy().tolist()
                    else:
                        response.final_joint_positions = final_pos.tolist() if hasattr(final_pos, 'tolist') else list(final_pos)
                    
                    response.success = True
                    response.message = f"Moved to joint positions (time_to_go={time_to_go:.1f}s)"
                else:
                    response.success = False
                    response.message = "PyTorch not available"
        except Exception as e:
            response.success = False
            response.message = f"Failed to move to joint positions: {str(e)}"
            response.final_joint_positions = []
            self.get_logger().error(f"Error moving to joint positions: {e}\n{traceback.format_exc()}")
        
        return response
    
    def _move_to_ee_pose_callback(self, request, response):
        """
        Service callback for moving to end-effector pose (BLOCKING).
        
        This is a blocking service - it waits until movement completes.
        Use MultiThreadedExecutor to allow /joint_states to continue publishing.
        
        Same pattern as robot.py update_pose(blocking=True):
        1. Solve IK to get joint positions
        2. Call update_joints(blocking=True)
        """
        try:
            if len(request.position) != 3:
                response.success = False
                response.message = f"Invalid position length: {len(request.position)}, expected 3"
                return response
            
            if len(request.orientation) != 4:
                response.success = False
                response.message = f"Invalid orientation length: {len(request.orientation)}, expected 4"
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
            
            # Get initial joint guess
            if len(request.q0) == 7:
                q0 = torch.tensor(request.q0, dtype=torch.float32)
            else:
                q0 = self._robot.get_joint_positions()
            
            # Solve IK (same as robot.py update_pose blocking)
            desired_joints, ik_success = self._robot.solve_inverse_kinematics(pos, quat, q0)
            
            if not ik_success:
                response.success = False
                response.message = "IK solution not found"
                response.final_joint_positions = []
                return response
            
            # Step 1: Terminate current policy (same as robot.py)
            if self._robot.is_running_policy():
                self._robot.terminate_current_policy()
            
            # Calculate time to go
            if request.time_to_go > 0:
                time_to_go = request.time_to_go
            else:
                curr_pos = self._robot.get_joint_positions()
                displacement = torch.abs(desired_joints - curr_pos)
                max_displacement = torch.max(displacement).item()
                time_to_go = min(4.0, max(0.5, max_displacement / 1.0))
            
            # Step 2: BLOCKING move to position (same as robot.py)
            self._robot.move_to_joint_positions(desired_joints, time_to_go=time_to_go)
            
            # Step 3: Restart impedance controller (same as robot.py)
            self._robot.start_cartesian_impedance()
            self._controller_mode = "cartesian_impedance"
            
            # Get final position
            final_pos = self._robot.get_joint_positions()
            if isinstance(final_pos, torch.Tensor):
                response.final_joint_positions = final_pos.cpu().numpy().tolist()
            else:
                response.final_joint_positions = final_pos.tolist() if hasattr(final_pos, 'tolist') else list(final_pos)
            
            response.success = True
            response.message = f"Moved to EE pose (time_to_go={time_to_go:.1f}s)"
            
        except Exception as e:
            response.success = False
            response.message = f"Failed to move to EE pose: {str(e)}"
            response.final_joint_positions = []
            self.get_logger().error(f"Error moving to EE pose: {e}\n{traceback.format_exc()}")
        
        return response
    
    def _solve_ik_callback(self, request, response):
        """
        Service callback for solving inverse kinematics.
        """
        try:
            if len(request.position) != 3:
                response.success = False
                response.message = f"Invalid position length: {len(request.position)}, expected 3"
                return response
            
            if len(request.orientation) != 4:
                response.success = False
                response.message = f"Invalid orientation length: {len(request.orientation)}, expected 4"
                return response
            
            if len(request.q0) != 7:
                response.success = False
                response.message = f"Invalid q0 length: {len(request.q0)}, expected 7"
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
                        response.joint_positions = joint_pos.tolist() if hasattr(joint_pos, 'tolist') else list(joint_pos)
                    
                    # Compute error
                    if hasattr(self._robot, 'robot_model'):
                        pos_output, quat_output = self._robot.robot_model.forward_kinematics(joint_pos)
                        # Simple error computation
                        pos_error = torch.norm(pos - pos_output).item()
                        quat_error = torch.norm(quat - quat_output).item()
                        response.error = pos_error + quat_error
                    else:
                        response.error = 0.0 if success else 1.0
                    response.message = "IK solved successfully" if success else "IK solution not found"
                    
                    response.success = success
                else:
                    response.success = False
                    response.message = "PyTorch not available"
                    response.joint_positions = []
                    response.error = 1.0
        except Exception as e:
            response.success = False
            response.message = f"Failed to solve IK: {str(e)}"
            response.joint_positions = []
            response.error = 1.0
            self.get_logger().error(f"Error solving IK: {e}\n{traceback.format_exc()}")
        
        return response
    
    def _compute_fk_callback(self, request, response):
        """
        Service callback for computing forward kinematics.
        """
        try:
            if len(request.joint_positions) != 7:
                response.success = False
                response.message = f"Invalid joint positions length: {len(request.joint_positions)}, expected 7"
                return response
            
            if isinstance(self._robot, MockRobotInterface):
                response.success = True
                response.position = [0.0, 0.0, 0.0]
                response.orientation = [0.0, 0.0, 0.0, 1.0]
            else:
                if TORCH_AVAILABLE and hasattr(self._robot, 'robot_model'):
                    joint_pos = torch.tensor(request.joint_positions, dtype=torch.float32)
                    pos, quat = self._robot.robot_model.forward_kinematics(joint_pos)
                    
                    if isinstance(pos, torch.Tensor):
                        response.position = pos.cpu().numpy().tolist()
                    else:
                        response.position = pos.tolist() if hasattr(pos, 'tolist') else list(pos)
                    
                    if isinstance(quat, torch.Tensor):
                        response.orientation = quat.cpu().numpy().tolist()
                    else:
                        response.orientation = quat.tolist() if hasattr(quat, 'tolist') else list(quat)
                    
                    response.success = True
                else:
                    # Fallback: use get_ee_pose
                    try:
                        pos, quat = self._robot.get_ee_pose()
                        if TORCH_AVAILABLE and isinstance(pos, torch.Tensor):
                            response.position = pos.cpu().numpy().tolist()
                            response.orientation = quat.cpu().numpy().tolist()
                        else:
                            response.position = pos.tolist() if hasattr(pos, 'tolist') else list(pos)
                            response.orientation = quat.tolist() if hasattr(quat, 'tolist') else list(quat)
                        response.success = True
                    except:
                        response.success = False
                        response.message = "FK computation not available"
                    else:
                        response.message = "FK computed successfully"
        except Exception as e:
            response.success = False
            response.message = f"Failed to compute FK: {str(e)}"
            self.get_logger().error(f"Error computing FK: {e}\n{traceback.format_exc()}")
        
        return response
    
    def _compute_time_to_go_callback(self, request, response):
        """
        Service callback for computing adaptive time to go.
        """
        try:
            if len(request.desired_joint_positions) != 7:
                response.success = False
                response.message = f"Invalid joint positions length: {len(request.desired_joint_positions)}, expected 7"
                return response
            
            if isinstance(self._robot, MockRobotInterface):
                response.success = True
                response.time_to_go = 2.0
            else:
                if TORCH_AVAILABLE:
                    desired_pos = torch.tensor(request.desired_joint_positions, dtype=torch.float32)
                    curr_pos = self._robot.get_joint_positions()
                    displacement = desired_pos - curr_pos
                    
                    if hasattr(self._robot, '_adaptive_time_to_go'):
                        time_to_go = self._robot._adaptive_time_to_go(displacement)
                        response.time_to_go = float(time_to_go.item() if isinstance(time_to_go, torch.Tensor) else time_to_go)
                    else:
                        # Fallback: simple heuristic
                        max_displacement = torch.max(torch.abs(displacement)).item()
                        response.time_to_go = min(4.0, max(0.5, max_displacement / 1.0))
                    
                    response.success = True
                else:
                    # Fallback: simple heuristic
                    desired_pos = np.array(request.desired_joint_positions)
                    curr_pos = np.array(self._robot.get_joint_positions())
                    displacement = np.abs(desired_pos - curr_pos)
                    max_displacement = np.max(displacement)
                    response.time_to_go = min(4.0, max(0.5, max_displacement / 1.0))
                    response.success = True
                    response.message = "Time to go computed successfully"
        except Exception as e:
            response.success = False
            response.message = f"Failed to compute time to go: {str(e)}"
            response.time_to_go = 0.0
            self.get_logger().error(f"Error computing time to go: {e}\n{traceback.format_exc()}")
        
        return response
    
    def _launch_controller(self):
        """
        Launch robot and gripper controller servers.
        
        Similar to robot.py launch_controller(), this:
        1. Kills any existing servers
        2. Launches robot server via launch_robot.sh
        3. Launches gripper server via launch_gripper.sh
        4. Waits for servers to initialize
        """
        try:
            # Kill existing servers first
            self._kill_controller()
        except Exception as e:
            self.get_logger().debug(f"No existing servers to kill: {e}")
        
        # Get script directory
        script_dir = Path(__file__).parent.absolute()
        launch_robot_script = script_dir / 'launch_robot.sh'
        launch_gripper_script = script_dir / 'launch_gripper.sh'
        
        if not launch_robot_script.exists():
            raise FileNotFoundError(f"launch_robot.sh not found at {launch_robot_script}")
        if not launch_gripper_script.exists():
            raise FileNotFoundError(f"launch_gripper.sh not found at {launch_gripper_script}")
        
        self.get_logger().info("Launching robot and gripper servers...")
        
        # Prepare commands with robot_ip parameter
        # The scripts expect environment to be set up (micromamba activated via polymetis_ros2.env)
        # and use ROBOT_IP from environment variable
        env = os.environ.copy()
        env['ROBOT_IP'] = self._robot_ip
        # Disable real-time by default in Docker to avoid SIGSEGV issues
        # User can enable by setting USE_REAL_TIME=true environment variable
        env['USE_REAL_TIME'] = os.environ.get('USE_REAL_TIME', 'false')
        
        # Note: Scripts assume Docker environment with micromamba already activated
        # They should be run after sourcing polymetis_ros2.env
        robot_cmd = f"bash {launch_robot_script}"
        gripper_cmd = f"bash {launch_gripper_script}"
        
        # Add sudo if password provided
        if self._sudo_password:
            robot_cmd = f"echo {self._sudo_password} | sudo -S {robot_cmd}"
            gripper_cmd = f"echo {self._sudo_password} | sudo -S {gripper_cmd}"
        
        # Launch robot server
        try:
            self._robot_process = subprocess.Popen(
                robot_cmd,
                shell=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                env=env,
                preexec_fn=os.setsid  # Create new process group
            )
            self.get_logger().info(f"Robot server launched (PID: {self._robot_process.pid})")
        except Exception as e:
            self.get_logger().error(f"Failed to launch robot server: {e}")
            raise
        
        # Launch gripper server
        try:
            self._gripper_process = subprocess.Popen(
                gripper_cmd,
                shell=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                env=env,
                preexec_fn=os.setsid  # Create new process group
            )
            self.get_logger().info(f"Gripper server launched (PID: {self._gripper_process.pid})")
        except Exception as e:
            self.get_logger().error(f"Failed to launch gripper server: {e}")
            # Try to kill robot process if gripper failed
            if self._robot_process:
                try:
                    os.killpg(os.getpgid(self._robot_process.pid), signal.SIGTERM)
                except:
                    pass
            raise
        
        self._server_launched = True
        self.get_logger().info("Controller servers launched successfully")
    
    def _kill_controller(self):
        """
        Kill robot and gripper controller servers.
        
        Similar to robot.py kill_controller(), this safely terminates:
        1. Robot server process
        2. Gripper server process
        3. Any stale processes (run_server, franka_panda_cl, gripper)
        """
        self.get_logger().info("Killing robot and gripper servers...")
        
        # Kill processes by PID if we have them
        if self._robot_process:
            try:
                # Kill entire process group
                os.killpg(os.getpgid(self._robot_process.pid), signal.SIGTERM)
                self._robot_process.wait(timeout=2)
                self.get_logger().info("Robot server process terminated")
            except ProcessLookupError:
                self.get_logger().debug("Robot server process already terminated")
            except subprocess.TimeoutExpired:
                self.get_logger().warn("Robot server process did not terminate, forcing kill")
                try:
                    os.killpg(os.getpgid(self._robot_process.pid), signal.SIGKILL)
                except:
                    pass
            except Exception as e:
                self.get_logger().warn(f"Error killing robot process: {e}")
            finally:
                self._robot_process = None
        
        if self._gripper_process:
            try:
                # Kill entire process group
                os.killpg(os.getpgid(self._gripper_process.pid), signal.SIGTERM)
                self._gripper_process.wait(timeout=2)
                self.get_logger().info("Gripper server process terminated")
            except ProcessLookupError:
                self.get_logger().debug("Gripper server process already terminated")
            except subprocess.TimeoutExpired:
                self.get_logger().warn("Gripper server process did not terminate, forcing kill")
                try:
                    os.killpg(os.getpgid(self._gripper_process.pid), signal.SIGKILL)
                except:
                    pass
            except Exception as e:
                self.get_logger().warn(f"Error killing gripper process: {e}")
            finally:
                self._gripper_process = None
        
        # Kill any stale processes (same as launch scripts do)
        try:
            # Kill run_server and franka_panda_cl (robot server)
            subprocess.run(['pkill', '-9', 'run_server'], 
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=1)
            subprocess.run(['pkill', '-9', 'franka_panda_cl'], 
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=1)
            # Kill gripper processes
            subprocess.run(['pkill', '-9', 'gripper'], 
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=1)
            self.get_logger().info("Stale processes cleaned up")
        except Exception as e:
            self.get_logger().debug(f"Error cleaning up stale processes: {e}")
        
        self._server_launched = False
    
    def destroy_node(self):
        """Clean up resources on shutdown."""
        try:
            # Close connections gracefully
            if self._robot and not isinstance(self._robot, MockRobotInterface):
                try:
                    if self._robot.is_running_policy():
                        self._robot.terminate_current_policy()
                except Exception as e:
                    self.get_logger().warn(f"Error terminating robot policy: {e}")
            
            # Kill controller servers if we launched them
            if self._server_launched:
                self._kill_controller()
        except Exception as e:
            self.get_logger().warn(f"Error during node destruction: {e}")
        finally:
            super().destroy_node()


def main(args=None):
    """Main function to run the Polymetis Combined Node."""
    from rclpy.executors import MultiThreadedExecutor
    
    rclpy.init(args=args)
    
    # Create node
    node = PolymetisCombinedNode(use_mock=not POLYMETIS_AVAILABLE)
    
    # Use MultiThreadedExecutor to allow timer callbacks to run during blocking service calls
    # This ensures /joint_states continues publishing during robot movement
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # Ensure proper cleanup
        try:
            node.destroy_node()
        except Exception as e:
            print(f"Error destroying node: {e}")
        finally:
            rclpy.shutdown()


if __name__ == '__main__':
    main()

