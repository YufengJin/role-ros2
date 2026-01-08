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
  - /polymetis/robot_command: High-level arm control commands (joint positions from FrankaRobot)
  - /polymetis/gripper/command: Gripper control commands
- Services:
  - /polymetis/reset: Reset robot to home position
  - /polymetis/arm/move_to_joint_positions: Blocking joint position control
  - /polymetis/arm/move_to_ee_pose: Blocking end-effector pose control
  - And more for controller management and computation
- Mock interfaces for testing without hardware
- Proper PyTorch tensor conversion for Polymetis API

Design:
- FrankaRobot (robot.py) handles all IK computations via create_action_dict()
- FrankaRobot calls update_joints() and update_gripper() separately
- This node only receives and executes joint position commands (no IK)
- Gripper is handled separately via /polymetis/gripper/command topic

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
    from polymetis import RobotInterface, GripperInterface
    POLYMETIS_AVAILABLE = True
except ImportError:
    POLYMETIS_AVAILABLE = False
    print("Warning: Polymetis not available. Using MockRobotInterface and MockGripperInterface.")

# Import custom messages
from role_ros2.msg import (
    PolymetisRobotState, PolymetisRobotCommand, 
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
    from role_ros2.misc.transformations import add_poses, euler_to_quat, quat_to_euler
    IK_SOLVER_AVAILABLE = True
except ImportError as e:
    IK_SOLVER_AVAILABLE = False
    print(f"Warning: RobotIKSolver not available: {e}. High-level control commands will not work.")


# ============================================================================
# HYPERPARAMETERS - Control max velocities and speeds
# ============================================================================
# These parameters match robot_ik_solver.py configuration
# Reference: droid/droid/robot_ik/robot_ik_solver.py

# Joint velocity limits (rad/s) - matches robot_ik_solver.py
RELATIVE_MAX_JOINT_DELTA = np.array([0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2])  # Per-joint max delta
MAX_JOINT_DELTA = 0.2  # Maximum joint position delta per command (rad) - matches robot_ik_solver.max_joint_delta
MAX_JOINT_VELOCITY = 1.0  # Maximum joint velocity for blocking movements (rad/s)
MAX_JOINT_VELOCITY_NON_BLOCKING = 0.2  # Maximum joint velocity for non-blocking commands (rad/s)

# Cartesian velocity limits - matches robot_ik_solver.py
MAX_CARTESIAN_LIN_DELTA = 0.075  # Maximum linear position delta per command (m) - matches robot_ik_solver.max_lin_delta
MAX_CARTESIAN_ROT_DELTA = 0.15  # Maximum angular position delta per command (rad) - matches robot_ik_solver.max_rot_delta
MAX_CARTESIAN_LIN_VELOCITY = 0.1  # Maximum linear velocity (m/s)
MAX_CARTESIAN_ROT_VELOCITY = 0.3  # Maximum angular velocity (rad/s)

# Gripper limits - matches robot_ik_solver.py
MAX_GRIPPER_DELTA = 0.5  # Maximum gripper position delta per command (normalized 0-1) - matches robot_ik_solver.max_gripper_delta
DEFAULT_GRIPPER_SPEED = 0.15  # Default gripper movement speed (m/s)
DEFAULT_GRIPPER_FORCE = 0.1  # Default gripper force (normalized 0-1)

# Control frequency (Hz) - matches robot_ik_solver.py
CONTROL_HZ = 15.0  # Control frequency for velocity-to-delta conversion - matches robot_ik_solver.control_hz

# Time-to-go calculation parameters
MIN_TIME_TO_GO = 0.5  # Minimum time for movement (seconds)
MAX_TIME_TO_GO = 4.0  # Maximum time for movement (seconds)
MIN_TIME_TO_GO_SLOW = 5.0  # Minimum time for slow/safe movements (seconds)
MAX_TIME_TO_GO_SLOW = 40.0  # Maximum time for slow/safe movements (seconds)
MAX_VELOCITY_FOR_TIME_CALC = 1.0  # Maximum velocity for time calculation (rad/s)
MAX_VELOCITY_SLOW = 0.1  # Maximum velocity for slow/safe movements (rad/s)

# ============================================================================


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
    without physical robot hardware. State updates based on commands, simulating realistic motion.
    """
    
    def __init__(self, num_dofs: int = 7, update_rate: float = 50.0):
        """
        Initialize mock robot interface.
        
        Args:
            num_dofs: Number of degrees of freedom (default: 7 for Franka arm)
            update_rate: Update rate in Hz for state simulation (default: 50.0)
        """
        self.num_dofs = num_dofs
        self.update_rate = update_rate
        self.dt = 1.0 / update_rate
        
        # Franka rest pose (home position) - from default_metadata.yaml
        # This is the typical starting position for Franka robots
        rest_pose = [
            -0.13935425877571106,
            -0.020481698215007782,
            -0.05201413854956627,
            -2.0691256523132324,
            0.05058913677930832,
            2.0028650760650635,
            -0.9167874455451965
        ]
        
        # Current state (actual positions/velocities) - initialize to rest pose
        if TORCH_AVAILABLE:
            self._joint_positions = torch.tensor(rest_pose[:num_dofs], dtype=torch.float32)
            self._joint_velocities = torch.zeros(num_dofs, dtype=torch.float32)
            self._kp = torch.ones(num_dofs, dtype=torch.float32) * 40.0
            self._kd = torch.ones(num_dofs, dtype=torch.float32) * 4.0
            # Target state (desired positions/velocities from commands) - initialize to rest pose
            self._target_joint_positions = torch.tensor(rest_pose[:num_dofs], dtype=torch.float32)
            self._target_joint_velocities = torch.zeros(num_dofs, dtype=torch.float32)
        else:
            self._joint_positions = np.array(rest_pose[:num_dofs], dtype=np.float32)
            self._joint_velocities = np.zeros(num_dofs)
            self._kp = np.ones(num_dofs) * 40.0
            self._kd = np.ones(num_dofs) * 4.0
            self._target_joint_positions = np.array(rest_pose[:num_dofs], dtype=np.float32)
            self._target_joint_velocities = np.zeros(num_dofs)
        
        # In mock mode, we want state to update even without explicit policy start
        # This allows the robot to respond to commands immediately
        self._is_running_policy = True  # Changed from False to True for mock mode
        self._last_update_time = time.time()
        
        # Thread safety for state updates
        self._state_lock = threading.Lock()
        
        # Motion parameters (simulate impedance control response)
        self._max_velocity = 1.0  # rad/s
        self._position_gain = 5.0  # How fast to reach target position
        self._velocity_gain = 2.0  # How fast to reach target velocity
    
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
        For mock, we update target positions/velocities which will be gradually reached.
        
        Args:
            q: Desired joint positions (7 DOF) in radians
            qd: Desired joint velocities (7 DOF) in rad/s
            Kq: Position gains (stiffness) for each joint
            Kqd: Velocity gains (damping) for each joint
            tau: Desired joint torques (7 DOF) in Nm
        """
        if TORCH_AVAILABLE:
            if q is not None:
                self._target_joint_positions = torch.tensor(q, dtype=torch.float32)
            if qd is not None:
                self._target_joint_velocities = torch.tensor(qd, dtype=torch.float32)
            if Kq is not None:
                self._kp = torch.tensor(Kq, dtype=torch.float32)
            if Kqd is not None:
                self._kd = torch.tensor(Kqd, dtype=torch.float32)
        else:
            if q is not None:
                self._target_joint_positions = np.array(q)
            if qd is not None:
                self._target_joint_velocities = np.array(qd)
            if Kq is not None:
                self._kp = np.array(Kq)
            if Kqd is not None:
                self._kd = np.array(Kqd)
        
        self._is_running_policy = True
    
    def update_state(self, dt=None):
        """
        Update robot state based on target positions/velocities.
        
        Simulates impedance control: gradually moves toward target positions/velocities.
        This should be called periodically (e.g., in the state publishing loop).
        
        Args:
            dt: Time delta in seconds. If None, computed from last update time.
        """
        if not self._is_running_policy:
            return
        
        with self._state_lock:
            if dt is None:
                current_time = time.time()
                dt = min(current_time - self._last_update_time, 0.1)  # Cap dt to avoid large jumps
                self._last_update_time = current_time
            
            if TORCH_AVAILABLE:
                # Compute position error
                pos_error = self._target_joint_positions - self._joint_positions
                
                # Compute desired velocity (proportional control + target velocity)
                desired_velocity = self._position_gain * pos_error + self._target_joint_velocities
                
                # Limit velocity
                vel_norm = torch.norm(desired_velocity)
                if vel_norm > self._max_velocity:
                    desired_velocity = desired_velocity * (self._max_velocity / vel_norm)
                
                # Update velocity (exponential approach to desired velocity)
                vel_error = desired_velocity - self._joint_velocities
                self._joint_velocities += self._velocity_gain * vel_error * dt
                
                # Update position
                self._joint_positions += self._joint_velocities * dt
            else:
                # NumPy version
                pos_error = self._target_joint_positions - self._joint_positions
                desired_velocity = self._position_gain * pos_error + self._target_joint_velocities
                
                # Limit velocity
                vel_norm = np.linalg.norm(desired_velocity)
                if vel_norm > self._max_velocity:
                    desired_velocity = desired_velocity * (self._max_velocity / vel_norm)
                
                # Update velocity
                vel_error = desired_velocity - self._joint_velocities
                self._joint_velocities += self._velocity_gain * vel_error * dt
                
                # Update position
                self._joint_positions += self._joint_velocities * dt
    
    def get_joint_positions(self):
        """Get current joint positions (thread-safe)."""
        with self._state_lock:
            if TORCH_AVAILABLE:
                return self._joint_positions.clone() if isinstance(self._joint_positions, torch.Tensor) else self._joint_positions.copy()
            else:
                return self._joint_positions.copy()
    
    def get_joint_velocities(self):
        """Get current joint velocities (thread-safe)."""
        with self._state_lock:
            if TORCH_AVAILABLE:
                return self._joint_velocities.clone() if isinstance(self._joint_velocities, torch.Tensor) else self._joint_velocities.copy()
            else:
                return self._joint_velocities.copy()
    
    def get_robot_state(self):
        """Get robot state (mock implementation, thread-safe)."""
        with self._state_lock:
            # Create copies to ensure consistency
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
    
    def start_cartesian_impedance(self, Kx=None, Kxd=None):
        """
        Start cartesian impedance control mode.
        
        In mock mode, this is equivalent to starting joint impedance control.
        The actual cartesian control is handled by the IK solver in the bridge node.
        
        Args:
            Kx: Cartesian position gains (stiffness) - 6D [x, y, z, roll, pitch, yaw]
            Kxd: Cartesian velocity gains (damping) - 6D
        """
        # In mock mode, cartesian impedance is handled via IK solver
        # Just start the policy to enable control
        # Store cartesian gains if provided (for compatibility)
        if Kx is not None:
            # Convert 6D cartesian gains to 7D joint gains (simplified)
            if TORCH_AVAILABLE:
                # Use average of cartesian gains for joint gains
                avg_gain = torch.mean(torch.tensor(Kx, dtype=torch.float32))
                self._kp = torch.ones(self.num_dofs, dtype=torch.float32) * avg_gain
            else:
                avg_gain = np.mean(np.array(Kx))
                self._kp = np.ones(self.num_dofs) * avg_gain
        if Kxd is not None:
            if TORCH_AVAILABLE:
                avg_gain = torch.mean(torch.tensor(Kxd, dtype=torch.float32))
                self._kd = torch.ones(self.num_dofs, dtype=torch.float32) * avg_gain
            else:
                avg_gain = np.mean(np.array(Kxd))
                self._kd = np.ones(self.num_dofs) * avg_gain
        self._is_running_policy = True
    
    def update_desired_joint_positions(self, positions):
        """Update desired joint positions (non-blocking, thread-safe)."""
        with self._state_lock:
            if TORCH_AVAILABLE:
                if isinstance(positions, torch.Tensor):
                    self._target_joint_positions = positions.clone()
                else:
                    self._target_joint_positions = torch.tensor(positions, dtype=torch.float32)
            else:
                self._target_joint_positions = np.array(positions)
    
    def move_to_joint_positions(self, positions, time_to_go: float = 2.0):
        """Move to joint positions (blocking, thread-safe)."""
        with self._state_lock:
            if TORCH_AVAILABLE:
                if isinstance(positions, torch.Tensor):
                    self._target_joint_positions = positions.clone()
                else:
                    self._target_joint_positions = torch.tensor(positions, dtype=torch.float32)
            else:
                self._target_joint_positions = np.array(positions)
        
        # Simulate movement by updating state until target is reached
        start_time = time.time()
        dt = 1.0 / self.update_rate
        while time.time() - start_time < time_to_go:
            self.update_state(dt=dt)
            # Check if close enough to target (thread-safe read)
            with self._state_lock:
                if TORCH_AVAILABLE:
                    error = torch.norm(self._target_joint_positions - self._joint_positions).item()
                else:
                    error = np.linalg.norm(self._target_joint_positions - self._joint_positions)
            if error < 0.01:  # 0.01 rad tolerance
                break
            time.sleep(0.01)  # Small sleep to avoid busy loop
    
    def terminate_current_policy(self):
        """Terminate current policy."""
        self._is_running_policy = False
    
    def solve_inverse_kinematics(self, pos, quat, curr_joints, tol=None):
        """
        Solve inverse kinematics (mock implementation).
        
        In mock mode, this is a simplified implementation that returns current joints.
        Real IK would compute desired joints from position and orientation.
        
        Args:
            pos: Target position [x, y, z]
            quat: Target orientation quaternion [x, y, z, w]
            curr_joints: Current joint positions (used as initial guess)
            tol: Tolerance (optional, for compatibility)
        
        Returns:
            tuple: (joint_positions, success)
        """
        # Simple mock: return current joints (real IK would compute desired joints)
        # In a real implementation, this would use IK solver to compute target joints
        if TORCH_AVAILABLE:
            if isinstance(curr_joints, torch.Tensor):
                return curr_joints.clone(), True
            else:
                return torch.tensor(curr_joints, dtype=torch.float32), True
        else:
            return np.array(curr_joints), True
    
    def get_ee_pose(self):
        """
        Get end-effector pose (mock implementation, thread-safe).
        
        Uses simple forward kinematics approximation based on joint positions.
        For a more accurate FK, would need robot model, but this is sufficient for mock.
        """
        # Simple FK approximation: use joint positions to compute approximate EE pose
        # This is a simplified model - real FK would use DH parameters or URDF
        with self._state_lock:
            if TORCH_AVAILABLE:
                joints = self._joint_positions.cpu().numpy() if isinstance(self._joint_positions, torch.Tensor) else self._joint_positions.copy()
            else:
                joints = self._joint_positions.copy()
        
        # Simplified FK for Franka Panda (approximate)
        # Base to EE transformation (simplified)
        # x = 0.333 * cos(j1) * cos(j2) + 0.316 * cos(j1) * sin(j2) + ...
        # This is a very rough approximation
        x = 0.3 + 0.1 * joints[0] + 0.1 * joints[1]
        y = 0.0 + 0.1 * joints[0] - 0.1 * joints[2]
        z = 0.5 + 0.1 * joints[1] + 0.1 * joints[3]
        
        # Simple orientation (roll, pitch, yaw from joints)
        roll = joints[3]
        pitch = joints[4]
        yaw = joints[5]
        
        # Convert to quaternion (simplified)
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)
        
        quat = np.array([
            sr * cp * cy - cr * sp * sy,  # x
            cr * sp * cy + sr * cp * sy,  # y
            cr * cp * sy - sr * sp * cy,  # z
            cr * cp * cy + sr * sp * sy   # w
        ])
        
        if TORCH_AVAILABLE:
            pos = torch.tensor([x, y, z], dtype=torch.float32)
            quat = torch.tensor(quat, dtype=torch.float32)
        else:
            pos = np.array([x, y, z])
        
        return pos, quat


class MockGripperInterface:
    """
    Mock implementation of Polymetis GripperInterface for testing without hardware.
    
    State updates based on commands, simulating realistic motion.
    """
    
    def __init__(self, update_rate: float = 50.0):
        """
        Initialize mock gripper interface.
        
        Args:
            update_rate: Update rate in Hz for state simulation (default: 50.0)
        """
        self._current_width = 0.08  # Current width in meters
        self._target_width = 0.08  # Target width from commands
        self._is_moving = False
        self._last_update_time = time.time()
        self._update_rate = update_rate
        self._max_speed = 0.1  # m/s
        self.metadata = type('obj', (object,), {'max_width': 0.08})()
        
        # Thread safety for state updates
        self._state_lock = threading.Lock()
    
    def update_state(self, dt=None):
        """
        Update gripper state based on target width.
        
        Gradually moves toward target width. Should be called periodically.
        
        Args:
            dt: Time delta in seconds. If None, computed from last update time.
        """
        with self._state_lock:
            if dt is None:
                current_time = time.time()
                dt = min(current_time - self._last_update_time, 0.1)  # Cap dt
                self._last_update_time = current_time
            
            # Compute error
            width_error = self._target_width - self._current_width
            
            # Check if moving
            if abs(width_error) > 0.001:  # 1mm tolerance
                self._is_moving = True
                # Move toward target (proportional control)
                max_step = self._max_speed * dt
                step = np.clip(width_error, -max_step, max_step)
                self._current_width += step
                self._current_width = np.clip(self._current_width, 0.0, self.metadata.max_width)
            else:
                self._is_moving = False
                self._current_width = self._target_width
    
    def goto(self, width: float, speed: float = 0.05, force: float = 0.1, blocking: bool = False):
        """
        Move gripper to specified width.
        
        Args:
            width: Target width in meters (0=closed, max_width=open)
            speed: Movement speed (0.0-1.0) - affects max_speed
            force: Gripper force (0.0-1.0)
            blocking: Whether to wait for completion
        """
        with self._state_lock:
            self._target_width = max(0.0, min(width, self.metadata.max_width))
            # Adjust max speed based on speed parameter
            self._max_speed = 0.1 * speed if speed > 0 else 0.1
        
        if blocking:
            # Simulate movement by updating until target reached
            start_time = time.time()
            timeout = 5.0  # Max 5 seconds
            dt = 1.0 / self._update_rate
            while True:
                with self._state_lock:
                    error = abs(self._target_width - self._current_width)
                if error <= 0.001 or (time.time() - start_time) >= timeout:
                    break
                self.update_state(dt=dt)
                time.sleep(0.01)
            with self._state_lock:
                self._is_moving = False
    
    def get_state(self):
        """Get gripper state (thread-safe)."""
        with self._state_lock:
            # Create snapshot to ensure consistency
            width = float(self._current_width)  # Ensure float type
            is_moving = bool(self._is_moving)  # Ensure bool type
        
        class MockGripperState:
            def __init__(self, width, is_moving):
                self.width = float(width)  # Ensure float type
                self.is_moving = bool(is_moving)  # Ensure bool type
                self.is_grasped = bool(width < 0.01)  # Ensure bool type (not numpy.bool_)
                self.prev_command_successful = True
                self.error_code = 0
        
        return MockGripperState(width, is_moving)


class PolymetisCombinedNode(Node):
    """
    ROS 2 Node that bridges Polymetis Python SDK with ROS 2.
    
    This node:
    - Subscribes to /polymetis/robot_command for arm joint position commands
    - Subscribes to /polymetis/gripper/command for gripper commands
    - Publishes /joint_states at high frequency (50Hz+)
    - Handles both Arm and Gripper control
    - Converts ROS messages to PyTorch tensors for Polymetis API
    
    Design:
    - FrankaRobot (robot.py) handles all IK computations
    - This node only receives and executes joint position commands
    """
    
    def _init_gripper_interface_with_retry(self, ip_address: str, max_retries: int = 10, retry_delay: float = 0.5):
        """
        Initialize GripperInterface with retry mechanism to handle metadata initialization race condition.
        
        Problem: When GripperInterface.__init__() is called, it immediately tries to get metadata
        via GetRobotClientMetadata(). However, the gripper server's metadata may not be initialized yet
        (franka_hand_client hasn't called InitRobotClient), causing:
        1. gRPC serialization error (TypeError) when server returns None
        2. Metadata unavailable warning
        
        Solution: Retry initialization with exponential backoff until metadata is available or max retries reached.
        
        Args:
            ip_address: IP address of the gripper server
            max_retries: Maximum number of retry attempts
            retry_delay: Initial delay between retries in seconds
        
        Returns:
            GripperInterface: Initialized gripper interface, or None if all retries failed
        """
        import grpc
        
        for attempt in range(max_retries):
            try:
                if attempt > 0:
                    # Only log retry attempts at INFO level to show retry mechanism is working
                    self.get_logger().info(
                        f"Initializing GripperInterface (attempt {attempt + 1}/{max_retries})..."
                    )
                gripper = GripperInterface(ip_address=ip_address)
                
                # Check if metadata was successfully retrieved
                if hasattr(gripper, 'metadata') and gripper.metadata is not None:
                    self.get_logger().info(
                        f"✓ GripperInterface initialized successfully with metadata "
                        f"(max_width={gripper.metadata.max_width})"
                    )
                    return gripper
                else:
                    # Metadata not available yet, set default and continue
                    self.get_logger().info(
                        f"GripperInterface created but metadata not available yet (attempt {attempt + 1})"
                    )
                    # Set default metadata to avoid None errors
                    try:
                        import polymetis_pb2
                        default_metadata = polymetis_pb2.GripperMetadata()
                        default_metadata.polymetis_version = "0.2"
                        default_metadata.hz = 50
                        default_metadata.max_width = 0.08  # Default for Franka Hand
                        gripper.metadata = default_metadata
                        self.get_logger().info(
                            "Set default gripper metadata (server metadata not available yet, will retry later)"
                        )
                        # Return it but we'll try to get real metadata later
                        return gripper
                    except ImportError as e:
                        self.get_logger().warn(f"Could not import polymetis_pb2: {e}")
                        if attempt < max_retries - 1:
                            time.sleep(retry_delay * (attempt + 1))  # Exponential backoff
                            continue
                        return None
                
            except (grpc.RpcError, TypeError) as e:
                # Catch both gRPC errors and serialization errors
                error_type = type(e).__name__
                if attempt < max_retries - 1:
                    wait_time = retry_delay * (attempt + 1)  # Exponential backoff
                    # Log at INFO level to show retry mechanism is working
                    self.get_logger().info(
                        f"GripperInterface initialization failed ({error_type}): {str(e)[:100]}. "
                        f"Retrying in {wait_time:.1f}s... (attempt {attempt + 1}/{max_retries})"
                    )
                    time.sleep(wait_time)
                else:
                    self.get_logger().warn(
                        f"GripperInterface initialization failed after {max_retries} attempts: {e}"
                    )
            except Exception as e:
                # Other unexpected errors
                self.get_logger().error(f"Unexpected error initializing GripperInterface: {e}")
                if attempt < max_retries - 1:
                    time.sleep(retry_delay * (attempt + 1))
                else:
                    return None
        
        # All retries exhausted
        self.get_logger().error(
            f"Failed to initialize GripperInterface after {max_retries} attempts. "
            "Gripper functionality will be limited."
        )
        return None
    
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
                self._robot = MockRobotInterface(num_dofs=7, update_rate=self.publish_rate)
                self._gripper = MockGripperInterface(update_rate=self.publish_rate)
            else:
                self.get_logger().info(f"Connecting to Polymetis server at {ip_address}")
                # Wait a bit for servers to start if we just launched them
                if self._server_launched:
                    self.get_logger().info("Waiting for Polymetis servers to initialize...")
                    time.sleep(5)
                self._robot = RobotInterface(ip_address=ip_address)
                
                # Initialize gripper with retry mechanism to handle metadata race condition
                # This avoids gRPC serialization errors when server metadata is not yet initialized
                self._gripper = self._init_gripper_interface_with_retry(
                    ip_address=ip_address,
                    max_retries=10,
                    retry_delay=0.5
                )
                
                # Fallback to MockGripperInterface if initialization failed
                if self._gripper is None:
                    self.get_logger().warn(
                        "Failed to initialize real GripperInterface. Using MockGripperInterface."
                    )
                    self._gripper = MockGripperInterface(update_rate=self.publish_rate)
                else:
                    # Try to refresh metadata if we got default metadata initially
                    # This happens in a background thread to not block initialization
                    def refresh_metadata():
                        """Try to get real metadata from server after client initializes."""
                        max_attempts = 20
                        for attempt in range(max_attempts):
                            try:
                                time.sleep(1.0)  # Wait for client to initialize
                                # Try to get metadata directly via gRPC
                                import grpc
                                import polymetis_pb2
                                from polymetis_pb2_grpc import GripperServerStub
                                
                                channel = grpc.insecure_channel(f"{ip_address}:50052")
                                stub = GripperServerStub(channel)
                                
                                try:
                                    metadata = stub.GetRobotClientMetadata(polymetis_pb2.Empty())
                                    if metadata is not None:
                                        self._gripper.metadata = metadata
                                        self.get_logger().info(
                                            f"✓ Updated gripper metadata from server "
                                            f"(max_width={metadata.max_width})"
                                        )
                                        channel.close()
                                        return
                                except grpc.RpcError:
                                    # Metadata still not available, continue waiting
                                    pass
                                
                                channel.close()
                                
                            except Exception as e:
                                self.get_logger().debug(f"Metadata refresh attempt {attempt + 1} failed: {e}")
                        
                        self.get_logger().debug(
                            "Could not refresh metadata from server, using default values"
                        )
                    
                    # Start metadata refresh in background thread
                    threading.Thread(target=refresh_metadata, daemon=True).start()
                
                self.get_logger().info("Successfully connected to Polymetis server")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to Polymetis: {e}. Using mock interfaces.")
            self._robot = MockRobotInterface(num_dofs=7, update_rate=self.publish_rate)
            self._gripper = MockGripperInterface(update_rate=self.publish_rate)
        
        # Initialize IK solver if available
        self._ik_solver = None
        self._ik_solver_lock = threading.Lock()  # Lock for thread-safe IK solver access (MuJoCo is not thread-safe)
        if IK_SOLVER_AVAILABLE:
            try:
                self.get_logger().info("Initializing RobotIKSolver...")
                self._ik_solver = RobotIKSolver()
                self.get_logger().info("✓ RobotIKSolver initialized successfully")
            except Exception as e:
                # Log error but don't fail - IK solver is optional for basic functionality
                error_str = str(e)
                if 'eq_active' in error_str or 'MjModel' in error_str:
                    self.get_logger().warning(
                        f"RobotIKSolver initialization failed due to MuJoCo/dm_control version incompatibility: {error_str}\n"
                        "This is a known issue - IK solver will be disabled. Low-level control will still work.\n"
                        "To fix: Update MuJoCo and dm_control to compatible versions."
                    )
                else:
                    self.get_logger().warning(f"RobotIKSolver initialization failed: {error_str}\nIK solver will be disabled. Low-level control will still work.")
                self._ik_solver = None
        else:
            self.get_logger().error(
                "✗ RobotIKSolver import failed. IK_SOLVER_AVAILABLE=False. "
                "High-level control commands will not work. "
                "Please check if role_ros2.robot_ik module is properly installed."
            )
            print("[ERROR] RobotIKSolver import failed. Check dependencies.")
        
        # Get max gripper width from metadata (now guaranteed to exist)
        try:
            if hasattr(self._gripper, 'metadata') and self._gripper.metadata is not None:
                self._max_gripper_width = self._gripper.metadata.max_width
                self.get_logger().info(f"Using gripper max_width={self._max_gripper_width} from metadata")
            else:
                self._max_gripper_width = 0.08  # Default for Franka hand
                self.get_logger().info("Using default max_gripper_width=0.08 (gripper metadata not available)")
        except (AttributeError, TypeError) as e:
            self._max_gripper_width = 0.08  # Default for Franka hand
            self.get_logger().info(f"Using default max_gripper_width=0.08 (error accessing metadata: {type(e).__name__})")
        
        # State storage and thread safety
        self._command_lock = threading.Lock()
        self._last_command_time = None
        
        # Callback groups for parallel execution with MultiThreadedExecutor
        # ReentrantCallbackGroup: allows timer to run in parallel with services
        # MutuallyExclusiveCallbackGroup: blocking services execute one at a time
        self._timer_callback_group = ReentrantCallbackGroup()
        self._service_callback_group = MutuallyExclusiveCallbackGroup()
        
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
        self.get_logger().info('Subscribers: /polymetis/robot_command, /polymetis/gripper/command')
        self.get_logger().info('Services: /polymetis/reset, /polymetis/arm/start_cartesian_impedance, /polymetis/arm/start_joint_impedance, /polymetis/arm/terminate_policy, /polymetis/arm/move_to_joint_positions, /polymetis/arm/move_to_ee_pose, /polymetis/arm/solve_ik, /polymetis/arm/compute_fk, /polymetis/arm/compute_time_to_go')
        
        # Log IK solver status (IK is used for FK and services, but robot_command_callback uses joint positions directly)
        if self._ik_solver is None:
            self.get_logger().warn("=" * 80)
            self.get_logger().warn("⚠️  RobotIKSolver is NOT available!")
            self.get_logger().warn("  - /polymetis/robot_command still works (receives joint positions)")
            self.get_logger().warn("  - FK computation for mock mode will use simplified model")
            self.get_logger().warn("  - IK services (/polymetis/arm/solve_ik) will not work")
            if IK_SOLVER_AVAILABLE:
                self.get_logger().warn("  - IK_SOLVER_AVAILABLE: True (import succeeded)")
                self.get_logger().warn("  - _ik_solver: None (initialization failed)")
                self.get_logger().warn("  - Common causes: MuJoCo/dm_control version incompatibility")
            else:
                self.get_logger().warn("  - IK_SOLVER_AVAILABLE: False (import failed)")
            self.get_logger().warn("=" * 80)
        else:
            self.get_logger().info("=" * 80)
            self.get_logger().info("✓ RobotIKSolver is available")
            self.get_logger().info("  - Used for: FK computation, IK services")
            self.get_logger().info("  - Note: FrankaRobot (robot.py) handles IK for robot_command")
            self.get_logger().info("=" * 80)
        
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
    
    def _robot_command_callback(self, msg: PolymetisRobotCommand):
        """
        Callback for arm joint position commands (NON-BLOCKING ONLY).
        
        Design Philosophy:
        - FrankaRobot (robot.py) handles ALL IK computations via create_action_dict()
        - FrankaRobot calls update_joints() and update_gripper() SEPARATELY
        - This callback ONLY receives and executes joint positions (no IK needed)
        - Gripper is handled by _gripper_command_callback via /polymetis/gripper/command
        
        Data Flow:
        1. User calls robot.update_command(cartesian_velocity, ...)
        2. robot.py computes IK -> joint_position via create_action_dict()
        3. robot.py calls update_joints(joint_position) -> publishes to /polymetis/robot_command
        4. This callback receives joint_position and sends to Polymetis
        5. robot.py calls update_gripper() -> publishes to /polymetis/gripper/command
        
        Note: The command array is [joint_positions (7), gripper_position (1)]
        We only use the first 7 values; gripper is handled separately.
        
        Works identically for both Mock and real robot modes.
        
        Args:
            msg: PolymetisRobotCommand with joint positions (blocking flag ignored)
        """
        # Skip if controller is being loaded (same as robot.py)
        if self._controller_not_loaded:
            self.get_logger().debug("Controller loading, skipping command")
            return
        
        # Debug: Log command received
        self.get_logger().debug(
            f"Received robot command: action_space={msg.action_space}, "
            f"command_len={len(msg.command)}, blocking={msg.blocking}"
        )
        
        try:
            command = list(msg.command)
            
            # Validate command length (should have at least 7 joint positions)
            if len(command) < 7:
                self.get_logger().error(f"Invalid command length: {len(command)}, expected at least 7")
                return
            
            # Extract joint positions (first 7 values)
            # Note: FrankaRobot.update_joints() sends joint_position + gripper_position
            # The gripper is handled separately via /polymetis/gripper/command
            joint_position = command[:7]
            
            if not TORCH_AVAILABLE:
                self.get_logger().error("PyTorch not available, cannot process command")
                return
            
            pos_tensor = torch.tensor(joint_position, dtype=torch.float32)
            self.get_logger().debug(
                f"Target joint position: [{', '.join([f'{p:.3f}' for p in joint_position])}]"
            )
            
            def helper_non_blocking():
                """
                Non-blocking helper - same as robot.py:
                1. Ensure controller is running
                2. Send command
                """
                try:
                    self.get_logger().debug("helper_non_blocking: Starting...")
                    
                    # Step 1: Ensure controller is running (blocking wait)
                    if not self._robot.is_running_policy():
                        self.get_logger().debug("helper_non_blocking: Controller not running, starting...")
                        self._controller_not_loaded = True
                        self._robot.start_cartesian_impedance()
                        self._controller_mode = "cartesian_impedance"
                        
                        # Wait for controller to be ready (with timeout)
                        timeout = time.time() + 5
                        wait_count = 0
                        while not self._robot.is_running_policy():
                            time.sleep(0.01)
                            wait_count += 1
                            if time.time() > timeout:
                                self.get_logger().warn("helper_non_blocking: Controller timeout, retrying...")
                                self._robot.start_cartesian_impedance()
                                timeout = time.time() + 5
                        
                        self.get_logger().debug(f"helper_non_blocking: Controller ready after {wait_count * 0.01:.2f}s")
                        self._controller_not_loaded = False
                    else:
                        self.get_logger().debug("helper_non_blocking: Controller already running")
                    
                    # Step 2: Send command
                    self.get_logger().debug("helper_non_blocking: Sending joint position command...")
                    try:
                        self._robot.update_desired_joint_positions(pos_tensor)
                        self.get_logger().debug("helper_non_blocking: Command sent successfully")
                    except Exception as e:
                        # Log gRPC errors for debugging
                        self.get_logger().warn(f"helper_non_blocking: update_desired_joint_positions error: {e}")
                        
                except Exception as e:
                    error_msg = f"helper_non_blocking error: {e}\n{traceback.format_exc()}"
                    self.get_logger().error(error_msg)
                    self._controller_not_loaded = False
            
            # Run in thread (same as robot.py run_threaded_command)
            self.get_logger().debug("Starting helper_non_blocking thread...")
            thread = threading.Thread(target=helper_non_blocking, daemon=True)
            thread.start()
            self.get_logger().debug(f"helper_non_blocking thread started (daemon={thread.daemon})")
            
            self.get_logger().debug(f"✓ Processed non-blocking robot command: {msg.action_space}")
            
        except Exception as e:
            error_msg = f"Error processing robot command: {e}\n{traceback.format_exc()}"
            self.get_logger().error(error_msg)
            print(f"[ERROR] {error_msg}")
    
    def _compute_forward_kinematics(self, joint_positions):
        """
        Compute forward kinematics for end-effector pose.
        
        Uses robot_model for real robot, IK solver physics for mock robot,
        or fallback to get_ee_pose().
        
        Args:
            joint_positions: Joint positions (7 DOF) - can be list, numpy array, or torch tensor
            
        Returns:
            tuple: (position [x, y, z], quaternion [x, y, z, w])
        """
        try:
            if TORCH_AVAILABLE and hasattr(self._robot, 'robot_model'):
                # Real robot: use robot_model.forward_kinematics
                if isinstance(joint_positions, torch.Tensor):
                    joint_pos_tensor = joint_positions
                else:
                    joint_pos_tensor = torch.Tensor(joint_positions)
                pos, quat = self._robot.robot_model.forward_kinematics(joint_pos_tensor)
                # Convert to numpy
                if isinstance(pos, torch.Tensor):
                    pos = pos.cpu().numpy()
                if isinstance(quat, torch.Tensor):
                    quat = quat.cpu().numpy()
                return pos, quat
            elif isinstance(self._robot, MockRobotInterface) and self._ik_solver is not None:
                # Mock robot with IK solver: use IK solver's physics for accurate FK
                # ee_pose should be panda_link8 frame, not panda_hand frame
                # NOTE: MuJoCo physics is NOT thread-safe, must use lock
                with self._ik_solver_lock:
                    joint_pos = np.array(joint_positions)
                    joint_vel = np.zeros(7)  # Use zero velocity for FK
                    
                    # Update physics state with current joint positions
                    self._ik_solver._arm.update_state(self._ik_solver._physics, joint_pos, joint_vel)
                    
                    # Get panda_link8 body pose (not wrist_site which is in panda_hand)
                    # Find panda_link8 body in the MJCF model
                    link8_body = self._ik_solver._arm.mjcf_model.find("body", "panda_link8")
                    if link8_body is not None:
                        # Get panda_link8 body position and orientation from physics
                        body_bind = self._ik_solver._physics.bind(link8_body)
                        pos = body_bind.xpos.copy()  # 3D position
                        quat_mat = body_bind.xmat.copy().reshape(3, 3)  # Rotation matrix (3x3)
                        
                        # Convert rotation matrix to quaternion
                        quat_rot = R.from_matrix(quat_mat)
                        quat = quat_rot.as_quat()  # [x, y, z, w]
                    else:
                        # Fallback: use wrist_site and transform from hand frame to link8 frame
                        # wrist_site is in panda_hand, which is rotated -45deg around Z relative to link8
                        # In XML: <body name="panda_hand" euler="0 0 -0.785398163397">
                        # This means hand is rotated -45deg (euler="0 0 -0.785398163397") relative to link8
                        site_bind = self._ik_solver._physics.bind(self._ik_solver._arm.wrist_site)
                        wrist_pos = site_bind.xpos.copy()
                        wrist_quat_mat = site_bind.xmat.copy().reshape(3, 3)
                        
                        # Transform from hand frame to link8 frame
                        # In world frame: wrist_rot = link8_rot * hand_rot (hand_rot is -45deg around Z)
                        # So: link8_rot = wrist_rot * inverse(hand_rot)
                        # hand_rot is -45deg around Z, so inverse is +45deg
                        hand_rot = R.from_euler('z', -0.785398163397, degrees=False)  # -45deg (hand relative to link8)
                        wrist_rot = R.from_matrix(wrist_quat_mat)
                        # Compose: link8_rot = wrist_rot * hand_rot.inv()
                        link8_rot = wrist_rot * hand_rot.inv()
                        
                        # Position: wrist_site is at origin of hand frame (0,0,0 in hand frame)
                        # hand frame origin is same as link8 frame origin (no translation in XML: pos="0 0 0.107" is link8 relative to link7)
                        # So position is the same
                        pos = wrist_pos.copy()
                        quat = link8_rot.as_quat()  # [x, y, z, w]
                
                return pos, quat
            else:
                # Fallback: use get_ee_pose (may use simplified FK for mock)
                pos, quat = self._robot.get_ee_pose()
                # Convert to numpy if needed
                if isinstance(pos, torch.Tensor):
                    pos = pos.cpu().numpy()
                if isinstance(quat, torch.Tensor):
                    quat = quat.cpu().numpy()
                return pos, quat
        except Exception as e:
            self.get_logger().warn(f"Failed to compute forward kinematics: {e}")
            # Return default pose on error
            return np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.0, 0.0, 1.0])
    
    def _get_robot_state(self):
        """
        Get comprehensive robot state similar to robot.py get_robot_state().
        
        Returns:
            tuple: (state_dict, timestamp_dict)
        """
        try:
            # Record ROS time before getting data from interface (for polymetis_timestamp_ns)
            data_ros_time = self.get_clock().now()
            data_timestamp_ns = data_ros_time.nanoseconds
            
            robot_state = self._robot.get_robot_state()
            gripper_state = self._gripper.get_state()
            
            # Get gripper position (normalized)
            gripper_position = 1 - (gripper_state.width / self._max_gripper_width)
            
            # Get cartesian position via forward kinematics
            # ee_pose is frame fr3_panda_link8
            pos, quat = self._compute_forward_kinematics(robot_state.joint_positions)
            cartesian_position = pos.tolist() + quat_to_euler(quat).tolist()
            ee_quat = quat.tolist() if hasattr(quat, 'tolist') else list(quat)
            
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
                "gripper_width": float(gripper_state.width),
                "gripper_is_grasped": bool(getattr(gripper_state, 'is_grasped', False)),
                "gripper_is_moving": bool(getattr(gripper_state, 'is_moving', False)),
                "gripper_prev_command_successful": bool(getattr(gripper_state, 'prev_command_successful', True)),
                "gripper_error_code": int(getattr(gripper_state, 'error_code', 0)),
                "joint_positions": to_list(robot_state.joint_positions),
                "joint_velocities": to_list(robot_state.joint_velocities),
                "joint_torques_computed": to_list(robot_state.joint_torques_computed),
                "prev_joint_torques_computed": to_list(robot_state.prev_joint_torques_computed),
                "prev_joint_torques_computed_safened": to_list(robot_state.prev_joint_torques_computed_safened),
                "motor_torques_measured": to_list(robot_state.motor_torques_measured),
                "prev_controller_latency_ms": float(robot_state.prev_controller_latency_ms),
                "prev_command_successful": bool(robot_state.prev_command_successful),  # Ensure bool type
            }
            
            # Use ROS time for polymetis_timestamp_ns (recorded before getting data)
            timestamp_dict = {
                "robot_timestamp_seconds": data_timestamp_ns // 1_000_000_000,
                "robot_timestamp_nanos": data_timestamp_ns % 1_000_000_000,
                "polymetis_timestamp_ns": data_timestamp_ns,  # ROS time in nanoseconds
            }
            
            return state_dict, timestamp_dict
            
        except Exception as e:
            self.get_logger().error(f"Error getting robot state: {e}\n{traceback.format_exc()}")
            # Return empty state on error
            return {
                "cartesian_position": [0.0] * 6,
                "gripper_position": 0.0,
                "gripper_width": 0.0,
                "gripper_is_grasped": False,
                "gripper_is_moving": False,
                "gripper_prev_command_successful": True,
                "gripper_error_code": 0,
                "joint_positions": [0.0] * 7,
                "joint_velocities": [0.0] * 7,
                "joint_torques_computed": [0.0] * 7,
                "prev_joint_torques_computed": [0.0] * 7,
                "prev_joint_torques_computed_safened": [0.0] * 7,
                "motor_torques_measured": [0.0] * 7,
                "prev_controller_latency_ms": 0.0,
                "prev_command_successful": False,
            }, {"robot_timestamp_seconds": 0, "robot_timestamp_nanos": 0, "polymetis_timestamp_ns": 0}
    
    def _publish_states(self):
        """
        Publish joint states, robot state, gripper state, EE pose, and controller status at high frequency.
        
        In mock mode, updates state once and uses consistent snapshot for all publications
        to avoid asynchronous state issues.
        """
        now = self.get_clock().now()
        
        # Update mock state if using mock interfaces (simulate motion based on commands)
        # Use consistent dt for both robot and gripper to ensure synchronized updates
        if isinstance(self._robot, MockRobotInterface) or isinstance(self._gripper, MockGripperInterface):
            # Calculate dt based on expected update rate
            dt = 1.0 / self.publish_rate
            
            # Update both with same dt to ensure consistency
            if isinstance(self._robot, MockRobotInterface):
                self._robot.update_state(dt=dt)
            if isinstance(self._gripper, MockGripperInterface):
                self._gripper.update_state(dt=dt)
        
        # Publish all states using consistent snapshot
        # All publish methods will read from the same state snapshot
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
            
            # Gripper state (use data from state_dict to avoid duplicate get_state() calls)
            msg.gripper_width = float(state_dict.get("gripper_width", 0.0))
            msg.gripper_position = float(state_dict["gripper_position"])
            msg.gripper_is_grasped = bool(state_dict.get("gripper_is_grasped", False))
            msg.gripper_is_moving = bool(state_dict.get("gripper_is_moving", False))
            msg.gripper_prev_command_successful = bool(state_dict.get("gripper_prev_command_successful", True))
            msg.gripper_error_code = int(state_dict.get("gripper_error_code", 0))
            
            # Controller info
            msg.prev_controller_latency_ms = float(state_dict["prev_controller_latency_ms"])
            msg.prev_command_successful = bool(state_dict["prev_command_successful"])  # Ensure bool type
            msg.is_running_policy = bool(self._robot.is_running_policy() if hasattr(self._robot, 'is_running_policy') else False)  # Ensure bool type
            
            # Polymetis timestamp (ROS time recorded before getting data from interface)
            msg.polymetis_timestamp_ns = int(timestamp_dict.get("polymetis_timestamp_ns", 0))
            
            # Publish
            self._robot_state_publisher.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f"Error publishing robot state: {e}\n{traceback.format_exc()}")
    
    def _publish_gripper_state(self):
        """
        Publish gripper state under /polymetis/gripper_state.
        """
        try:
            # Record ROS time before getting data from interface (for polymetis_timestamp_ns)
            data_ros_time = self.get_clock().now()
            data_timestamp_ns = data_ros_time.nanoseconds
            
            gripper_state = self._gripper.get_state()
            
            # Create PolymetisGripperState message
            msg = PolymetisGripperState()
            now = self.get_clock().now()
            msg.header.stamp = now.to_msg()
            msg.header.frame_id = 'gripper_link'
            
            msg.width = float(gripper_state.width)
            msg.position = float(1 - (gripper_state.width / self._max_gripper_width))  # Normalized
            msg.is_grasped = bool(getattr(gripper_state, 'is_grasped', False))  # Ensure bool type
            msg.is_moving = bool(getattr(gripper_state, 'is_moving', False))  # Ensure bool type
            msg.prev_command_successful = bool(getattr(gripper_state, 'prev_command_successful', True))  # Ensure bool type
            msg.error_code = int(getattr(gripper_state, 'error_code', 0))  # Ensure int type
            msg.max_width = self._max_gripper_width
            
            # Use ROS time for polymetis_timestamp_ns (recorded before getting data from interface)
            msg.polymetis_timestamp_ns = int(data_timestamp_ns)
            
            # Publish
            self._gripper_state_publisher.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f"Error publishing gripper state: {e}\n{traceback.format_exc()}")
    
    def _publish_ee_pose(self):
        """
        Publish end-effector pose under /polymetis/arm/ee_pose.
        """
        try:
            # Get current joint positions
            joint_positions = self._robot.get_joint_positions()
            # Use compute_forward_kinematics for accurate FK (uses IK solver for mock)
            pos, quat = self._compute_forward_kinematics(joint_positions)
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
                msg.is_running_policy = bool(self._robot.is_running_policy())  # Ensure bool type
            else:
                msg.is_running_policy = bool(self._robot.is_running_policy() if hasattr(self._robot, 'is_running_policy') else False)  # Ensure bool type
            
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
            speed = msg.speed if msg.speed > 0 else DEFAULT_GRIPPER_SPEED
            force = msg.force if msg.force > 0 else DEFAULT_GRIPPER_FORCE
            blocking = msg.blocking
            
            if blocking:
                # Run in separate thread to avoid blocking
                def gripper_thread():
                    self._gripper.goto(width=width, speed=speed, force=force, blocking=True)
                threading.Thread(target=gripper_thread, daemon=True).start()
            else:
                self._gripper.goto(width=width, speed=speed, force=force, blocking=False)
            
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
            self._gripper.goto(width=0.0, speed=DEFAULT_GRIPPER_SPEED, force=DEFAULT_GRIPPER_FORCE, blocking=True)
            
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
                    
                    # Calculate time to go (slow speed for safety)
                    curr_joints = self._robot.get_joint_positions()
                    displacement = torch.abs(target_joints_tensor - curr_joints)
                    max_displacement = torch.max(displacement).item()
                    time_to_go = min(MAX_TIME_TO_GO_SLOW, max(MIN_TIME_TO_GO_SLOW, max_displacement / MAX_VELOCITY_SLOW))
                    
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
                    time_to_go = min(MAX_TIME_TO_GO_SLOW, max(MIN_TIME_TO_GO_SLOW, max_displacement / MAX_VELOCITY_SLOW))
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
            
            # Forward kinematics to get current pose using compute_forward_kinematics
            # Convert to list for _compute_forward_kinematics
            if isinstance(original_joints_tensor, torch.Tensor):
                original_joints_list = original_joints_tensor.cpu().numpy().tolist()
            else:
                original_joints_list = original_joints_tensor.tolist() if hasattr(original_joints_tensor, 'tolist') else list(original_joints_tensor)
            pos, quat = self._compute_forward_kinematics(original_joints_list)
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
                        time_to_go = min(MAX_TIME_TO_GO, max(MIN_TIME_TO_GO, max_displacement / MAX_VELOCITY_FOR_TIME_CALC))
                    
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
                time_to_go = min(MAX_TIME_TO_GO, max(MIN_TIME_TO_GO, max_displacement / MAX_VELOCITY_FOR_TIME_CALC))
            
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
                    
                    # Compute error using compute_forward_kinematics
                    if isinstance(joint_pos, torch.Tensor):
                        joint_pos_list = joint_pos.cpu().numpy().tolist()
                    else:
                        joint_pos_list = joint_pos.tolist() if hasattr(joint_pos, 'tolist') else list(joint_pos)
                    pos_output, quat_output = self._compute_forward_kinematics(joint_pos_list)
                    # Convert to torch for error computation
                    pos_output_tensor = torch.tensor(pos_output, dtype=torch.float32)
                    quat_output_tensor = torch.tensor(quat_output, dtype=torch.float32)
                    # Simple error computation
                    pos_error = torch.norm(pos - pos_output_tensor).item()
                    quat_error = torch.norm(quat - quat_output_tensor).item()
                    response.error = pos_error + quat_error
                    response.message = "IK solved successfully" if success else "IK solution not found"
                    
                    response.success = bool(success)  # Ensure bool type (not numpy.bool_)
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
            
            # Use compute_forward_kinematics for accurate FK (uses IK solver for mock)
            pos, quat = self._compute_forward_kinematics(request.joint_positions)
            response.position = pos.tolist() if hasattr(pos, 'tolist') else list(pos)
            response.orientation = quat.tolist() if hasattr(quat, 'tolist') else list(quat)
            response.success = True
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
                        response.time_to_go = min(MAX_TIME_TO_GO, max(MIN_TIME_TO_GO, max_displacement / MAX_VELOCITY_FOR_TIME_CALC))
                    
                    response.success = True
                else:
                    # Fallback: simple heuristic
                    desired_pos = np.array(request.desired_joint_positions)
                    curr_pos = np.array(self._robot.get_joint_positions())
                    displacement = np.abs(desired_pos - curr_pos)
                    max_displacement = np.max(displacement)
                    response.time_to_go = min(MAX_TIME_TO_GO, max(MIN_TIME_TO_GO, max_displacement / MAX_VELOCITY_FOR_TIME_CALC))
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

