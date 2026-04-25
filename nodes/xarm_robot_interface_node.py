#!/usr/bin/env python3
"""
xArm Robot Interface Node - ROS 2 Driver for UFACTORY xArm6

Mirrors the topic / service interface of franka_robot_interface_node.py so the
same downstream code (e.g. RobotEnv, trajectory collection scripts) can drive
either an xArm or a Franka by switching the namespace.

Internals:
- Position units between ROS and xArm SDK are bridged at this node's boundary:
  ROS uses meters, the xArm SDK uses millimeters.
- Angles are kept in radians end-to-end via XArmAPI(is_radian=True).
- xArm has no native impedance controller; "joint_impedance" /
  "cartesian_impedance" services are mapped to xArm mode 1 (servo streaming).
  Kp/Kd values are cached for ArmState publishing only.

Modes used (xArm SDK):
- 0: position control (blocking moves with set_servo_angle / set_position)
- 1: servo streaming (set_servo_angle_j / set_servo_cartesian, ~100 Hz)
- 4: joint velocity (vc_set_joint_velocity)
- 5: cartesian velocity (vc_set_cartesian_velocity)

Author: Role-ROS2 Team
"""

import os
import time
import threading
import traceback
from typing import List, Optional, Tuple

import numpy as np
from scipy.spatial.transform import Rotation as R

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

try:
    from xarm.wrapper import XArmAPI
    XARM_AVAILABLE = True
except ImportError:
    XARM_AVAILABLE = False
    print("Warning: xarm-python-sdk not available. Mock mode will be used.")

from role_ros2.msg import (
    ArmState, ControllerStatus,
    JointPositionCommand, JointVelocityCommand,
    CartesianPositionCommand, CartesianVelocityCommand
)
from role_ros2.srv import (
    Reset, StartCartesianImpedance, StartJointImpedance, StartJointVelocity,
    TerminatePolicy, MoveToJointPositions, MoveToEEPose,
    SolveIK, ComputeFK, ComputeTimeToGo
)

try:
    from role_ros2.misc.transformations import quat_to_euler, euler_to_quat
    TRANSFORM_UTILS_AVAILABLE = True
except ImportError:
    TRANSFORM_UTILS_AVAILABLE = False


# ============================================================================
# Hyperparameters
# ============================================================================
M_TO_MM = 1000.0

# Time-to-go bounds for adaptive speed calculation (mirror franka node).
MIN_TIME_TO_GO = 0.5
MAX_TIME_TO_GO = 4.0
MIN_TIME_TO_GO_SLOW = 5.0
MAX_TIME_TO_GO_SLOW = 40.0
MAX_VELOCITY_FOR_TIME_CALC = 1.0   # rad/s
MAX_VELOCITY_SLOW = 0.1            # rad/s

# Default streaming speed/accel passed to set_servo_cartesian (SDK reserved).
DEFAULT_SERVO_CART_SPEED = 100.0   # mm/s
DEFAULT_SERVO_CART_ACC = 2000.0    # mm/s^2


# ============================================================================
# Mock interface (no hardware)
# ============================================================================
class MockXArmInterface:
    """
    Minimal stand-in for xarm.wrapper.XArmAPI.

    Only implements the subset called by XArmRobotInterfaceNode. Keeps an
    integrator over set_servo_angle_j commands so joint_states show motion.
    """

    def __init__(self, n_dof: int = 6):
        self.n_dof = n_dof
        self._lock = threading.Lock()
        self._joint_positions = np.zeros(n_dof, dtype=np.float64)
        self._target_positions = np.zeros(n_dof, dtype=np.float64)
        self._joint_velocities = np.zeros(n_dof, dtype=np.float64)
        # SDK reports torques/speeds in fixed 7-slot arrays; mimic that.
        self._joints_torque = [0.0] * 7
        self._realtime_joint_speeds = [0.0] * 7

        self._mode = 0
        self._state = 0
        self._error_code = 0
        self._warn_code = 0
        self.connected = True
        self.axis = n_dof

    # -- mode / state --------------------------------------------------
    def motion_enable(self, enable: bool):
        return 0

    def set_mode(self, mode: int, detection_param: int = 0):
        self._mode = mode
        return 0

    def set_state(self, state: int = 0):
        self._state = state
        return 0

    def get_state(self):
        return 0, self._state

    def clean_error(self):
        self._error_code = 0
        return 0

    def clean_warn(self):
        self._warn_code = 0
        return 0

    def register_error_warn_changed_callback(self, cb):
        return 0

    def disconnect(self):
        self.connected = False

    # -- state queries -------------------------------------------------
    @property
    def mode(self):
        return self._mode

    @property
    def state(self):
        return self._state

    @property
    def has_error(self):
        return self._error_code != 0

    @property
    def has_warn(self):
        return self._warn_code != 0

    @property
    def error_code(self):
        return self._error_code

    @property
    def warn_code(self):
        return self._warn_code

    @property
    def joints_torque(self):
        return list(self._joints_torque)

    @property
    def realtime_joint_speeds(self):
        return list(self._realtime_joint_speeds)

    def get_servo_angle(self, is_radian: bool = True):
        with self._lock:
            angles = list(self._joint_positions)
        # SDK pads to 7 entries when axis<7; mimic that.
        if len(angles) < 7:
            angles = angles + [0.0] * (7 - len(angles))
        return 0, angles

    def get_position(self, is_radian: bool = True):
        # Crude but stable mock pose from joint angles.
        with self._lock:
            j = self._joint_positions.copy()
        x = 300.0 + 50.0 * j[0]
        y = 0.0 + 50.0 * (j[1] if len(j) > 1 else 0.0)
        z = 200.0 + 50.0 * (j[2] if len(j) > 2 else 0.0)
        roll = j[3] if len(j) > 3 else 0.0
        pitch = j[4] if len(j) > 4 else 0.0
        yaw = j[5] if len(j) > 5 else 0.0
        return 0, [x, y, z, roll, pitch, yaw]

    # -- forward kinematics -------------------------------------------
    def get_forward_kinematics(self, angles, input_is_radian=True, return_is_radian=True):
        n = self.n_dof
        ang = np.asarray(angles[:n], dtype=np.float64)
        # Save current state, compute mock pose, restore.
        with self._lock:
            saved = self._joint_positions.copy()
            self._joint_positions = ang
        code, pose = self.get_position(is_radian=True)
        with self._lock:
            self._joint_positions = saved
        return code, pose

    def get_inverse_kinematics(self, pose, input_is_radian=True, return_is_radian=True):
        # Mock returns current joints — enough for unit-test of the wiring.
        with self._lock:
            angles = list(self._joint_positions)
        if len(angles) < 7:
            angles = angles + [0.0] * (7 - len(angles))
        return 0, angles

    # -- command methods ----------------------------------------------
    def set_servo_angle(self, angle=None, speed=None, mvacc=None,
                        is_radian=True, wait=True, **kwargs):
        if angle is None:
            return 0
        ang = np.asarray(angle[:self.n_dof], dtype=np.float64)
        with self._lock:
            self._target_positions = ang
        # Simulate "blocking": just snap (mock has no timing model).
        if wait:
            self._step_to_target()
        return 0

    def set_servo_angle_j(self, angles, speed=None, mvacc=None, mvtime=0,
                          is_radian=True, **kwargs):
        ang = np.asarray(angles[:self.n_dof], dtype=np.float64)
        with self._lock:
            self._target_positions = ang
        return 0

    def set_position(self, x=None, y=None, z=None, roll=None, pitch=None,
                     yaw=None, speed=None, mvacc=None, is_radian=True,
                     wait=True, **kwargs):
        # Mock: no inverse kinematics; just record the target pose has been
        # accepted and snap joint state minimally so blocking returns.
        if wait:
            self._step_to_target()
        return 0

    def set_servo_cartesian(self, mvpose, speed=None, mvacc=None, mvtime=0,
                            is_radian=True, is_tool_coord=False, **kwargs):
        return 0

    def vc_set_joint_velocity(self, speeds, is_radian=True, is_sync=True,
                              duration=-1):
        spd = np.asarray(speeds[:self.n_dof], dtype=np.float64)
        with self._lock:
            self._joint_velocities = spd
            self._realtime_joint_speeds = list(spd) + [0.0] * (7 - len(spd))
            self._target_positions = self._joint_positions + spd * 0.1
        return 0

    def vc_set_cartesian_velocity(self, speeds, is_radian=True,
                                  is_tool_coord=False, duration=-1):
        return 0

    def move_gohome(self, speed=None, wait=True, **kwargs):
        with self._lock:
            self._target_positions = np.zeros(self.n_dof, dtype=np.float64)
        if wait:
            self._step_to_target()
        return 0

    def reset(self, speed=None, wait=True, **kwargs):
        return self.move_gohome(speed=speed, wait=wait)

    def emergency_stop(self):
        self._state = 4
        return 0

    # -- internal -----------------------------------------------------
    def _step_to_target(self):
        with self._lock:
            self._joint_positions = self._target_positions.copy()


# ============================================================================
# ROS 2 node
# ============================================================================
class XArmRobotInterfaceNode(Node):
    """ROS 2 driver for an xArm6 (or any xArm whose joint count comes from config)."""

    def __init__(self, use_mock: bool = False, ip_address: str = '192.168.1.185',
                 namespace: str = ''):
        super().__init__('xarm_robot_interface_node', namespace=namespace)

        # ---- Parameters ----
        self.declare_parameter('use_mock', use_mock)
        self.declare_parameter('ip_address', ip_address)
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('namespace', namespace)
        self.declare_parameter(
            'arm_joint_names', Parameter.Type.STRING_ARRAY,
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING_ARRAY)
        )
        self.declare_parameter('ee_frame_id', 'base_link')
        self.declare_parameter(
            'home_joints', Parameter.Type.DOUBLE_ARRAY,
            ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE_ARRAY)
        )
        self.declare_parameter('max_joint_velocity', 1.0)
        self.declare_parameter('auto_reset_on_startup', False)
        self.declare_parameter('auto_reset_delay', 5.0)

        use_mock = self.get_parameter('use_mock').get_parameter_value().bool_value
        ip_address = self.get_parameter('ip_address').get_parameter_value().string_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self._namespace = self.get_parameter('namespace').get_parameter_value().string_value
        self.arm_joint_names = list(
            self.get_parameter('arm_joint_names').get_parameter_value().string_array_value
        )
        self._ee_frame_id = self.get_parameter('ee_frame_id').get_parameter_value().string_value
        self._home_joints = list(
            self.get_parameter('home_joints').get_parameter_value().double_array_value
        )
        self._max_joint_velocity = self.get_parameter('max_joint_velocity').get_parameter_value().double_value
        auto_reset_on_startup = self.get_parameter('auto_reset_on_startup').get_parameter_value().bool_value
        auto_reset_delay = self.get_parameter('auto_reset_delay').get_parameter_value().double_value

        if not self.arm_joint_names:
            raise ValueError(
                "arm_joint_names is empty. Launch file must pass arm_joints from "
                "config/xarm_robot_config.yaml."
            )
        self._n_dof = len(self.arm_joint_names)
        if len(self._home_joints) != self._n_dof:
            self.get_logger().warn(
                f"home_joints length {len(self._home_joints)} != n_dof {self._n_dof}; "
                f"padding/truncating to {self._n_dof}."
            )
            if len(self._home_joints) < self._n_dof:
                self._home_joints = list(self._home_joints) + [0.0] * (self._n_dof - len(self._home_joints))
            else:
                self._home_joints = self._home_joints[:self._n_dof]

        self.get_logger().info(
            f"xArm interface — n_dof={self._n_dof}, joints={self.arm_joint_names}, "
            f"ee_frame_id={self._ee_frame_id}, ip={ip_address}, mock={use_mock}"
        )

        # ---- Robot interface ----
        self._arm = None
        try:
            if use_mock or not XARM_AVAILABLE:
                self.get_logger().info("Using MockXArmInterface (no hardware).")
                self._arm = MockXArmInterface(n_dof=self._n_dof)
            else:
                self.get_logger().info(f"Connecting to xArm at {ip_address} ...")
                self._arm = XArmAPI(ip_address, is_radian=True)
                self._arm.motion_enable(enable=True)
                self._arm.clean_error()
                self._arm.clean_warn()
                self._arm.set_mode(0)
                self._arm.set_state(0)
                self.get_logger().info(
                    f"xArm connected. axis={getattr(self._arm, 'axis', '?')}, "
                    f"version={getattr(self._arm, 'version', '?')}"
                )
        except Exception as e:
            self.get_logger().error(f"Failed to connect to xArm: {e}. Using mock.")
            self._arm = MockXArmInterface(n_dof=self._n_dof)

        # Track the current xArm-side mode so we don't thrash set_mode().
        self._current_xarm_mode = 0
        self._controller_mode = "none"
        self._is_running_policy = False
        self._current_kp: Optional[List[float]] = None
        self._current_kd: Optional[List[float]] = None
        self._command_lock = threading.Lock()
        self._mode_lock = threading.Lock()

        # ---- Error/warn callback ----
        try:
            if hasattr(self._arm, 'register_error_warn_changed_callback'):
                self._arm.register_error_warn_changed_callback(self._on_error_warn_changed)
        except Exception as e:
            self.get_logger().warn(f"Could not register error callback: {e}")

        # ---- Callback groups ----
        self._timer_callback_group = ReentrantCallbackGroup()
        self._service_callback_group = MutuallyExclusiveCallbackGroup()

        # ---- Publishers ----
        self._joint_state_publisher = self.create_publisher(JointState, 'joint_states', 10)
        self._arm_state_publisher = self.create_publisher(ArmState, 'arm_state', 10)
        self._ee_pose_publisher = self.create_publisher(PoseStamped, 'ee_pose', 10)
        self._controller_status_publisher = self.create_publisher(ControllerStatus, 'controller_status', 10)

        # ---- Subscribers ----
        self.create_subscription(
            JointPositionCommand, 'joint_position_controller/command',
            self._joint_position_command_callback, 10
        )
        self.create_subscription(
            JointVelocityCommand, 'joint_velocity_controller/command',
            self._joint_velocity_command_callback, 10
        )
        self.create_subscription(
            CartesianPositionCommand, 'cartesian_position_controller/command',
            self._cartesian_position_command_callback, 10
        )
        self.create_subscription(
            CartesianVelocityCommand, 'cartesian_velocity_controller/command',
            self._cartesian_velocity_command_callback, 10
        )

        # ---- Services ----
        self.create_service(Reset, 'reset', self._reset_service_callback,
                            callback_group=self._service_callback_group)
        self.create_service(MoveToJointPositions, 'move_to_joint_positions',
                            self._move_to_joint_positions_callback,
                            callback_group=self._service_callback_group)
        self.create_service(MoveToEEPose, 'move_to_ee_pose',
                            self._move_to_ee_pose_callback,
                            callback_group=self._service_callback_group)
        self.create_service(StartJointImpedance, 'start_joint_impedance',
                            self._start_joint_impedance_callback,
                            callback_group=self._service_callback_group)
        self.create_service(StartCartesianImpedance, 'start_cartesian_impedance',
                            self._start_cartesian_impedance_callback,
                            callback_group=self._service_callback_group)
        self.create_service(StartJointVelocity, 'start_joint_velocity',
                            self._start_joint_velocity_callback,
                            callback_group=self._service_callback_group)
        self.create_service(TerminatePolicy, 'terminate_policy',
                            self._terminate_policy_callback,
                            callback_group=self._service_callback_group)
        self.create_service(SolveIK, 'solve_ik', self._solve_ik_callback,
                            callback_group=self._service_callback_group)
        self.create_service(ComputeFK, 'compute_fk', self._compute_fk_callback,
                            callback_group=self._service_callback_group)
        self.create_service(ComputeTimeToGo, 'compute_time_to_go',
                            self._compute_time_to_go_callback,
                            callback_group=self._service_callback_group)

        # ---- State publish timer ----
        self._state_timer = self.create_timer(
            1.0 / self.publish_rate, self._publish_states,
            callback_group=self._timer_callback_group
        )

        if auto_reset_on_startup:
            self.get_logger().info(
                f"auto_reset_on_startup=true — reset will run in {auto_reset_delay:.1f}s"
            )
            threading.Timer(auto_reset_delay, lambda: self._reset_robot(False)).start()

        self.get_logger().info(
            f"XArmRobotInterfaceNode ready. Namespace: {self._namespace or '(default)'}, "
            f"publish_rate: {self.publish_rate} Hz"
        )

    # ------------------------------------------------------------------
    # Mode state machine
    # ------------------------------------------------------------------
    _MODE_TO_LABEL = {
        0: "position",
        1: "cartesian_impedance",   # also used for joint streaming
        4: "joint_velocity",
        5: "cartesian_velocity",
    }

    def _set_xarm_mode(self, target_mode: int, controller_label: Optional[str] = None):
        """Switch xArm controller mode if needed. Safe to call from multiple threads."""
        with self._mode_lock:
            if self._current_xarm_mode == target_mode:
                if controller_label:
                    self._controller_mode = controller_label
                return True
            try:
                # Pause any running motion before switching modes.
                self._arm.set_state(3)
                time.sleep(0.02)
                self._arm.set_mode(target_mode)
                self._arm.set_state(0)
                time.sleep(0.05)
                self._current_xarm_mode = target_mode
                self._controller_mode = controller_label or self._MODE_TO_LABEL.get(target_mode, "unknown")
                self._is_running_policy = (target_mode != 0)
                return True
            except Exception as e:
                self.get_logger().error(f"set_xarm_mode({target_mode}) failed: {e}")
                return False

    def _on_error_warn_changed(self, data):
        try:
            err = data.get('error_code', 0) if isinstance(data, dict) else getattr(data, 'error_code', 0)
            warn = data.get('warn_code', 0) if isinstance(data, dict) else getattr(data, 'warn_code', 0)
        except Exception:
            err, warn = 0, 0
        if err == 0 and warn == 0:
            return
        self.get_logger().warn(f"xArm reported error_code={err}, warn_code={warn} — recovering.")
        try:
            self._arm.clean_error()
            self._arm.clean_warn()
            self._arm.motion_enable(enable=True)
            self._arm.set_mode(0)
            self._arm.set_state(0)
            with self._mode_lock:
                self._current_xarm_mode = 0
                self._controller_mode = "error_recovered"
                self._is_running_policy = False
        except Exception as e:
            self.get_logger().error(f"Error recovery failed: {e}")

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def _truncate(self, arr, n=None) -> List[float]:
        n = n if n is not None else self._n_dof
        if arr is None:
            return [0.0] * n
        out = list(arr)[:n]
        if len(out) < n:
            out = out + [0.0] * (n - len(out))
        return [float(x) for x in out]

    def _get_joint_positions(self) -> np.ndarray:
        code, angles = self._arm.get_servo_angle(is_radian=True)
        return np.asarray(self._truncate(angles), dtype=np.float64)

    def _get_joint_velocities(self) -> np.ndarray:
        try:
            speeds = getattr(self._arm, 'realtime_joint_speeds', None)
            if speeds is None:
                return np.zeros(self._n_dof, dtype=np.float64)
            return np.asarray(self._truncate(list(speeds)), dtype=np.float64)
        except Exception:
            return np.zeros(self._n_dof, dtype=np.float64)

    def _get_joint_torques(self) -> np.ndarray:
        try:
            t = getattr(self._arm, 'joints_torque', None)
            if t is None:
                return np.zeros(self._n_dof, dtype=np.float64)
            return np.asarray(self._truncate(list(t)), dtype=np.float64)
        except Exception:
            return np.zeros(self._n_dof, dtype=np.float64)

    def _get_ee_pose(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Return (position[m], quat_xyzw, euler_xyz)."""
        code, pose = self._arm.get_position(is_radian=True)
        # pose = [x_mm, y_mm, z_mm, roll, pitch, yaw] (rad)
        pos = np.array([pose[0] / M_TO_MM, pose[1] / M_TO_MM, pose[2] / M_TO_MM])
        euler = np.array([pose[3], pose[4], pose[5]])
        if TRANSFORM_UTILS_AVAILABLE:
            quat = euler_to_quat(euler)
        else:
            quat = R.from_euler('xyz', euler).as_quat()
        return pos, np.asarray(quat, dtype=np.float64), euler

    @staticmethod
    def _quat_to_euler_xyz(quat_xyzw) -> np.ndarray:
        if TRANSFORM_UTILS_AVAILABLE:
            return np.asarray(quat_to_euler(quat_xyzw))
        return R.from_quat(quat_xyzw).as_euler('xyz')

    def _validate_len(self, arr, expected: int, name: str) -> bool:
        if len(arr) != expected:
            self.get_logger().error(f"{name} length {len(arr)} != expected {expected}")
            return False
        return True

    # ------------------------------------------------------------------
    # Publishers
    # ------------------------------------------------------------------
    def _publish_states(self):
        try:
            self._publish_joint_states()
            self._publish_arm_state()
            self._publish_ee_pose()
            self._publish_controller_status()
        except Exception as e:
            self.get_logger().error(f"_publish_states error: {e}\n{traceback.format_exc()}")

    def _publish_joint_states(self):
        positions = self._get_joint_positions()
        velocities = self._get_joint_velocities()
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._ee_frame_id
        msg.name = list(self.arm_joint_names)
        msg.position = positions.tolist()
        msg.velocity = velocities.tolist()
        msg.effort = [0.0] * self._n_dof
        self._joint_state_publisher.publish(msg)

    def _publish_arm_state(self):
        positions = self._get_joint_positions()
        velocities = self._get_joint_velocities()
        torques = self._get_joint_torques()
        pos, quat, euler = self._get_ee_pose()

        msg = ArmState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._ee_frame_id

        msg.joint_positions = positions.tolist()
        msg.joint_velocities = velocities.tolist()
        msg.joint_torques_computed = torques.tolist()
        msg.prev_joint_torques_computed = torques.tolist()
        msg.prev_joint_torques_computed_safened = torques.tolist()
        msg.motor_torques_measured = torques.tolist()

        msg.ee_position = pos.tolist()
        msg.ee_quaternion = quat.tolist()
        msg.ee_euler = euler.tolist()
        msg.ee_position_local = pos.tolist()
        msg.ee_quaternion_local = quat.tolist()
        msg.ee_euler_local = euler.tolist()

        msg.prev_controller_latency_ms = 0.0
        msg.prev_command_successful = True
        msg.is_running_policy = bool(self._is_running_policy)
        msg.controller_mode = self._controller_mode

        msg.kp = list(self._current_kp) if self._current_kp is not None else []
        msg.kd = list(self._current_kd) if self._current_kd is not None else []

        msg.polymetis_timestamp_ns = int(self.get_clock().now().nanoseconds)
        self._arm_state_publisher.publish(msg)

    def _publish_ee_pose(self):
        pos, quat, _ = self._get_ee_pose()
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._ee_frame_id
        msg.pose.position.x = float(pos[0])
        msg.pose.position.y = float(pos[1])
        msg.pose.position.z = float(pos[2])
        msg.pose.orientation.x = float(quat[0])
        msg.pose.orientation.y = float(quat[1])
        msg.pose.orientation.z = float(quat[2])
        msg.pose.orientation.w = float(quat[3])
        self._ee_pose_publisher.publish(msg)

    def _publish_controller_status(self):
        msg = ControllerStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._ee_frame_id
        msg.is_running_policy = bool(self._is_running_policy)
        msg.controller_mode = self._controller_mode
        msg.kp = list(self._current_kp) if self._current_kp is not None else []
        msg.kd = list(self._current_kd) if self._current_kd is not None else []
        self._controller_status_publisher.publish(msg)

    # ------------------------------------------------------------------
    # Subscribers
    # ------------------------------------------------------------------
    def _joint_position_command_callback(self, msg: JointPositionCommand):
        positions = list(msg.positions)
        if not self._validate_len(positions, self._n_dof, 'joint positions'):
            return

        if msg.blocking:
            def run():
                try:
                    self._set_xarm_mode(0, controller_label="position")
                    speed = self._compute_speed_from_time_to_go(positions, msg.time_to_go)
                    self._arm.set_servo_angle(angle=positions, speed=speed,
                                              is_radian=True, wait=True)
                    self._set_xarm_mode(1, controller_label="cartesian_impedance")
                except Exception as e:
                    self.get_logger().error(f"Blocking joint move error: {e}")
            threading.Thread(target=run, daemon=True).start()
        else:
            try:
                self._set_xarm_mode(1, controller_label="cartesian_impedance")
                self._arm.set_servo_angle_j(positions, is_radian=True)
            except Exception as e:
                self.get_logger().error(f"Streaming joint command error: {e}")

    def _joint_velocity_command_callback(self, msg: JointVelocityCommand):
        velocities = list(msg.velocities)
        if not self._validate_len(velocities, self._n_dof, 'joint velocities'):
            return
        try:
            self._set_xarm_mode(4, controller_label="joint_velocity")
            self._arm.vc_set_joint_velocity(velocities, is_radian=True,
                                            is_sync=True, duration=-1)
        except Exception as e:
            self.get_logger().error(f"Joint velocity command error: {e}")

    def _cartesian_position_command_callback(self, msg: CartesianPositionCommand):
        position = list(msg.position)
        orientation = list(msg.orientation)
        if not self._validate_len(position, 3, 'cartesian position'):
            return
        if not self._validate_len(orientation, 4, 'cartesian orientation'):
            return
        try:
            euler = self._quat_to_euler_xyz(orientation)
            mvpose = [
                position[0] * M_TO_MM,
                position[1] * M_TO_MM,
                position[2] * M_TO_MM,
                float(euler[0]), float(euler[1]), float(euler[2]),
            ]
        except Exception as e:
            self.get_logger().error(f"Pose conversion failed: {e}")
            return

        if msg.blocking:
            def run():
                try:
                    self._set_xarm_mode(0, controller_label="position")
                    self._arm.set_position(
                        x=mvpose[0], y=mvpose[1], z=mvpose[2],
                        roll=mvpose[3], pitch=mvpose[4], yaw=mvpose[5],
                        speed=DEFAULT_SERVO_CART_SPEED,
                        mvacc=DEFAULT_SERVO_CART_ACC,
                        is_radian=True, wait=True
                    )
                    self._set_xarm_mode(1, controller_label="cartesian_impedance")
                except Exception as e:
                    self.get_logger().error(f"Blocking cartesian move error: {e}")
            threading.Thread(target=run, daemon=True).start()
        else:
            try:
                self._set_xarm_mode(1, controller_label="cartesian_impedance")
                self._arm.set_servo_cartesian(
                    mvpose, speed=DEFAULT_SERVO_CART_SPEED,
                    mvacc=DEFAULT_SERVO_CART_ACC, is_radian=True
                )
            except Exception as e:
                self.get_logger().error(f"Streaming cartesian command error: {e}")

    def _cartesian_velocity_command_callback(self, msg: CartesianVelocityCommand):
        linear = list(msg.linear_velocity)
        angular = list(msg.angular_velocity)
        if not self._validate_len(linear, 3, 'linear velocity'):
            return
        if not self._validate_len(angular, 3, 'angular velocity'):
            return
        try:
            speeds = [
                linear[0] * M_TO_MM,    # m/s -> mm/s
                linear[1] * M_TO_MM,
                linear[2] * M_TO_MM,
                angular[0], angular[1], angular[2],   # rad/s
            ]
            self._set_xarm_mode(5, controller_label="cartesian_velocity")
            self._arm.vc_set_cartesian_velocity(speeds, is_radian=True,
                                                is_tool_coord=False, duration=-1)
        except Exception as e:
            self.get_logger().error(f"Cartesian velocity command error: {e}")

    # ------------------------------------------------------------------
    # Service callbacks
    # ------------------------------------------------------------------
    def _compute_speed_from_time_to_go(self, target_positions, time_to_go: float) -> float:
        curr = self._get_joint_positions()
        target = np.asarray(target_positions, dtype=np.float64)
        max_disp = float(np.max(np.abs(target - curr)))
        if time_to_go and time_to_go > 0:
            return max(max_disp / time_to_go, 0.05)
        # Default mid-speed if not specified.
        return min(self._max_joint_velocity, max(0.1, max_disp / 2.0))

    def _reset_robot(self, randomize: bool = False):
        try:
            self.get_logger().info(f"Resetting xArm to home (randomize={randomize})")
            target = list(self._home_joints)
            if randomize:
                target = (np.array(target) + np.random.uniform(-0.05, 0.05, size=self._n_dof)).tolist()
            self._set_xarm_mode(0, controller_label="position")
            speed = max(MAX_VELOCITY_SLOW, min(self._max_joint_velocity,
                       np.max(np.abs(np.array(target) - self._get_joint_positions())) / MIN_TIME_TO_GO_SLOW))
            self._arm.set_servo_angle(angle=target, speed=speed, is_radian=True, wait=True)
            self._set_xarm_mode(1, controller_label="cartesian_impedance")
            self.get_logger().info("xArm reset complete.")
        except Exception as e:
            self.get_logger().error(f"Reset error: {e}\n{traceback.format_exc()}")

    def _reset_service_callback(self, request, response):
        try:
            randomize = bool(getattr(request, 'randomize', False))
            threading.Thread(target=self._reset_robot, args=(randomize,), daemon=True).start()
            response.success = True
            response.message = "Reset accepted"
        except Exception as e:
            response.success = False
            response.message = f"Reset failed: {e}"
        return response

    def _move_to_joint_positions_callback(self, request, response):
        try:
            positions = list(request.joint_positions)
            if not self._validate_len(positions, self._n_dof, 'joint_positions'):
                response.success = False
                response.message = f"Invalid length {len(positions)}, expected {self._n_dof}"
                response.final_joint_positions = []
                return response

            self._set_xarm_mode(0, controller_label="position")
            speed = self._compute_speed_from_time_to_go(positions, request.time_to_go)
            code = self._arm.set_servo_angle(angle=positions, speed=speed,
                                             is_radian=True, wait=True)
            self._set_xarm_mode(1, controller_label="cartesian_impedance")

            final = self._get_joint_positions().tolist()
            response.success = (code == 0) if isinstance(code, int) else True
            response.message = (
                f"Moved to joint positions (speed={speed:.3f} rad/s)"
                if response.success else f"set_servo_angle returned code={code}"
            )
            response.final_joint_positions = final
        except Exception as e:
            response.success = False
            response.message = f"Failed: {e}"
            response.final_joint_positions = []
        return response

    def _move_to_ee_pose_callback(self, request, response):
        try:
            position = list(request.position)
            orientation = list(request.orientation)
            if not self._validate_len(position, 3, 'position') or \
               not self._validate_len(orientation, 4, 'orientation'):
                response.success = False
                response.message = "Invalid position or orientation length"
                response.final_joint_positions = []
                return response

            euler = self._quat_to_euler_xyz(orientation)
            self._set_xarm_mode(0, controller_label="position")
            speed = DEFAULT_SERVO_CART_SPEED
            if request.time_to_go and request.time_to_go > 0:
                speed = max(20.0, 1000.0 / request.time_to_go)  # heuristic mm/s
            code = self._arm.set_position(
                x=position[0] * M_TO_MM, y=position[1] * M_TO_MM, z=position[2] * M_TO_MM,
                roll=float(euler[0]), pitch=float(euler[1]), yaw=float(euler[2]),
                speed=speed, mvacc=DEFAULT_SERVO_CART_ACC,
                is_radian=True, wait=True
            )
            self._set_xarm_mode(1, controller_label="cartesian_impedance")

            response.success = (code == 0) if isinstance(code, int) else True
            response.message = "Moved to EE pose" if response.success else f"set_position code={code}"
            response.final_joint_positions = self._get_joint_positions().tolist()
        except Exception as e:
            response.success = False
            response.message = f"Failed: {e}"
            response.final_joint_positions = []
        return response

    def _start_joint_impedance_callback(self, request, response):
        # xArm has no native joint-impedance controller; map to servo streaming.
        try:
            self._current_kp = list(request.kq) if len(request.kq) > 0 else None
            self._current_kd = list(request.kqd) if len(request.kqd) > 0 else None
            ok = self._set_xarm_mode(1, controller_label="joint_impedance")
            response.success = ok
            response.message = (
                "Joint impedance (mapped to xArm servo mode 1) started"
                if ok else "Failed to switch to xArm mode 1"
            )
        except Exception as e:
            response.success = False
            response.message = f"Failed: {e}"
        return response

    def _start_cartesian_impedance_callback(self, request, response):
        try:
            self._current_kp = list(request.kx) if len(request.kx) > 0 else None
            self._current_kd = list(request.kxd) if len(request.kxd) > 0 else None
            ok = self._set_xarm_mode(1, controller_label="cartesian_impedance")
            response.success = ok
            response.message = (
                "Cartesian impedance (mapped to xArm servo mode 1) started"
                if ok else "Failed to switch to xArm mode 1"
            )
        except Exception as e:
            response.success = False
            response.message = f"Failed: {e}"
        return response

    def _start_joint_velocity_callback(self, request, response):
        try:
            self._current_kp = None
            self._current_kd = None
            ok = self._set_xarm_mode(4, controller_label="joint_velocity")
            if ok and len(request.qd_init) > 0:
                qd = self._truncate(list(request.qd_init))
                self._arm.vc_set_joint_velocity(qd, is_radian=True,
                                                is_sync=True, duration=-1)
            response.success = ok
            response.message = "Joint velocity started" if ok else "Failed to switch to mode 4"
        except Exception as e:
            response.success = False
            response.message = f"Failed: {e}"
        return response

    def _terminate_policy_callback(self, request, response):
        try:
            with self._mode_lock:
                try:
                    self._arm.set_state(3)  # pause
                except Exception:
                    pass
                try:
                    self._arm.set_mode(0)
                    self._arm.set_state(0)
                except Exception:
                    pass
                self._current_xarm_mode = 0
                self._controller_mode = "none"
                self._is_running_policy = False
            response.success = True
            response.message = "Policy terminated (xArm in mode 0, ready)"
        except Exception as e:
            response.success = False
            response.message = f"Failed: {e}"
        return response

    def _solve_ik_callback(self, request, response):
        try:
            position = list(request.position)
            orientation = list(request.orientation)
            if not self._validate_len(position, 3, 'position') or \
               not self._validate_len(orientation, 4, 'orientation'):
                response.success = False
                response.message = "Invalid position or orientation length"
                response.joint_positions = []
                response.error = 1.0
                return response

            euler = self._quat_to_euler_xyz(orientation)
            pose_xarm = [
                position[0] * M_TO_MM, position[1] * M_TO_MM, position[2] * M_TO_MM,
                float(euler[0]), float(euler[1]), float(euler[2]),
            ]
            code, angles = self._arm.get_inverse_kinematics(
                pose=pose_xarm, input_is_radian=True, return_is_radian=True
            )
            ok = (code == 0)
            response.success = ok
            response.message = "IK solved" if ok else f"IK failed (code={code})"
            response.joint_positions = self._truncate(angles) if angles else []
            response.error = 0.0 if ok else 1.0
        except Exception as e:
            response.success = False
            response.message = f"Failed: {e}"
            response.joint_positions = []
            response.error = 1.0
        return response

    def _compute_fk_callback(self, request, response):
        try:
            joints = list(request.joint_positions)
            if not self._validate_len(joints, self._n_dof, 'joint_positions'):
                response.success = False
                response.message = f"Invalid length {len(joints)}, expected {self._n_dof}"
                response.position = []
                response.orientation = []
                return response

            # SDK expects fixed-7 in some firmware revisions; pad to 7.
            angles = list(joints) + [0.0] * (7 - self._n_dof)
            code, pose = self._arm.get_forward_kinematics(
                angles=angles, input_is_radian=True, return_is_radian=True
            )
            ok = (code == 0)
            if not ok:
                response.success = False
                response.message = f"FK failed (code={code})"
                response.position = []
                response.orientation = []
                return response

            pos_m = [pose[0] / M_TO_MM, pose[1] / M_TO_MM, pose[2] / M_TO_MM]
            quat = R.from_euler('xyz', [pose[3], pose[4], pose[5]]).as_quat().tolist()
            response.success = True
            response.message = "FK computed"
            response.position = [float(p) for p in pos_m]
            response.orientation = [float(q) for q in quat]
        except Exception as e:
            response.success = False
            response.message = f"Failed: {e}"
            response.position = []
            response.orientation = []
        return response

    def _compute_time_to_go_callback(self, request, response):
        try:
            target = list(request.desired_joint_positions)
            if not self._validate_len(target, self._n_dof, 'desired_joint_positions'):
                response.success = False
                response.message = f"Invalid length {len(target)}, expected {self._n_dof}"
                response.time_to_go = 0.0
                return response
            curr = self._get_joint_positions()
            max_disp = float(np.max(np.abs(np.asarray(target) - curr)))
            ttg = min(MAX_TIME_TO_GO,
                      max(MIN_TIME_TO_GO, max_disp / MAX_VELOCITY_FOR_TIME_CALC))
            response.success = True
            response.message = "ok"
            response.time_to_go = ttg
        except Exception as e:
            response.success = False
            response.message = f"Failed: {e}"
            response.time_to_go = 0.0
        return response

    # ------------------------------------------------------------------
    def destroy_node(self):
        try:
            if self._arm is not None and not isinstance(self._arm, MockXArmInterface):
                try:
                    self._arm.set_state(3)
                except Exception:
                    pass
                try:
                    self._arm.disconnect()
                except Exception:
                    pass
        finally:
            super().destroy_node()


def main(args=None):
    from rclpy.executors import MultiThreadedExecutor

    rclpy.init(args=args)
    namespace = os.environ.get('ROBOT_NAMESPACE', 'xarm6_arm')

    node = XArmRobotInterfaceNode(
        use_mock=not XARM_AVAILABLE,
        namespace=namespace,
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
