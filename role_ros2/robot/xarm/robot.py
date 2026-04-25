"""
XArmRobot — ROS2 client interface for the UFACTORY xArm6 driver node.

Mirrors the public surface of `role_ros2.robot.franka.robot.FrankaRobot` so the
same downstream code (RobotEnv, trajectory collection scripts) can drive an
xArm by swapping the class and the namespace.

Key differences from FrankaRobot:
- 6 DOF (xArm6) instead of 7 DOF.
- No gripper integration in the launch — gripper methods are stubs that no-op.
- IK / FK delegated to the xArm controller box via `solve_ik` / `compute_fk`
  ROS services (the xArm SDK's on-board IK). There is no local URDF-based IK
  solver, so cartesian-action conversions go through the network.
- `cartesian_velocity_to_joint_velocity` is not available without a local IK
  solver; `create_action_dict` therefore omits joint cross-conversions for
  cartesian action spaces and only fills the natively-known fields.

Topic / service contract (must match nodes/xarm_robot_interface_node.py):
- Subscribes:  /robot_state                          (root, from aggregator)
- Publishes:   /{arm_namespace}/joint_position_controller/command
               /{arm_namespace}/joint_velocity_controller/command
               /{arm_namespace}/cartesian_position_controller/command
               /{arm_namespace}/cartesian_velocity_controller/command
- Service clients (all under /{arm_namespace}):
               reset, move_to_joint_positions, move_to_ee_pose,
               start_joint_impedance, start_cartesian_impedance,
               start_joint_velocity, terminate_policy,
               solve_ik, compute_fk, compute_time_to_go
"""

import time
import threading
from collections import deque
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from role_ros2.msg import (
    ArmState, RobotState,
    JointPositionCommand, JointVelocityCommand,
    CartesianPositionCommand, CartesianVelocityCommand,
)
from role_ros2.srv import (
    Reset, StartCartesianImpedance, StartJointImpedance, StartJointVelocity,
    TerminatePolicy, MoveToJointPositions, MoveToEEPose,
    SolveIK, ComputeFK, ComputeTimeToGo,
)

from role_ros2.misc.transformations import euler_to_quat, quat_to_euler, pose_diff, add_poses
from role_ros2.misc.ros2_utils import get_ros_time_ns
from role_ros2.robot.base_robot import BaseRobot


class XArmRobot(BaseRobot):
    """ROS2 client for the xArm robot driver node.

    Args:
        node: Optional shared rclpy.Node. If None, creates an internal node and
              spins it on a background thread.
        arm_namespace: Namespace where the xArm interface node publishes.
        n_dof: Number of arm DOF (6 for xArm6, 7 for xArm7). Defaults to 6.
    """

    # No gripper in this implementation; keep the dim layout convention as
    # FrankaRobot (last entry would be gripper) so downstream scripts that
    # always pass an extra slot still work — that slot is silently ignored.
    DOF_CARTESIAN = 7   # 6 (pose) + 1 (gripper slot, ignored)
    DOF_JOINT = 7       # 6 (joints) + 1 (gripper slot, ignored)

    def __init__(
        self,
        node: Optional[Node] = None,
        arm_namespace: str = "xarm6_arm",
        n_dof: int = 6,
    ):
        self._arm_namespace = arm_namespace
        self._n_dof = n_dof

        if node is None:
            if not rclpy.ok():
                rclpy.init()
            self._node = Node('xarm_robot_interface')
            self._own_node = True
        else:
            self._node = node
            self._own_node = False

        # Default home pose for xArm6: all joints at 0 (matches xacro default).
        self.reset_joints = np.zeros(self._n_dof, dtype=np.float64)
        # Cartesian-noise bounds (m, rad) for `randomize=True` resets.
        self.randomize_low = np.array([-0.05, -0.10, -0.05, -0.2, -0.2, -0.2])
        self.randomize_high = np.array([0.05, 0.10, 0.05, 0.2, 0.2, 0.2])

        self._cb_group = ReentrantCallbackGroup()

        # ---- State storage ----
        self._state_lock = threading.Lock()
        self._robot_state_snapshot: Optional[Tuple[RobotState, int]] = None

        self._state_cache: deque = deque(maxlen=100)
        self._state_cache_lock = threading.Lock()
        self._cache_timestamps_ns: np.ndarray = np.zeros(100, dtype=np.int64)

        # ---- Subscribers ----
        self._robot_state_sub = self._node.create_subscription(
            RobotState, '/robot_state', self._robot_state_callback, 10,
            callback_group=self._cb_group,
        )

        # ---- Publishers ----
        self._joint_pos_cmd_pub = self._node.create_publisher(
            JointPositionCommand,
            f'/{arm_namespace}/joint_position_controller/command', 10,
        )
        self._joint_vel_cmd_pub = self._node.create_publisher(
            JointVelocityCommand,
            f'/{arm_namespace}/joint_velocity_controller/command', 10,
        )
        self._cartesian_pos_cmd_pub = self._node.create_publisher(
            CartesianPositionCommand,
            f'/{arm_namespace}/cartesian_position_controller/command', 10,
        )
        self._cartesian_vel_cmd_pub = self._node.create_publisher(
            CartesianVelocityCommand,
            f'/{arm_namespace}/cartesian_velocity_controller/command', 10,
        )

        # ---- Service clients ----
        ns = arm_namespace
        self._reset_client = self._node.create_client(Reset, f'/{ns}/reset')
        self._start_cart_imp_client = self._node.create_client(
            StartCartesianImpedance, f'/{ns}/start_cartesian_impedance')
        self._start_joint_imp_client = self._node.create_client(
            StartJointImpedance, f'/{ns}/start_joint_impedance')
        self._start_joint_vel_client = self._node.create_client(
            StartJointVelocity, f'/{ns}/start_joint_velocity')
        self._terminate_policy_client = self._node.create_client(
            TerminatePolicy, f'/{ns}/terminate_policy')
        self._move_to_joints_client = self._node.create_client(
            MoveToJointPositions, f'/{ns}/move_to_joint_positions')
        self._move_to_ee_pose_client = self._node.create_client(
            MoveToEEPose, f'/{ns}/move_to_ee_pose')
        self._solve_ik_client = self._node.create_client(
            SolveIK, f'/{ns}/solve_ik')
        self._compute_fk_client = self._node.create_client(
            ComputeFK, f'/{ns}/compute_fk')
        self._compute_time_to_go_client = self._node.create_client(
            ComputeTimeToGo, f'/{ns}/compute_time_to_go')

        self._node.get_logger().info(
            f"Waiting for xArm services under /{arm_namespace} ...")
        if not self._reset_client.wait_for_service(timeout_sec=5.0):
            self._node.get_logger().warn(
                f"Reset service not available at /{arm_namespace}/reset")

        self._executor = None
        self._spin_thread = None
        if self._own_node:
            from rclpy.executors import MultiThreadedExecutor
            self._executor = MultiThreadedExecutor()
            self._executor.add_node(self._node)
            self._spin_thread = threading.Thread(
                target=self._spin_executor, daemon=True)
            self._spin_thread.start()
            self._wait_for_state(timeout=5.0)
        else:
            self._node.get_logger().debug(
                "XArmRobot: using shared node — caller must spin and "
                "invoke wait_for_ready() before calling state getters.")

        self._node.get_logger().info(
            f"XArmRobot initialized (namespace=/{arm_namespace}, n_dof={n_dof})")

    # ==================================================================
    # State callback / cache
    # ==================================================================
    def _robot_state_callback(self, msg: RobotState):
        received_time_ns = get_ros_time_ns(self._node)
        pub_stamp = msg.header.stamp
        pub_timestamp_ns = int(pub_stamp.sec * 1_000_000_000 + pub_stamp.nanosec)

        with self._state_lock:
            self._robot_state_snapshot = (msg, received_time_ns)

        with self._state_cache_lock:
            self._state_cache.append((pub_timestamp_ns, msg, received_time_ns))
            for i, (ts, _, _) in enumerate(self._state_cache):
                if i < len(self._cache_timestamps_ns):
                    self._cache_timestamps_ns[i] = ts

    def _wait_for_state(self, timeout: float = 5.0):
        start = time.time()
        while time.time() - start < timeout:
            with self._state_lock:
                if self._robot_state_snapshot is not None:
                    return
            time.sleep(0.1)
        self._node.get_logger().warn(
            "XArmRobot: timeout waiting for initial /robot_state — "
            "is robot_state_aggregator running?")

    def wait_for_ready(self, timeout: float = 5.0):
        self._wait_for_state(timeout=timeout)

    # ==================================================================
    # State getters
    # ==================================================================
    def _get_robot_state_msg(self) -> Optional[RobotState]:
        with self._state_lock:
            snap = self._robot_state_snapshot
        return snap[0] if snap is not None else None

    def _get_arm_state(self) -> Optional[ArmState]:
        msg = self._get_robot_state_msg()
        if msg is None or not msg.arm_states:
            return None
        # Single-robot case: pick the entry matching our namespace if available.
        if msg.robot_namespaces:
            try:
                idx = list(msg.robot_namespaces).index(self._arm_namespace)
                if idx < len(msg.arm_states):
                    return msg.arm_states[idx]
            except ValueError:
                pass
        return msg.arm_states[0]

    def get_joint_positions(self) -> List[float]:
        s = self._get_arm_state()
        return list(s.joint_positions) if s is not None else [0.0] * self._n_dof

    def get_joint_velocities(self) -> List[float]:
        s = self._get_arm_state()
        return list(s.joint_velocities) if s is not None else [0.0] * self._n_dof

    def get_ee_pose(self) -> List[float]:
        s = self._get_arm_state()
        if s is None:
            return [0.0] * 6
        return list(s.ee_position) + list(s.ee_euler)

    def get_ee_position(self) -> List[float]:
        s = self._get_arm_state()
        return list(s.ee_position) if s is not None else [0.0] * 3

    def get_ee_quaternion(self) -> List[float]:
        s = self._get_arm_state()
        return list(s.ee_quaternion) if s is not None else [0.0, 0.0, 0.0, 1.0]

    # ---------------- Gripper stubs (no gripper integrated) ----------------
    def get_gripper_position(self) -> float:
        return 0.0

    def get_gripper_width(self) -> float:
        return 0.0

    def get_gripper_state(self) -> float:
        return 0.0

    def is_gripper_grasped(self) -> bool:
        return False

    def update_gripper(self, command, velocity=True, blocking=False):
        # No-op: xArm launch has no gripper node attached.
        return None

    def gripper_open(self) -> bool:
        return True

    def gripper_close(self) -> bool:
        return True

    # ==================================================================
    # Combined robot state
    # ==================================================================
    _DEFAULT_STATE_KEYS = (
        "cartesian_position", "cartesian_position_local", "gripper_position",
        "joint_positions", "joint_velocities", "joint_torques_computed",
        "prev_joint_torques_computed", "prev_joint_torques_computed_safened",
        "motor_torques_measured",
        "prev_controller_latency_ms", "prev_command_successful",
    )

    def _default_state(self) -> Tuple[Dict[str, Any], Dict[str, Any]]:
        zeros_n = [0.0] * self._n_dof
        state = {
            "cartesian_position": [0.0] * 6,
            "cartesian_position_local": [0.0] * 6,
            "gripper_position": 0.0,
            "joint_positions": list(zeros_n),
            "joint_velocities": list(zeros_n),
            "joint_torques_computed": list(zeros_n),
            "prev_joint_torques_computed": list(zeros_n),
            "prev_joint_torques_computed_safened": list(zeros_n),
            "motor_torques_measured": list(zeros_n),
            "prev_controller_latency_ms": 0.0,
            "prev_command_successful": False,
        }
        timestamps = {
            "robot_polymetis_t": 0,
            "robot_pub_t": 0,
            "robot_sub_t": 0,
            "robot_end_t": 0,
        }
        return state, timestamps

    def get_robot_state(self) -> Tuple[Dict[str, Any], Dict[str, Any]]:
        with self._state_lock:
            snap = self._robot_state_snapshot
        if snap is None:
            return self._default_state()

        msg, received_ns = snap
        state_dict = self._build_state_dict_from_msg(msg)

        pub_stamp = msg.header.stamp
        pub_ns = int(pub_stamp.sec * 1_000_000_000 + pub_stamp.nanosec)
        arm_state = self._get_arm_state()
        polymetis_ns = int(arm_state.polymetis_timestamp_ns) if arm_state else 0

        timestamp_dict = {
            "robot_polymetis_t": polymetis_ns,
            "robot_pub_t": int(pub_ns),
            "robot_sub_t": int(received_ns),
            "robot_end_t": int(get_ros_time_ns(self._node)),
        }
        return state_dict, timestamp_dict

    def _build_state_dict_from_msg(self, msg: RobotState) -> Dict[str, Any]:
        # Pick our arm
        arm_state = None
        if msg.robot_namespaces:
            try:
                idx = list(msg.robot_namespaces).index(self._arm_namespace)
                if idx < len(msg.arm_states):
                    arm_state = msg.arm_states[idx]
            except ValueError:
                pass
        if arm_state is None and msg.arm_states:
            arm_state = msg.arm_states[0]

        if arm_state is None:
            return self._default_state()[0]

        if arm_state.ee_position_local:
            cart_local = list(arm_state.ee_position_local) + list(arm_state.ee_euler_local)
        else:
            cart_local = list(arm_state.ee_position) + list(arm_state.ee_euler)
        cart = list(arm_state.ee_position) + list(arm_state.ee_euler)

        return {
            "cartesian_position": cart,
            "cartesian_position_local": cart_local,
            "gripper_position": 0.0,
            "joint_positions": list(arm_state.joint_positions),
            "joint_velocities": list(arm_state.joint_velocities),
            "joint_torques_computed": list(arm_state.joint_torques_computed),
            "prev_joint_torques_computed": list(arm_state.prev_joint_torques_computed),
            "prev_joint_torques_computed_safened":
                list(arm_state.prev_joint_torques_computed_safened),
            "motor_torques_measured": list(arm_state.motor_torques_measured),
            "prev_controller_latency_ms": float(arm_state.prev_controller_latency_ms),
            "prev_command_successful": bool(arm_state.prev_command_successful),
        }

    def get_arm_state_raw(self) -> Optional[ArmState]:
        return self._get_arm_state()

    # ==================================================================
    # Reset / home
    # ==================================================================
    def reset(self, randomize: bool = False, wait_for_completion: bool = True,
              wait_time_sec: float = 30.0, open_gripper: bool = False):
        """Move arm to home (xArm SDK runs on the controller, blocking)."""
        target = self.reset_joints.copy()
        if randomize:
            try:
                noise = np.random.uniform(low=self.randomize_low,
                                          high=self.randomize_high)
                target = self.add_noise_to_joints(target, noise)
            except Exception as e:
                self._node.get_logger().warn(
                    f"randomize failed ({e}); using nominal home_joints")

        current = np.array(self.get_joint_positions())
        max_disp = float(np.max(np.abs(np.array(target) - current)))
        # Mirror franka's slow-reset clamp.
        MAX_VELOCITY_SLOW = 0.1
        MIN_TIME_TO_GO_SLOW = 5.0
        MAX_TIME_TO_GO_SLOW = 40.0
        if max_disp > 0:
            ttg = min(MAX_TIME_TO_GO_SLOW,
                      max(MIN_TIME_TO_GO_SLOW, max_disp / MAX_VELOCITY_SLOW))
        else:
            ttg = MIN_TIME_TO_GO_SLOW

        self._node.get_logger().info(
            f"XArmRobot.reset: target={target.tolist()}, "
            f"max_disp={max_disp:.3f} rad, time_to_go={ttg:.2f}s, "
            f"blocking={wait_for_completion}")
        return self.update_joints(
            target, velocity=False, blocking=wait_for_completion, time_to_go=ttg)

    def home(self):
        self.reset()

    # ==================================================================
    # Arm control
    # ==================================================================
    def update_command(self, command, action_space="joint_position",
                       gripper_action_space=None, blocking=False):
        """High-level entry point, mirroring FrankaRobot.update_command.

        For xArm we ignore the gripper slot (no gripper) but accept its
        presence in `command` to stay drop-in compatible with the franka API.
        """
        action_dict = self.create_action_dict(
            command, action_space=action_space,
            gripper_action_space=gripper_action_space)
        self.update_joints(action_dict["joint_position"],
                           velocity=False, blocking=blocking)
        return action_dict

    def update_joints(self, command, velocity: bool = False,
                      blocking: bool = False, cartesian_noise=None,
                      time_to_go: Optional[float] = None):
        if cartesian_noise is not None:
            self._node.get_logger().warn(
                "XArmRobot: cartesian_noise is not supported; ignoring")
        if blocking and not velocity:
            return self._update_joints_blocking(command, time_to_go=time_to_go)
        return self._update_joints_non_blocking(command, velocity)

    def _update_joints_blocking(self, command, time_to_go=None) -> bool:
        if not self._move_to_joints_client.wait_for_service(timeout_sec=1.0):
            self._node.get_logger().error(
                "MoveToJointPositions service not available")
            return False
        req = MoveToJointPositions.Request()
        req.joint_positions = [float(x) for x in list(command)[:self._n_dof]]
        req.time_to_go = (
            float(time_to_go) if time_to_go is not None and time_to_go > 0 else 0.0
        )
        return self._call_service(self._move_to_joints_client, req,
                                  timeout_sec=30.0, name="move_to_joint_positions")

    def _update_joints_non_blocking(self, command, velocity: bool):
        if velocity:
            msg = JointVelocityCommand()
            msg.header.stamp = self._node.get_clock().now().to_msg()
            msg.velocities = [float(x) for x in list(command)[:self._n_dof]]
            self._joint_vel_cmd_pub.publish(msg)
        else:
            msg = JointPositionCommand()
            msg.header.stamp = self._node.get_clock().now().to_msg()
            msg.positions = [float(x) for x in list(command)[:self._n_dof]]
            msg.blocking = False
            self._joint_pos_cmd_pub.publish(msg)
        return True

    def update_pose(self, command, velocity: bool = False, blocking: bool = False):
        if blocking and not velocity:
            return self._update_pose_blocking(command)
        return self._update_pose_non_blocking(command, velocity)

    def _update_pose_blocking(self, command) -> bool:
        if not self._move_to_ee_pose_client.wait_for_service(timeout_sec=1.0):
            self._node.get_logger().error("MoveToEEPose service not available")
            return False
        pos = [float(x) for x in list(command[:3])]
        if len(command) >= 7:
            quat = [float(x) for x in list(command[3:7])]
        else:
            quat = [float(x) for x in euler_to_quat(command[3:6]).tolist()]

        req = MoveToEEPose.Request()
        req.position = pos
        req.orientation = quat
        req.time_to_go = 0.0
        arm = self._get_arm_state()
        if arm is not None:
            req.q0 = [float(x) for x in list(arm.joint_positions)]
        else:
            req.q0 = [0.0] * self._n_dof
        return self._call_service(self._move_to_ee_pose_client, req,
                                  timeout_sec=30.0, name="move_to_ee_pose")

    def _update_pose_non_blocking(self, command, velocity: bool):
        if velocity:
            msg = CartesianVelocityCommand()
            msg.header.stamp = self._node.get_clock().now().to_msg()
            msg.linear_velocity = [float(x) for x in list(command[:3])]
            msg.angular_velocity = (
                [float(x) for x in list(command[3:6])]
                if len(command) >= 6 else [0.0, 0.0, 0.0]
            )
            self._cartesian_vel_cmd_pub.publish(msg)
        else:
            pos = [float(x) for x in list(command[:3])]
            if len(command) >= 7:
                quat = [float(x) for x in list(command[3:7])]
            else:
                quat = [float(x) for x in euler_to_quat(command[3:6]).tolist()]
            msg = CartesianPositionCommand()
            msg.header.stamp = self._node.get_clock().now().to_msg()
            msg.position = pos
            msg.orientation = quat
            msg.blocking = False
            self._cartesian_pos_cmd_pub.publish(msg)
        return True

    # ==================================================================
    # Controller mode helpers
    # ==================================================================
    def start_cartesian_impedance(self, kx=None, kxd=None) -> bool:
        if not self._start_cart_imp_client.wait_for_service(timeout_sec=1.0):
            self._node.get_logger().warn(
                "StartCartesianImpedance service not available")
            return False
        req = StartCartesianImpedance.Request()
        req.kx = list(kx) if kx is not None else []
        req.kxd = list(kxd) if kxd is not None else []
        return self._call_service(self._start_cart_imp_client, req,
                                  timeout_sec=5.0, name="start_cartesian_impedance")

    def start_joint_impedance(self, kq=None, kqd=None) -> bool:
        if not self._start_joint_imp_client.wait_for_service(timeout_sec=1.0):
            self._node.get_logger().warn(
                "StartJointImpedance service not available")
            return False
        req = StartJointImpedance.Request()
        req.kq = list(kq) if kq is not None else []
        req.kqd = list(kqd) if kqd is not None else []
        return self._call_service(self._start_joint_imp_client, req,
                                  timeout_sec=5.0, name="start_joint_impedance")

    def start_joint_velocity(self, qd_init=None) -> bool:
        if not self._start_joint_vel_client.wait_for_service(timeout_sec=1.0):
            self._node.get_logger().warn(
                "StartJointVelocity service not available")
            return False
        req = StartJointVelocity.Request()
        req.qd_init = list(qd_init) if qd_init is not None else []
        return self._call_service(self._start_joint_vel_client, req,
                                  timeout_sec=5.0, name="start_joint_velocity")

    def terminate_current_policy(self) -> bool:
        if not self._terminate_policy_client.wait_for_service(timeout_sec=1.0):
            self._node.get_logger().warn("TerminatePolicy service not available")
            return False
        req = TerminatePolicy.Request()
        return self._call_service(self._terminate_policy_client, req,
                                  timeout_sec=5.0, name="terminate_policy")

    # ==================================================================
    # IK / FK (delegated to controller)
    # ==================================================================
    def solve_inverse_kinematics(self, position, orientation, q0=None,
                                 tolerance: float = 1e-3):
        if not self._solve_ik_client.wait_for_service(timeout_sec=1.0):
            self._node.get_logger().warn("SolveIK service not available")
            return None, False
        req = SolveIK.Request()
        req.position = [float(x) for x in list(position)[:3]]
        if len(orientation) == 3:
            quat = [float(x) for x in euler_to_quat(orientation).tolist()]
        else:
            quat = [float(x) for x in list(orientation)[:4]]
        req.orientation = quat
        if q0 is None:
            arm = self._get_arm_state()
            req.q0 = (
                [float(x) for x in list(arm.joint_positions)]
                if arm is not None else [0.0] * self._n_dof
            )
        else:
            req.q0 = [float(x) for x in list(q0)]
        req.tolerance = float(tolerance)

        future = self._solve_ik_client.call_async(req)
        if not self._wait_future(future, timeout_sec=5.0):
            self._node.get_logger().error("solve_ik service call timed out")
            return None, False
        response = future.result()
        if response.success:
            return list(response.joint_positions), True
        self._node.get_logger().warn(f"IK not found: {response.message}")
        return list(response.joint_positions), False

    def compute_forward_kinematics(self, joint_positions):
        if not self._compute_fk_client.wait_for_service(timeout_sec=1.0):
            self._node.get_logger().warn("ComputeFK service not available")
            return None, None
        req = ComputeFK.Request()
        req.joint_positions = [float(x) for x in list(joint_positions)[:self._n_dof]]
        future = self._compute_fk_client.call_async(req)
        if not self._wait_future(future, timeout_sec=5.0):
            self._node.get_logger().error("compute_fk service call timed out")
            return None, None
        response = future.result()
        if response.success:
            return list(response.position), list(response.orientation)
        self._node.get_logger().error(f"FK failed: {response.message}")
        return None, None

    def add_noise_to_joints(self, original_joints, cartesian_noise):
        """Add cartesian noise to joints via FK + IK (controller round-trip)."""
        pos, quat = self.compute_forward_kinematics(original_joints)
        if pos is None or quat is None:
            self._node.get_logger().warn(
                "add_noise_to_joints: FK failed, returning original joints")
            return original_joints
        curr_euler = quat_to_euler(quat)
        curr_pose = list(pos) + list(curr_euler)
        new_pose = add_poses(cartesian_noise, curr_pose)
        new_joints, ok = self.solve_inverse_kinematics(
            new_pose[:3], new_pose[3:6], q0=original_joints)
        if ok and new_joints is not None:
            return np.array(new_joints, dtype=np.float64)
        self._node.get_logger().warn(
            "add_noise_to_joints: IK failed, returning original joints")
        return original_joints

    # ==================================================================
    # Action dict
    # ==================================================================
    def create_action_dict(self, action, action_space="joint_position",
                           gripper_action_space=None, robot_state=None):
        """Convert raw action array → labeled action dict.

        Layout convention (drop-in compatible with FrankaRobot):
            joint_*  : action[:n_dof] are joints, action[-1] is gripper (ignored)
            cartesian_*: action[:6] is pose/twist, action[-1] is gripper (ignored)

        Cross-conversions (cartesian → joint) require a network IK call to the
        controller, so we keep them lazy: only `cartesian_position` blocking
        path uses solve_ik when needed for actuation.
        """
        assert action_space in (
            "cartesian_position", "joint_position",
            "cartesian_velocity", "joint_velocity",
        )

        if robot_state is None:
            robot_state = self.get_robot_state()[0]

        action = np.asarray(action, dtype=np.float64)
        action_dict: Dict[str, Any] = {"robot_state": robot_state}

        # Gripper slot: the xArm has no gripper here, but downstream code may
        # always include a final value. Record it for trajectory logging then
        # silently ignore on actuation.
        action_dict["gripper_position"] = float(np.clip(action[-1], 0.0, 1.0))
        action_dict["gripper_velocity"] = 0.0
        action_dict["gripper_delta"] = 0.0

        if "joint" in action_space:
            arm_part = action[:self._n_dof]
            if "velocity" in action_space:
                action_dict["joint_velocity"] = arm_part.tolist()
                # Coarse Euler integration for an immediate target estimate.
                dt = 0.05
                joint_pos = (np.array(robot_state["joint_positions"]) + arm_part * dt)
                action_dict["joint_position"] = joint_pos.tolist()
            else:
                action_dict["joint_position"] = arm_part.tolist()
                joint_delta = arm_part - np.array(robot_state["joint_positions"])
                action_dict["joint_velocity"] = (joint_delta / 0.05).tolist()
        else:  # cartesian_*
            arm_part = action[:6]
            if "velocity" in action_space:
                action_dict["cartesian_velocity"] = arm_part.tolist()
                # Integrate over a fixed dt to obtain a target pose.
                dt = 0.05
                next_pose = add_poses(arm_part * dt,
                                      np.array(robot_state["cartesian_position_local"]))
                action_dict["cartesian_position"] = next_pose.tolist()
            else:
                action_dict["cartesian_position"] = arm_part.tolist()
                delta = pose_diff(arm_part,
                                  np.array(robot_state["cartesian_position_local"]))
                action_dict["cartesian_velocity"] = (delta / 0.05).tolist()

            # Cross-convert to joint_position via controller IK so update_command
            # can drive update_joints. This is a network round-trip per call.
            target_pos = action_dict["cartesian_position"][:3]
            target_euler = action_dict["cartesian_position"][3:6]
            ik_joints, ok = self.solve_inverse_kinematics(
                target_pos, target_euler,
                q0=robot_state["joint_positions"])
            if ok and ik_joints is not None:
                action_dict["joint_position"] = list(ik_joints)[:self._n_dof]
            else:
                action_dict["joint_position"] = list(robot_state["joint_positions"])
            action_dict["joint_velocity"] = [0.0] * self._n_dof

        return action_dict

    # ==================================================================
    # Utilities
    # ==================================================================
    def _wait_future(self, future, timeout_sec: float) -> bool:
        start = time.time()
        while not future.done() and (time.time() - start) < timeout_sec:
            time.sleep(0.001)
        return future.done()

    def _call_service(self, client, request, timeout_sec: float, name: str) -> bool:
        future = client.call_async(request)
        if not self._wait_future(future, timeout_sec=timeout_sec):
            self._node.get_logger().error(f"{name} service call timed out")
            return False
        response = future.result()
        if getattr(response, 'success', False):
            return True
        msg = getattr(response, 'message', '')
        self._node.get_logger().error(f"{name} failed: {msg}")
        return False

    def _spin_executor(self):
        try:
            self._executor.spin()
        except Exception as e:
            if rclpy.ok():
                self._node.get_logger().error(
                    f"XArmRobot spin thread error: {e}")

    def shutdown(self):
        if not self._own_node:
            return
        self._node.get_logger().info("XArmRobot: shutting down ...")
        if self._executor is not None:
            try:
                self._executor.shutdown(timeout_sec=2.0)
            except Exception as e:
                self._node.get_logger().warn(
                    f"executor shutdown error: {e}")
        if self._spin_thread is not None and self._spin_thread.is_alive():
            try:
                self._spin_thread.join(timeout=3.0)
            except Exception as e:
                self._node.get_logger().warn(f"spin thread join error: {e}")
        try:
            self._node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass

    @property
    def arm_namespace(self) -> str:
        return self._arm_namespace

    @property
    def n_dof(self) -> int:
        return self._n_dof
