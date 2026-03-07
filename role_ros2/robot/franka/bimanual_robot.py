"""
BimanualFrankaRobot - ROS2 Interface for Bimanual Franka Robot.

This class provides a Python interface for controlling two Franka arms (left and right)
with separate grippers. It subscribes to /robot_state (from aggregator) and uses
ee_position_local / ee_euler_local for IK (arm base frame) while ee_position / ee_euler
provide the world (base_link) frame pose for visualization.

Usage:
    robot = BimanualFrankaRobot(node=node)
    state_dict, timestamp_dict = robot.get_robot_state()
    action_dict = robot.create_action_dict(action, action_space="cartesian_velocity")
    robot.update_command(action, action_space="cartesian_velocity")
"""

import time
import threading
from typing import Optional, Tuple, Dict, Any

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from role_ros2.msg import (
    ArmState,
    GripperState,
    RobotState,
    JointPositionCommand,
    JointVelocityCommand,
    GripperCommand,
)
from role_ros2.srv import MoveToJointPositions
from std_srvs.srv import Trigger

from role_ros2.misc.transformations import add_poses, pose_diff
from role_ros2.misc.ros2_utils import get_ros_time_ns
from role_ros2.robot_ik.robot_ik_solver import RobotIKSolver
from role_ros2.robot.base_robot import BaseRobot


def _build_arm_state_dict(arm_state: ArmState) -> Dict[str, Any]:
    """
    Build arm state dict from ArmState message.

    cartesian_position_local: ee_position_local + ee_euler_local (arm base frame, for IK)
    cartesian_position: ee_position + ee_euler (base_link/world frame, for observation)
    """
    cartesian_position_local = list(arm_state.ee_position_local) + list(arm_state.ee_euler_local)
    cartesian_position = list(arm_state.ee_position) + list(arm_state.ee_euler)
    return {
        "cartesian_position_local": cartesian_position_local,
        "cartesian_position": cartesian_position,
        "joint_positions": list(arm_state.joint_positions),
        "joint_velocities": list(arm_state.joint_velocities),
        "joint_torques_computed": list(arm_state.joint_torques_computed),
        "prev_joint_torques_computed": list(arm_state.prev_joint_torques_computed),
        "prev_joint_torques_computed_safened": list(arm_state.prev_joint_torques_computed_safened),
        "motor_torques_measured": list(arm_state.motor_torques_measured),
        "prev_controller_latency_ms": arm_state.prev_controller_latency_ms,
        "prev_command_successful": arm_state.prev_command_successful,
    }


class BimanualFrankaRobot(BaseRobot):
    """
    ROS2-based robot interface for Bimanual Franka Robot.

    Subscribes to /robot_state (aggregated) which contains:
    - arm_states[i].ee_position / ee_euler: in base_link (world) frame
    - arm_states[i].ee_position_local / ee_euler_local: in arm's own base frame (for IK)
    - gripper_states[i]: gripper position, width, etc.
    """

    DOF_CARTESIAN = 16  # 6+6+2 (left arm, right arm, left gripper, right gripper)
    DOF_JOINT = 18     # 7+7+2

    _DEFAULT_ARM_STATE = {
        "cartesian_position_local": [0.0] * 6,
        "cartesian_position": [0.0] * 6,
        "joint_positions": [0.0] * 7,
        "joint_velocities": [0.0] * 7,
        "joint_torques_computed": [0.0] * 7,
        "prev_joint_torques_computed": [0.0] * 7,
        "prev_joint_torques_computed_safened": [0.0] * 7,
        "motor_torques_measured": [0.0] * 7,
        "prev_controller_latency_ms": 0.0,
        "prev_command_successful": False,
    }

    def __init__(
        self,
        node: Optional[Node] = None,
        arm_namespaces: Tuple[str, str] = ("left_arm", "right_arm"),
        gripper_namespaces: Tuple[str, str] = ("left_gripper", "right_gripper"),
    ):
        """
        Initialize BimanualFrankaRobot.

        Args:
            node: Optional ROS2 node. If None, creates a new node.
            arm_namespaces: (left_arm_ns, right_arm_ns)
            gripper_namespaces: (left_gripper_ns, right_gripper_ns)
        """
        self._arm_namespaces = arm_namespaces
        self._gripper_namespaces = gripper_namespaces
        left_arm_ns, right_arm_ns = arm_namespaces
        left_gripper_ns, right_gripper_ns = gripper_namespaces

        if node is None:
            if not rclpy.ok():
                rclpy.init()
            self._node = Node("bimanual_franka_robot")
            self._own_node = True
        else:
            self._node = node
            self._own_node = False

        self._ik_solver = RobotIKSolver()
        self.reset_joints = np.array([0, -1 / 5 * np.pi, 0, -4 / 5 * np.pi, 0, 3 / 5 * np.pi, 0.0])
        self._max_gripper_width = 0.08

        self._cb_group = ReentrantCallbackGroup()
        self._state_lock = threading.Lock()

        self._robot_state_snapshot: Optional[Tuple[RobotState, int]] = None

        # Subscribe to aggregated /robot_state only
        self._node.create_subscription(
            RobotState,
            "/robot_state",
            self._robot_state_callback,
            10,
            callback_group=self._cb_group,
        )

        # Publishers for both arms and grippers
        self._joint_pos_pubs = {
            "left": self._node.create_publisher(
                JointPositionCommand,
                f"/{left_arm_ns}/joint_position_controller/command",
                10,
            ),
            "right": self._node.create_publisher(
                JointPositionCommand,
                f"/{right_arm_ns}/joint_position_controller/command",
                10,
            ),
        }
        self._joint_vel_pubs = {
            "left": self._node.create_publisher(
                JointVelocityCommand,
                f"/{left_arm_ns}/joint_velocity_controller/command",
                10,
            ),
            "right": self._node.create_publisher(
                JointVelocityCommand,
                f"/{right_arm_ns}/joint_velocity_controller/command",
                10,
            ),
        }
        self._gripper_pubs = {
            "left": self._node.create_publisher(
                GripperCommand,
                f"/{left_gripper_ns}/gripper/command",
                10,
            ),
            "right": self._node.create_publisher(
                GripperCommand,
                f"/{right_gripper_ns}/gripper/command",
                10,
            ),
        }

        # Service clients for reset
        self._move_to_joint_clients = {
            "left": self._node.create_client(
                MoveToJointPositions, f"/{left_arm_ns}/move_to_joint_positions"
            ),
            "right": self._node.create_client(
                MoveToJointPositions, f"/{right_arm_ns}/move_to_joint_positions"
            ),
        }
        self._gripper_open_clients = {
            "left": self._node.create_client(Trigger, f"/{left_gripper_ns}/gripper/open"),
            "right": self._node.create_client(Trigger, f"/{right_gripper_ns}/gripper/open"),
        }

        self._executor = None
        self._spin_thread = None
        if self._own_node:
            from rclpy.executors import MultiThreadedExecutor
            self._executor = MultiThreadedExecutor()
            self._executor.add_node(self._node)
            self._spin_thread = threading.Thread(target=self._spin_executor, daemon=True)
            self._spin_thread.start()

        self._wait_for_state(timeout=5.0)
        self._node.get_logger().info(
            f"BimanualFrankaRobot initialized "
            f"(arms: {arm_namespaces}, grippers: {gripper_namespaces})"
        )

    def _robot_state_callback(self, msg: RobotState):
        received_time_ns = get_ros_time_ns(self._node)
        with self._state_lock:
            self._robot_state_snapshot = (msg, received_time_ns)

    def _spin_executor(self):
        try:
            self._executor.spin()
        except Exception as e:
            if rclpy.ok():
                self._node.get_logger().error(f"Error in BimanualFrankaRobot spin: {e}")

    def _wait_for_state(self, timeout: float = 5.0):
        start = time.time()
        while time.time() - start < timeout:
            with self._state_lock:
                if self._robot_state_snapshot is not None:
                    return
            time.sleep(0.1)
        self._node.get_logger().warn("Timeout waiting for initial bimanual robot state")

    def _find_arm_gripper(self, msg: RobotState) -> Tuple:
        """
        Find left/right arm and gripper states from RobotState by namespace matching.

        The aggregator builds arm_states[] in order for namespaces containing 'arm',
        and gripper_states[] in order for namespaces containing 'gripper'.

        Returns:
            (left_arm, right_arm, left_gripper, right_gripper) - each ArmState/GripperState or None
        """
        left_arm_ns, right_arm_ns = self._arm_namespaces
        left_grip_ns, right_grip_ns = self._gripper_namespaces

        arm_ns_list = [ns for ns in msg.robot_namespaces if "arm" in ns.lower()]
        grip_ns_list = [ns for ns in msg.robot_namespaces if "gripper" in ns.lower()]

        left_arm = right_arm = left_gripper = right_gripper = None
        for i, ns in enumerate(arm_ns_list):
            if i >= len(msg.arm_states):
                break
            if ns == left_arm_ns:
                left_arm = msg.arm_states[i]
            elif ns == right_arm_ns:
                right_arm = msg.arm_states[i]

        for i, ns in enumerate(grip_ns_list):
            if i >= len(msg.gripper_states):
                break
            if ns == left_grip_ns:
                left_gripper = msg.gripper_states[i]
            elif ns == right_grip_ns:
                right_gripper = msg.gripper_states[i]

        return left_arm, right_arm, left_gripper, right_gripper

    def get_robot_state(self) -> Tuple[Dict[str, Any], Dict[str, Any]]:
        """
        Get comprehensive bimanual robot state.

        Returns:
            tuple: (state_dict, timestamp_dict)
            state_dict has left_arm, right_arm (each with cartesian_position in arm base
            frame for IK, and cartesian_position in base_link for observation),
            left_gripper_position, right_gripper_position.
        """
        with self._state_lock:
            snapshot = self._robot_state_snapshot

        if snapshot is None:
            state_dict = {
                "left_arm": self._DEFAULT_ARM_STATE.copy(),
                "right_arm": self._DEFAULT_ARM_STATE.copy(),
                "left_gripper_position": 0.0,
                "right_gripper_position": 0.0,
            }
            timestamp_dict = {
                "robot_polymetis_t": 0,
                "robot_pub_t": 0,
                "robot_sub_t": get_ros_time_ns(self._node),
                "robot_end_t": get_ros_time_ns(self._node),
            }
            return state_dict, timestamp_dict

        msg, received_time_ns = snapshot
        left_arm, right_arm, left_gripper, right_gripper = self._find_arm_gripper(msg)

        left_arm_dict = (
            _build_arm_state_dict(left_arm)
            if left_arm is not None
            else self._DEFAULT_ARM_STATE.copy()
        )
        right_arm_dict = (
            _build_arm_state_dict(right_arm)
            if right_arm is not None
            else self._DEFAULT_ARM_STATE.copy()
        )
        left_gripper_pos = float(left_gripper.position) if left_gripper else 0.0
        right_gripper_pos = float(right_gripper.position) if right_gripper else 0.0

        state_dict = {
            "left_arm": left_arm_dict,
            "right_arm": right_arm_dict,
            "left_gripper_position": left_gripper_pos,
            "right_gripper_position": right_gripper_pos,
        }

        pub_stamp = msg.header.stamp
        pub_t_ns = int(pub_stamp.sec * 1_000_000_000 + pub_stamp.nanosec)
        poly_t_ns = int(left_arm.polymetis_timestamp_ns) if left_arm is not None else 0

        timestamp_dict = {
            "robot_polymetis_t": poly_t_ns,
            "robot_pub_t": pub_t_ns,
            "robot_sub_t": int(received_time_ns),
            "robot_end_t": get_ros_time_ns(self._node),
        }
        return state_dict, timestamp_dict

    def get_robot_state_for_timestamp(
        self,
        timestamp_ns: int,
        max_time_diff_ns: int = 200_000_000,
    ) -> Tuple[Dict[str, Any], Dict[str, Any], int]:
        """
        Get robot state closest to timestamp. For bimanual, returns latest state
        (timestamp-based cache can be added later if needed).
        """
        state_dict, timestamp_dict = self.get_robot_state()
        return state_dict, timestamp_dict, 0

    def create_action_dict(
        self,
        action,
        action_space: str = "cartesian_velocity",
        gripper_action_space: Optional[str] = None,
        robot_state: Optional[Dict] = None,
    ) -> Dict[str, Any]:
        """
        Create action dictionary from action array.

        Action format:
        - cartesian_velocity: [left_lin(3), left_rot(3), right_lin(3), right_rot(3), left_grip, right_grip] = 16D
        - joint_velocity: [left_joint(7), right_joint(7), left_grip, right_grip] = 18D
        - cartesian_position / joint_position: same structure, 16D / 18D

        Uses robot_state["left_arm"]["cartesian_position"] (arm base frame) for IK.
        """
        assert action_space in [
            "cartesian_position",
            "joint_position",
            "cartesian_velocity",
            "joint_velocity",
        ]
        if robot_state is None:
            robot_state = self.get_robot_state()[0]

        velocity = "velocity" in action_space
        if gripper_action_space is None:
            gripper_action_space = "velocity" if velocity else "position"
        assert gripper_action_space in ["velocity", "position"]

        action_dict: Dict[str, Any] = {"robot_state": robot_state}
        left_state = robot_state["left_arm"]
        right_state = robot_state["right_arm"]
        left_grip = robot_state["left_gripper_position"]
        right_grip = robot_state["right_gripper_position"]

        # Gripper processing
        if gripper_action_space == "velocity":
            left_vel, right_vel = action[-2], action[-1]
            left_delta = self._ik_solver.gripper_velocity_to_delta(left_vel)
            right_delta = self._ik_solver.gripper_velocity_to_delta(right_vel)
            action_dict["left_gripper_position"] = float(np.clip(left_grip + left_delta, 0, 1))
            action_dict["right_gripper_position"] = float(np.clip(right_grip + right_delta, 0, 1))
        else:
            action_dict["left_gripper_position"] = float(np.clip(action[-2], 0, 1))
            action_dict["right_gripper_position"] = float(np.clip(action[-1], 0, 1))

        if "cartesian" in action_space:
            left_arm_part = np.array(action[:6])
            right_arm_part = np.array(action[6:12])
            if velocity:
                left_delta = self._ik_solver.cartesian_velocity_to_delta(left_arm_part)
                right_delta = self._ik_solver.cartesian_velocity_to_delta(right_arm_part)
                action_dict["left_cartesian_position"] = add_poses(
                    left_delta, left_state["cartesian_position_local"]
                ).tolist()
                action_dict["right_cartesian_position"] = add_poses(
                    right_delta, right_state["cartesian_position_local"]
                ).tolist()
                left_cart_vel = left_arm_part
                right_cart_vel = right_arm_part
            else:
                action_dict["left_cartesian_position"] = list(left_arm_part)
                action_dict["right_cartesian_position"] = list(right_arm_part)
                left_cart_vel = self._ik_solver.cartesian_delta_to_velocity(
                    pose_diff(left_arm_part, left_state["cartesian_position_local"])
                )
                right_cart_vel = self._ik_solver.cartesian_delta_to_velocity(
                    pose_diff(right_arm_part, right_state["cartesian_position_local"])
                )

            left_joint_vel = self._ik_solver.cartesian_velocity_to_joint_velocity(
                left_cart_vel.tolist() if hasattr(left_cart_vel, "tolist") else list(left_cart_vel),
                robot_state=left_state,
            )
            right_joint_vel = self._ik_solver.cartesian_velocity_to_joint_velocity(
                right_cart_vel.tolist() if hasattr(right_cart_vel, "tolist") else list(right_cart_vel),
                robot_state=right_state,
            )
            left_joint_delta = self._ik_solver.joint_velocity_to_delta(left_joint_vel)
            right_joint_delta = self._ik_solver.joint_velocity_to_delta(right_joint_vel)
            action_dict["left_joint_position"] = (
                left_joint_delta + np.array(left_state["joint_positions"])
            ).tolist()
            action_dict["right_joint_position"] = (
                right_joint_delta + np.array(right_state["joint_positions"])
            ).tolist()
            action_dict["left_joint_velocity"] = left_joint_vel.tolist()
            action_dict["right_joint_velocity"] = right_joint_vel.tolist()

        if "joint" in action_space:
            left_joint_part = action[:7]
            right_joint_part = action[7:14]
            if velocity:
                left_delta = self._ik_solver.joint_velocity_to_delta(left_joint_part)
                right_delta = self._ik_solver.joint_velocity_to_delta(right_joint_part)
                action_dict["left_joint_position"] = (
                    left_delta + np.array(left_state["joint_positions"])
                ).tolist()
                action_dict["right_joint_position"] = (
                    right_delta + np.array(right_state["joint_positions"])
                ).tolist()
            else:
                action_dict["left_joint_position"] = list(left_joint_part)
                action_dict["right_joint_position"] = list(right_joint_part)
            left_delta = np.array(action_dict["left_joint_position"]) - np.array(
                left_state["joint_positions"]
            )
            right_delta = np.array(action_dict["right_joint_position"]) - np.array(
                right_state["joint_positions"]
            )
            action_dict["left_joint_velocity"] = self._ik_solver.joint_delta_to_velocity(
                left_delta
            ).tolist()
            action_dict["right_joint_velocity"] = self._ik_solver.joint_delta_to_velocity(
                right_delta
            ).tolist()

        return action_dict

    def update_command(
        self,
        command,
        action_space: str = "cartesian_velocity",
        gripper_action_space: Optional[str] = None,
        blocking: bool = False,
    ) -> Dict[str, Any]:
        """
        Update robot command for both arms and grippers.

        Args:
            command: 16D (cartesian) or 18D (joint) action array
            action_space: "cartesian_position", "joint_position", "cartesian_velocity", "joint_velocity"
            gripper_action_space: "position" or "velocity"
            blocking: Whether to wait for completion (not fully supported for bimanual)
        """
        action_dict = self.create_action_dict(
            command,
            action_space=action_space,
            gripper_action_space=gripper_action_space,
        )

        left_joints = action_dict["left_joint_position"]
        right_joints = action_dict["right_joint_position"]
        left_grip = action_dict["left_gripper_position"]
        right_grip = action_dict["right_gripper_position"]

        velocity = "velocity" in action_space
        self._publish_joints("left", left_joints, velocity)
        self._publish_joints("right", right_joints, velocity)
        self._publish_gripper("left", left_grip)
        self._publish_gripper("right", right_grip)

        return action_dict

    def _publish_joints(self, side: str, command, velocity: bool):
        if velocity:
            msg = JointVelocityCommand()
            msg.header.stamp = self._node.get_clock().now().to_msg()
            msg.velocities = list(command)
            self._joint_vel_pubs[side].publish(msg)
        else:
            msg = JointPositionCommand()
            msg.header.stamp = self._node.get_clock().now().to_msg()
            msg.positions = list(command)
            msg.blocking = False
            self._joint_pos_pubs[side].publish(msg)

    def _publish_gripper(self, side: str, position: float):
        width = self._max_gripper_width * (1.0 - position)
        msg = GripperCommand()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.width = width
        msg.blocking = False
        self._gripper_pubs[side].publish(msg)

    def reset(
        self,
        randomize: bool = False,
        wait_for_completion: bool = True,
        wait_time_sec: float = 20.0,
        open_gripper: bool = True,
    ):
        """
        Reset both arms and grippers to home position.

        Args:
            randomize: If True, add random noise (not implemented for bimanual)
            wait_for_completion: Whether to wait for reset to complete
            wait_time_sec: Max time to wait
            open_gripper: If True, open both grippers first
        """
        if randomize:
            self._node.get_logger().warn("BimanualFrankaRobot: randomize not implemented")

        if open_gripper:
            for side in ("left", "right"):
                client = self._gripper_open_clients[side]
                if client.wait_for_service(timeout_sec=1.0):
                    future = client.call_async(Trigger.Request())
                    start = time.time()
                    while not future.done() and (time.time() - start) < 5.0:
                        time.sleep(0.001)
                else:
                    self._node.get_logger().warn(f"Gripper open service not available for {side}")

        target_joints = self.reset_joints.tolist()
        time_to_go = 5.0 if wait_for_completion else 0.0

        for side in ("left", "right"):
            client = self._move_to_joint_clients[side]
            if not client.wait_for_service(timeout_sec=1.0):
                self._node.get_logger().warn(f"MoveToJointPositions not available for {side}")
                continue
            request = MoveToJointPositions.Request()
            request.joint_positions = target_joints
            request.time_to_go = time_to_go
            future = client.call_async(request)
            start = time.time()
            while wait_for_completion and not future.done() and (time.time() - start) < wait_time_sec:
                time.sleep(0.001)
            if future.done():
                try:
                    resp = future.result()
                    if not resp.success:
                        self._node.get_logger().warn(f"Reset failed for {side}: {resp.message}")
                except Exception as e:
                    self._node.get_logger().warn(f"Reset exception for {side}: {e}")

    def shutdown(self):
        """Clean up resources."""
        if not self._own_node:
            return
        if self._executor:
            try:
                self._executor.shutdown(timeout_sec=2.0)
            except Exception:
                pass
        if self._spin_thread and self._spin_thread.is_alive():
            try:
                self._spin_thread.join(timeout=3.0)
            except Exception:
                pass
        if self._node:
            try:
                self._node.destroy_node()
            except Exception:
                pass

    @property
    def dof(self) -> int:
        """Degrees of freedom for cartesian_velocity (16) or joint_velocity (18)."""
        return self.DOF_CARTESIAN
