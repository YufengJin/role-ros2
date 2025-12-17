#!/usr/bin/env python3
"""
Teleoperation Node (Oculus -> Polymetis)

Usage:
  # 1) Start robot + TF (Polymetis manager + robot_state_publisher)
  ros2 launch role_ros2 franka_robot.launch.py

  # 2) Start Oculus reader (publishes /oculus/* topics)
  ros2 run role_ros2 oculus_reader_node --ros-args -p publish_rate:=50.0

  # 3) Start teleop (publishes PolymetisRobotCommand to polymetis_manager)
  ros2 run role_ros2 teleoperation_node --ros-args -p arm_id:=fr3 -p right_controller:=true

Controls (default):
  - Hold controller GRIP to enable motion ("deadman").
  - Trigger controls gripper open/close (normalized 0..1 mapped to velocity).
  - Press joystick (RJ/LJ) to reset VR orientation mapping.
"""

import numpy as np
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

from role_ros2.msg import OculusButtons, PolymetisRobotCommand, PolymetisRobotState
from role_ros2.misc.transformations import (
    add_angles,
    euler_to_quat,
    quat_diff,
    quat_to_euler,
)

from scipy.spatial.transform import Rotation as R


def _vec_to_reorder_mat(vec):
    """Same as droid.controllers.oculus_controller.vec_to_reorder_mat"""
    X = np.zeros((len(vec), len(vec)))
    for i in range(X.shape[0]):
        ind = int(abs(vec[i])) - 1
        X[i, ind] = np.sign(vec[i])
    return X


def _pose_to_mat(pose_msg: PoseStamped) -> np.ndarray:
    """PoseStamped -> 4x4 transform matrix."""
    q = pose_msg.pose.orientation
    t = pose_msg.pose.position
    rot = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    T = np.eye(4)
    T[:3, :3] = rot
    T[:3, 3] = [t.x, t.y, t.z]
    return T


class TeleoperationNode(Node):
    """
    Subscribes:
      - oculus right/left controller pose (PoseStamped)
      - oculus buttons (OculusButtons)
      - polymetis_manager robot_state (PolymetisRobotState)

    Publishes:
      - polymetis_manager/robot_command (PolymetisRobotCommand) with action_space=cartesian_velocity
    """

    def __init__(self):
        super().__init__("teleoperation_node")

        # Parameters
        self.declare_parameter("arm_id", "fr3")
        self.declare_parameter("right_controller", True)

        self.declare_parameter("control_rate", 15.0)
        self.declare_parameter("max_lin_vel", 1.0)
        self.declare_parameter("max_rot_vel", 1.0)
        self.declare_parameter("max_gripper_vel", 1.0)
        self.declare_parameter("spatial_coeff", 1.0)
        self.declare_parameter("pos_action_gain", 5.0)
        self.declare_parameter("rot_action_gain", 2.0)
        self.declare_parameter("gripper_action_gain", 3.0)
        # Coordinate reorder (same default as droid VRPolicy)
        self.declare_parameter("rmat_reorder", [-2, -1, -3, 4])

        self.declare_parameter("oculus_right_pose_topic", "oculus/right_controller/pose")
        self.declare_parameter("oculus_left_pose_topic", "oculus/left_controller/pose")
        self.declare_parameter("oculus_buttons_topic", "oculus/buttons")
        self.declare_parameter("robot_state_topic", "polymetis_manager/robot_state")
        self.declare_parameter("robot_command_topic", "polymetis_manager/robot_command")

        self.arm_id = self.get_parameter("arm_id").value
        self.right_controller = bool(self.get_parameter("right_controller").value)
        self.controller_side = "right" if self.right_controller else "left"

        self.control_rate = float(self.get_parameter("control_rate").value)
        self.max_lin_vel = float(self.get_parameter("max_lin_vel").value)
        self.max_rot_vel = float(self.get_parameter("max_rot_vel").value)
        self.max_gripper_vel = float(self.get_parameter("max_gripper_vel").value)
        self.spatial_coeff = float(self.get_parameter("spatial_coeff").value)
        self.pos_action_gain = float(self.get_parameter("pos_action_gain").value)
        self.rot_action_gain = float(self.get_parameter("rot_action_gain").value)
        self.gripper_action_gain = float(self.get_parameter("gripper_action_gain").value)
        self.rmat_reorder = list(self.get_parameter("rmat_reorder").value)

        self.oculus_right_pose_topic = str(self.get_parameter("oculus_right_pose_topic").value)
        self.oculus_left_pose_topic = str(self.get_parameter("oculus_left_pose_topic").value)
        self.oculus_buttons_topic = str(self.get_parameter("oculus_buttons_topic").value)
        self.robot_state_topic = str(self.get_parameter("robot_state_topic").value)
        self.robot_command_topic = str(self.get_parameter("robot_command_topic").value)

        # Internal state
        self._last_pose_msg: PoseStamped | None = None
        self._last_buttons_msg: OculusButtons | None = None
        self._last_robot_state_msg: PolymetisRobotState | None = None

        # VRPolicy-like state
        self._movement_enabled = False
        self._update_sensor = True
        self._reset_origin = True
        self._reset_orientation = True
        self._vr_to_global_mat = np.eye(4)
        self._global_to_env_mat = _vec_to_reorder_mat(self.rmat_reorder)

        self._robot_origin = None  # {"pos": np.ndarray(3), "quat": np.ndarray(4)}
        self._vr_origin = None     # {"pos": np.ndarray(3), "quat": np.ndarray(4)}
        self._vr_state = None      # {"pos": np.ndarray(3), "quat": np.ndarray(4), "gripper": float}

        # Pub/Sub
        pose_topic = self.oculus_right_pose_topic if self.right_controller else self.oculus_left_pose_topic
        self._pose_sub = self.create_subscription(PoseStamped, pose_topic, self._pose_cb, 10)
        self._buttons_sub = self.create_subscription(OculusButtons, self.oculus_buttons_topic, self._buttons_cb, 10)
        self._robot_state_sub = self.create_subscription(PolymetisRobotState, self.robot_state_topic, self._robot_state_cb, 10)

        self._cmd_pub = self.create_publisher(PolymetisRobotCommand, self.robot_command_topic, 10)

        self._timer = self.create_timer(1.0 / max(self.control_rate, 1e-6), self._tick)

        self.get_logger().info(
            f"TeleoperationNode ready. controller={self.controller_side}, "
            f"pose_topic={pose_topic}, buttons_topic={self.oculus_buttons_topic}, "
            f"robot_state_topic={self.robot_state_topic}, cmd_topic={self.robot_command_topic}"
        )

    def _pose_cb(self, msg: PoseStamped):
        self._last_pose_msg = msg

    def _buttons_cb(self, msg: OculusButtons):
        self._last_buttons_msg = msg

    def _robot_state_cb(self, msg: PolymetisRobotState):
        self._last_robot_state_msg = msg

    def _get_deadman_and_reset_flags(self, buttons: OculusButtons):
        if self.right_controller:
            grip_pressed = bool(buttons.right_grip_pressed)
            joystick_pressed = bool(buttons.right_joystick_pressed)
            trigger_value = float(buttons.right_trigger_value)
        else:
            grip_pressed = bool(buttons.left_grip_pressed)
            joystick_pressed = bool(buttons.left_joystick_pressed)
            trigger_value = float(buttons.left_trigger_value)
        return grip_pressed, joystick_pressed, trigger_value

    def _process_reading(self):
        """Update self._vr_state from latest pose/buttons."""
        assert self._last_pose_msg is not None
        assert self._last_buttons_msg is not None

        rot_mat = _pose_to_mat(self._last_pose_msg)

        grip_pressed, joystick_pressed, trigger_value = self._get_deadman_and_reset_flags(self._last_buttons_msg)

        toggled = (self._movement_enabled != grip_pressed)
        self._update_sensor = self._update_sensor or grip_pressed
        self._reset_orientation = self._reset_orientation or joystick_pressed
        self._reset_origin = self._reset_origin or toggled
        self._movement_enabled = grip_pressed

        stop_updating = joystick_pressed or self._movement_enabled
        if self._reset_orientation:
            if stop_updating:
                self._reset_orientation = False
            try:
                self._vr_to_global_mat = np.linalg.inv(rot_mat)
            except Exception:
                self._vr_to_global_mat = np.eye(4)
                self._reset_orientation = True

        rot_mat = self._global_to_env_mat @ self._vr_to_global_mat @ rot_mat
        vr_pos = self.spatial_coeff * rot_mat[:3, 3]
        vr_quat = R.from_matrix(rot_mat[:3, :3]).as_quat()  # [x, y, z, w]
        vr_gripper = float(np.clip(trigger_value, 0.0, 1.0))

        self._vr_state = {"pos": vr_pos, "quat": vr_quat, "gripper": vr_gripper}

    def _limit_velocity(self, lin_vel, rot_vel, gripper_vel):
        lin_norm = float(np.linalg.norm(lin_vel))
        rot_norm = float(np.linalg.norm(rot_vel))
        grip_norm = float(np.linalg.norm([gripper_vel]))

        if lin_norm > self.max_lin_vel and lin_norm > 1e-9:
            lin_vel = lin_vel * (self.max_lin_vel / lin_norm)
        if rot_norm > self.max_rot_vel and rot_norm > 1e-9:
            rot_vel = rot_vel * (self.max_rot_vel / rot_norm)
        if grip_norm > self.max_gripper_vel and grip_norm > 1e-9:
            gripper_vel = gripper_vel * (self.max_gripper_vel / grip_norm)
        return lin_vel, rot_vel, gripper_vel

    def _compute_action(self) -> np.ndarray:
        """
        Return action in [-1, 1]:
          [vx, vy, vz, wx, wy, wz, gripper_vel]
        """
        if self._last_pose_msg is None or self._last_buttons_msg is None or self._last_robot_state_msg is None:
            return np.zeros(7, dtype=np.float64)

        # Update VR state if needed
        if self._update_sensor:
            self._process_reading()
            self._update_sensor = False

        if not self._movement_enabled or self._vr_state is None:
            return np.zeros(7, dtype=np.float64)

        # Read robot state (from PolymetisRobotState)
        rs = self._last_robot_state_msg
        robot_pos = np.array(rs.ee_position[:3], dtype=np.float64)
        robot_euler = np.array(rs.ee_euler[:3], dtype=np.float64)
        robot_quat = euler_to_quat(robot_euler).astype(np.float64)
        robot_gripper = float(rs.gripper_position)

        # Reset origin on grip toggle release/press
        if self._reset_origin:
            self._robot_origin = {"pos": robot_pos.copy(), "quat": robot_quat.copy()}
            self._vr_origin = {"pos": self._vr_state["pos"].copy(), "quat": self._vr_state["quat"].copy()}
            self._reset_origin = False

        # Pos action
        robot_pos_offset = robot_pos - self._robot_origin["pos"]
        target_pos_offset = self._vr_state["pos"] - self._vr_origin["pos"]
        pos_action = target_pos_offset - robot_pos_offset

        # Rot action (quaternion diff -> euler)
        robot_quat_offset = quat_diff(robot_quat, self._robot_origin["quat"])
        target_quat_offset = quat_diff(self._vr_state["quat"], self._vr_origin["quat"])
        quat_action = quat_diff(target_quat_offset, robot_quat_offset)
        euler_action = quat_to_euler(quat_action)

        # Gripper action (velocity-like)
        gripper_action = (self._vr_state["gripper"] * 1.5) - robot_gripper

        # Scale
        pos_action *= self.pos_action_gain
        euler_action *= self.rot_action_gain
        gripper_action *= self.gripper_action_gain

        lin_vel, rot_vel, gripper_vel = self._limit_velocity(pos_action, euler_action, gripper_action)

        action = np.concatenate([lin_vel, rot_vel, [gripper_vel]]).clip(-1.0, 1.0)
        return action.astype(np.float64)

    def _publish_action(self, action: np.ndarray):
        now = self.get_clock().now()
        msg = PolymetisRobotCommand()
        msg.header = Header()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = ""  # base frame handled in polymetis_manager

        msg.action_space = "cartesian_velocity"
        msg.gripper_action_space = "velocity"
        msg.command = [float(x) for x in action.tolist()]
        msg.blocking = False
        msg.velocity = True
        msg.cartesian_noise = []

        self._cmd_pub.publish(msg)

    def _tick(self):
        action = self._compute_action()
        self._publish_action(action)


def main(args=None):
    rclpy.init(args=args)
    node = TeleoperationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()


