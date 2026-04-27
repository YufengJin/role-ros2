#!/usr/bin/env python3
"""
Allegro Hand Interface Node — role-ros2 wrapper around the official
allegro_hand_ros2 ros2_control stack.

The official Wonik driver (allegro_hand_ros2/) provides:
  - controller_manager + AllegroHandV4HardwareInterface (CAN @ 1 Mbps)
  - joint_state_broadcaster                  → /joint_states (root)
  - allegro_hand_position_controller         (ForwardCommandController)
                                             ← /allegro_hand_position_controller/commands (Float64MultiArray, 16 elements)
  - allegro_hand_posture_controller          (action server, position primitives)
                                             /allegro_hand_posture_controller/grasp_cmd
  - allegro_hand_grasp_controller            (action server, effort primitives)
                                             /allegro_hand_grasp_controller/grasp_cmd

This wrapper is a thin bidirectional bridge that exposes role-ros2 conventions
(namespaced topics, custom messages, blocking services) so RobotEnv-style
clients can talk to the Allegro hand the same way they talk to the Franka
gripper or xArm:

  Publishers (in /{namespace}):
    joint_states          (sensor_msgs/JointState)  — relayed/filtered
    hand_state            (role_ros2/HandState)
    controller_status     (role_ros2/ControllerStatus)

  Subscribers (in /{namespace}):
    joint_position_controller/command  (role_ros2/JointPositionCommand, 16 DoF)

  Services (in /{namespace}):
    reset                       (role_ros2/Reset)              → posture "home"
    move_to_joint_positions     (role_ros2/MoveToJointPositions) → ForwardCommandController + poll convergence
    grasp_primitive             (role_ros2/GraspPrimitive)    → posture or grasp action

Mock mode: no special code path here. Set ros2_control_hardware_type:=mock_components
in the launch file; the official driver simulates /joint_states and accepts
position commands without touching hardware.
"""

import os
import threading
import time
from typing import List, Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from role_ros2.msg import ControllerStatus, HandState, JointPositionCommand
from role_ros2.srv import GraspPrimitive, MoveToJointPositions, Reset

# Official allegro action type. The package is built in the same workspace.
from allegro_hand_control_msgs.action import GraspCommand


# ============================================================================
# HYPERPARAMETERS
# ============================================================================

POSTURE_PRIMITIVES = {"home", "ready", "off"}
GRASP_PRIMITIVES = {"grasp_3", "grasp_4", "pinch_it", "pinch_mt", "envelop"}

# ============================================================================


class AllegroHandInterfaceNode(Node):
    """ROS 2 wrapper that adapts the official Allegro driver to role-ros2 conventions."""

    def __init__(self, namespace: str = "allegro_hand"):
        super().__init__('allegro_hand_interface_node', namespace=namespace)

        # ---- Parameters (launch passes everything from allegro_hand_config.yaml) ----
        self.declare_parameter('use_mock', False)
        self.declare_parameter('namespace', namespace)
        self.declare_parameter('hand_joint_names', [])
        self.declare_parameter('prefix', 'ah_')
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('home_joints', [])
        self.declare_parameter('move_tolerance', 0.02)
        self.declare_parameter('move_timeout', 5.0)
        self.declare_parameter('auto_home_on_startup', False)
        self.declare_parameter('auto_home_delay', 5.0)
        self.declare_parameter('official_joint_states_topic', '/joint_states')
        self.declare_parameter('official_position_command_topic',
                               '/allegro_hand_position_controller/commands')
        self.declare_parameter('official_posture_action',
                               '/allegro_hand_posture_controller/grasp_cmd')
        self.declare_parameter('official_grasp_action',
                               '/allegro_hand_grasp_controller/grasp_cmd')

        self._use_mock = self.get_parameter('use_mock').get_parameter_value().bool_value
        self._namespace = self.get_parameter('namespace').get_parameter_value().string_value
        self.hand_joint_names = list(
            self.get_parameter('hand_joint_names').get_parameter_value().string_array_value
        )
        self._prefix = self.get_parameter('prefix').get_parameter_value().string_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self._home_joints = list(
            self.get_parameter('home_joints').get_parameter_value().double_array_value
        )
        self._move_tolerance = self.get_parameter('move_tolerance').get_parameter_value().double_value
        self._move_timeout = self.get_parameter('move_timeout').get_parameter_value().double_value
        self._auto_home = self.get_parameter('auto_home_on_startup').get_parameter_value().bool_value
        self._auto_home_delay = self.get_parameter('auto_home_delay').get_parameter_value().double_value
        official_joint_states = self.get_parameter('official_joint_states_topic').get_parameter_value().string_value
        official_position_cmd = self.get_parameter('official_position_command_topic').get_parameter_value().string_value
        official_posture_action = self.get_parameter('official_posture_action').get_parameter_value().string_value
        official_grasp_action = self.get_parameter('official_grasp_action').get_parameter_value().string_value

        if not self.hand_joint_names:
            raise ValueError(
                "hand_joint_names is empty. Launch file must pass it from "
                "config/allegro_hand_config.yaml."
            )
        if len(self.hand_joint_names) != 16:
            self.get_logger().warn(
                f"Expected 16 hand joints for Allegro V4, got {len(self.hand_joint_names)}. "
                "Continuing with whatever was passed in."
            )
        self._n_joints = len(self.hand_joint_names)

        if self._home_joints and len(self._home_joints) != self._n_joints:
            self.get_logger().warn(
                f"home_joints length ({len(self._home_joints)}) != hand_joint_names length "
                f"({self._n_joints}); ignoring home_joints"
            )
            self._home_joints = []

        self.get_logger().info(
            f"AllegroHand wrapper starting: namespace={self._namespace}, "
            f"n_joints={self._n_joints}, prefix={self._prefix}, use_mock={self._use_mock}"
        )

        # ---- State cache (filled by /joint_states subscriber) ----
        self._state_lock = threading.Lock()
        self._latest_positions: Optional[List[float]] = None
        self._latest_velocities: Optional[List[float]] = None
        self._latest_efforts: Optional[List[float]] = None
        self._latest_temperatures: Optional[List[float]] = None
        self._latest_state_ts_ns: int = 0
        # Sticky info from the most recent grasp/posture action result
        self._last_primitive: str = ""
        self._last_stalled: bool = False
        self._last_reached_goal: bool = False

        # ---- Callback groups ----
        self._timer_callback_group = ReentrantCallbackGroup()
        self._sub_callback_group = ReentrantCallbackGroup()
        self._service_callback_group = MutuallyExclusiveCallbackGroup()

        # ---- Subscribers ----
        # Cross-namespace subscribe to the official root /joint_states.
        # The leading slash short-circuits the wrapper namespace.
        self._joint_states_sub = self.create_subscription(
            JointState, official_joint_states,
            self._joint_states_callback, 10,
            callback_group=self._sub_callback_group,
        )

        # In-namespace subscribe for the role-ros2 command topic.
        self._command_sub = self.create_subscription(
            JointPositionCommand,
            'joint_position_controller/command',
            self._joint_position_command_callback, 10,
            callback_group=self._sub_callback_group,
        )

        # ---- Publishers ----
        self._joint_states_pub = self.create_publisher(JointState, 'joint_states', 10)
        self._hand_state_pub = self.create_publisher(HandState, 'hand_state', 10)
        self._controller_status_pub = self.create_publisher(ControllerStatus, 'controller_status', 10)

        # Cross-namespace publisher: the official ForwardCommandController lives at root.
        self._position_cmd_pub = self.create_publisher(
            Float64MultiArray, official_position_cmd, 10
        )

        # ---- Action clients (talk to official controllers, root namespace) ----
        self._posture_client = ActionClient(
            self, GraspCommand, official_posture_action,
            callback_group=self._sub_callback_group,
        )
        self._grasp_client = ActionClient(
            self, GraspCommand, official_grasp_action,
            callback_group=self._sub_callback_group,
        )

        # ---- Services ----
        self.create_service(
            Reset, 'reset',
            self._reset_callback,
            callback_group=self._service_callback_group,
        )
        self.create_service(
            MoveToJointPositions, 'move_to_joint_positions',
            self._move_to_joint_positions_callback,
            callback_group=self._service_callback_group,
        )
        self.create_service(
            GraspPrimitive, 'grasp_primitive',
            self._grasp_primitive_callback,
            callback_group=self._service_callback_group,
        )

        # ---- Publish timer ----
        self._state_timer = self.create_timer(
            1.0 / self.publish_rate, self._publish_states,
            callback_group=self._timer_callback_group,
        )

        # ---- Optional auto-home ----
        if self._auto_home:
            threading.Thread(target=self._home_after_delay, daemon=True).start()

        self.get_logger().info(
            f'AllegroHandInterfaceNode initialized. Listening on {official_joint_states}, '
            f'sending position commands to {official_position_cmd}.'
        )

    # ------------------------------------------------------------------ subs

    def _joint_states_callback(self, msg: JointState):
        """Filter the root /joint_states down to our 16 hand joints (in our order)."""
        # Build name -> idx map from the incoming msg
        idx_map = {n: i for i, n in enumerate(msg.name)}
        try:
            indices = [idx_map[n] for n in self.hand_joint_names]
        except KeyError:
            # Hand joints not in this message — could be a different robot's joint_states.
            return

        positions = [float(msg.position[i]) if i < len(msg.position) else 0.0 for i in indices]
        velocities = [float(msg.velocity[i]) if i < len(msg.velocity) else 0.0 for i in indices]
        efforts = [float(msg.effort[i]) if i < len(msg.effort) else 0.0 for i in indices]
        # Temperatures live in dynamic_joint_states (not joint_states) — leave zero unless we wire that up later.
        temperatures = [0.0] * self._n_joints

        with self._state_lock:
            self._latest_positions = positions
            self._latest_velocities = velocities
            self._latest_efforts = efforts
            self._latest_temperatures = temperatures
            stamp = msg.header.stamp
            self._latest_state_ts_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    def _joint_position_command_callback(self, msg: JointPositionCommand):
        """Translate a 16-DoF JointPositionCommand into a Float64MultiArray for the official FCC."""
        if len(msg.positions) != self._n_joints:
            self.get_logger().warn(
                f"Ignoring joint_position_controller/command: got {len(msg.positions)} "
                f"positions, expected {self._n_joints}."
            )
            return
        out = Float64MultiArray()
        out.data = [float(p) for p in msg.positions]
        self._position_cmd_pub.publish(out)

    # --------------------------------------------------------------- publish

    def _publish_states(self):
        """Pack the latest cached state into namespaced JointState/HandState/ControllerStatus."""
        with self._state_lock:
            positions = list(self._latest_positions) if self._latest_positions else None
            velocities = list(self._latest_velocities) if self._latest_velocities else None
            efforts = list(self._latest_efforts) if self._latest_efforts else None
            temperatures = list(self._latest_temperatures) if self._latest_temperatures else None
            data_ts_ns = self._latest_state_ts_ns
            last_primitive = self._last_primitive
            stalled = self._last_stalled
            reached_goal = self._last_reached_goal

        if positions is None:
            return  # No data yet from official driver.

        now = self.get_clock().now()
        ros_stamp = now.to_msg()

        # Namespaced JointState (filtered to 16 hand joints, our canonical order)
        js = JointState()
        js.header.stamp = ros_stamp
        js.header.frame_id = self._prefix + 'palm_link'
        js.name = list(self.hand_joint_names)
        js.position = positions
        js.velocity = velocities or [0.0] * self._n_joints
        js.effort = efforts or [0.0] * self._n_joints
        self._joint_states_pub.publish(js)

        # HandState aggregate
        hs = HandState()
        hs.header.stamp = ros_stamp
        hs.header.frame_id = self._prefix + 'palm_link'
        hs.joint_positions = positions
        hs.joint_velocities = velocities or [0.0] * self._n_joints
        hs.joint_efforts = efforts or [0.0] * self._n_joints
        hs.temperatures = temperatures or [0.0] * self._n_joints
        hs.stalled = bool(stalled)
        hs.reached_goal = bool(reached_goal)
        hs.last_primitive = last_primitive
        hs.timestamp_ns = int(data_ts_ns) if data_ts_ns else int(now.nanoseconds)
        self._hand_state_pub.publish(hs)

        # Static controller status — ForwardCommandController is always the active position channel
        cs = ControllerStatus()
        cs.header.stamp = ros_stamp
        cs.is_running_policy = True
        cs.controller_mode = "joint_position"
        cs.kp = []
        cs.kd = []
        self._controller_status_pub.publish(cs)

    # ------------------------------------------------------------- services

    def _reset_callback(self, request, response):
        """Drive the hand to home posture via the official posture action."""
        del request  # randomize flag is not meaningful for a hand — ignore.
        try:
            ok, msg, stalled, reached = self._call_grasp_action(
                self._posture_client, command="home", effort=-1.0, blocking=True,
            )
            response.success = bool(ok)
            response.message = msg
        except Exception as e:
            response.success = False
            response.message = f"reset failed: {e}"
        return response

    def _move_to_joint_positions_callback(self, request, response):
        """Send a 16-DoF position via FCC and poll /joint_states until convergence."""
        target = list(request.joint_positions)
        if len(target) != self._n_joints:
            response.success = False
            response.message = (
                f"joint_positions has {len(target)} entries, expected {self._n_joints}"
            )
            response.final_joint_positions = []
            return response

        out = Float64MultiArray()
        out.data = [float(p) for p in target]
        self._position_cmd_pub.publish(out)

        timeout = float(request.time_to_go) if request.time_to_go > 0 else self._move_timeout
        deadline = time.time() + timeout
        converged = False
        while time.time() < deadline:
            with self._state_lock:
                current = list(self._latest_positions) if self._latest_positions else None
            if current is not None and len(current) == self._n_joints:
                err = max(abs(c - t) for c, t in zip(current, target))
                if err <= self._move_tolerance:
                    converged = True
                    break
            time.sleep(0.02)

        with self._state_lock:
            final = list(self._latest_positions) if self._latest_positions else []

        response.success = bool(converged)
        response.message = (
            f"converged within {self._move_tolerance} rad" if converged
            else f"did not converge within {timeout:.2f}s"
        )
        response.final_joint_positions = final
        return response

    def _grasp_primitive_callback(self, request, response):
        """Route to posture or grasp action server based on the primitive name."""
        cmd = (request.command or "").strip()
        if cmd in POSTURE_PRIMITIVES:
            client = self._posture_client
        elif cmd in GRASP_PRIMITIVES:
            client = self._grasp_client
        else:
            response.success = False
            response.message = (
                f"unknown grasp primitive '{cmd}'. "
                f"Posture: {sorted(POSTURE_PRIMITIVES)}; Grasp: {sorted(GRASP_PRIMITIVES)}."
            )
            response.stalled = False
            response.reached_goal = False
            return response

        try:
            ok, msg, stalled, reached = self._call_grasp_action(
                client, command=cmd, effort=float(request.effort),
                blocking=bool(request.blocking),
            )
            response.success = bool(ok)
            response.message = msg
            response.stalled = bool(stalled)
            response.reached_goal = bool(reached)
        except Exception as e:
            response.success = False
            response.message = f"grasp_primitive failed: {e}"
            response.stalled = False
            response.reached_goal = False
        return response

    # ------------------------------------------------------- action helpers

    def _call_grasp_action(self, client: ActionClient, command: str, effort: float,
                           blocking: bool):
        """Send a GraspCommand goal; if blocking, wait for the result.

        We use threading.Event + done callbacks instead of rclpy.spin_until_future_complete
        because this is invoked from a service callback that itself runs inside the
        executor — recursive spin is not allowed. The action client's done callbacks fire
        on the ReentrantCallbackGroup, so the executor stays free to make progress while
        this service thread blocks on the Events.

        Returns (ok, msg, stalled, reached).
        """
        if not client.wait_for_server(timeout_sec=2.0):
            return False, f"action server '{client._action_name}' not available", False, False

        goal = GraspCommand.Goal()
        goal.command = command
        goal.effort = float(effort)

        accepted_event = threading.Event()
        accepted_holder = {}

        def on_goal_response(fut):
            accepted_holder['handle'] = fut.result()
            accepted_event.set()

        send_future = client.send_goal_async(goal)
        send_future.add_done_callback(on_goal_response)

        if not accepted_event.wait(timeout=2.0):
            return False, f"goal '{command}' acceptance timed out", False, False

        goal_handle = accepted_holder.get('handle')
        if goal_handle is None or not goal_handle.accepted:
            return False, f"goal '{command}' rejected", False, False

        if not blocking:
            with self._state_lock:
                self._last_primitive = command
            return True, f"goal '{command}' sent (non-blocking)", False, False

        result_event = threading.Event()
        result_holder = {}

        def on_result(fut):
            result_holder['wrapped'] = fut.result()
            result_event.set()

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(on_result)

        if not result_event.wait(timeout=self._move_timeout):
            return False, f"goal '{command}' timed out after {self._move_timeout}s", False, False

        wrapped = result_holder.get('wrapped')
        if wrapped is None:
            return False, f"goal '{command}' returned no result", False, False

        result = wrapped.result
        stalled = bool(getattr(result, 'stalled', False))
        reached = bool(getattr(result, 'reached_goal', False))
        with self._state_lock:
            self._last_primitive = command
            self._last_stalled = stalled
            self._last_reached_goal = reached
        return True, f"goal '{command}' completed (reached={reached}, stalled={stalled})", stalled, reached

    # -------------------------------------------------------------- startup

    def _home_after_delay(self):
        """Background thread: wait for state + delay, then drive to home posture."""
        time.sleep(self._auto_home_delay)
        # Wait for first /joint_states frame
        deadline = time.time() + 10.0
        while time.time() < deadline:
            with self._state_lock:
                ready = self._latest_positions is not None
            if ready:
                break
            time.sleep(0.1)
        if time.time() >= deadline:
            self.get_logger().warn(
                "auto_home: no /joint_states received within 10s; skipping startup home"
            )
            return
        try:
            self.get_logger().info("auto_home: driving hand to posture 'home'...")
            ok, msg, _, _ = self._call_grasp_action(
                self._posture_client, command="home", effort=-1.0, blocking=True,
            )
            if ok:
                self.get_logger().info("auto_home: hand is at home posture")
            else:
                self.get_logger().warn(f"auto_home: failed — {msg}")
        except Exception as e:
            self.get_logger().warn(f"auto_home: failed — {e}")


def main(args=None):
    rclpy.init(args=args)
    namespace = os.environ.get('HAND_NAMESPACE', 'allegro_hand')
    node = AllegroHandInterfaceNode(namespace=namespace)
    # 4 threads: timer + subs (1) + service (1) + action done callbacks (2)
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
