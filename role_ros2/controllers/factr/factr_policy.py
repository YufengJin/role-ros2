"""FACTR teleoperation policy for role_ros2 collectors."""

import select
import sys
import time
from typing import Dict, Optional, Tuple, Union

import numpy as np

from role_ros2.controllers.base_controller import BaseController
from role_ros2.controllers.factr.factr_leader import FactrLeaderInterface, FactrLeaderState


class FACTRPolicy(BaseController):
    """
    Convert FACTR leader state into role_ros2 robot actions.

    The FACTR leader loop runs independently at high frequency inside
    FactrLeaderInterface. This policy only reads cached state and therefore
    cannot block leader gravity compensation.
    """

    def __init__(self, config_path: Optional[str] = None, autostart: bool = True):
        self.leader = FactrLeaderInterface(config_path=config_path, autostart=autostart)
        self.config = self.leader.config
        self.teleop_cfg = self.config.get("teleop", {})
        self.mapping_cfg = self.teleop_cfg.get("mapping", {})
        self.keyboard_cfg = self.teleop_cfg.get("keyboard", {})

        self.control_hz = float(self.teleop_cfg.get("hz", 30.0))
        self.action_space = self.teleop_cfg.get("action_space", "joint_position")
        if self.action_space != "joint_position":
            raise ValueError(
                "FACTRPolicy currently supports action_space='joint_position' only. "
                f"Got: {self.action_space}"
            )

        self.use_keyboard = bool(self.keyboard_cfg.get("enable", True))
        self.start_key = str(self.keyboard_cfg.get("start_key", "s")).lower()
        self.stop_key = str(self.keyboard_cfg.get("stop_key", "f")).lower()
        self.success_key = str(self.keyboard_cfg.get("success_key", "a")).lower()
        self.failure_key = str(self.keyboard_cfg.get("failure_key", "b")).lower()
        self._keypress_latch_s = float(self.keyboard_cfg.get("keypress_latch_s", 0.75))

        self._start_enabled_default = bool(self.teleop_cfg.get("start_enabled", False))
        self._movement_enabled = self._start_enabled_default
        self._success_until = 0.0
        self._failure_until = 0.0

        alignment_cfg = self.teleop_cfg.get("alignment", {})
        self.alignment_enabled = bool(alignment_cfg.get("enabled", True))
        self.alignment_threshold = float(alignment_cfg.get("joint_threshold", 0.002))
        self.alignment_max_joint_step = float(alignment_cfg.get("max_joint_step", 0.02))
        self.alignment_print_period_s = float(alignment_cfg.get("print_period_s", 1.0))
        self.exclude_alignment_from_trajectory = bool(
            alignment_cfg.get("exclude_from_trajectory", True)
        )
        self._last_movement_enabled = self._movement_enabled
        self._alignment_done = not (self.alignment_enabled and self._movement_enabled)
        self._last_alignment_print_t = 0.0

        self.joint_indices = np.asarray(
            self.mapping_cfg.get("joint_indices", list(range(7))),
            dtype=np.int64,
        )
        self.joint_signs = np.asarray(
            self.mapping_cfg.get("joint_signs", [1.0] * 7),
            dtype=np.float64,
        )
        self.joint_offsets = np.asarray(
            self.mapping_cfg.get("joint_offsets", [0.0] * 7),
            dtype=np.float64,
        )
        if len(self.joint_indices) != 7 or len(self.joint_signs) != 7 or len(self.joint_offsets) != 7:
            raise ValueError(
                "FACTR teleop mapping requires 7 joint_indices, 7 joint_signs, and 7 joint_offsets"
            )

        self.gripper_open_raw = float(self.mapping_cfg.get("gripper_open_raw", 0.0))
        self.gripper_closed_raw = float(
            self.mapping_cfg.get("gripper_closed_raw", self.leader.gripper_limit_max)
        )
        self.gripper_invert = bool(self.mapping_cfg.get("gripper_invert", False))
        self.gripper_deadband = float(self.mapping_cfg.get("gripper_deadband", 0.0))

        self.feedback_source = str(
            self.teleop_cfg.get("force_feedback", {}).get(
                "source", "motor_torques_external"
            )
        )
        self.feedback_fallback_zero = bool(
            self.teleop_cfg.get("force_feedback", {}).get("fallback_zero", True)
        )

        self._last_action_info: Dict[str, object] = {}

        if self.use_keyboard:
            print(
                "FACTR teleop keyboard: "
                f"'{self.start_key}' start, '{self.stop_key}' stop, "
                f"'{self.success_key}' success, '{self.failure_key}' failure"
            )

    def reset_state(self) -> None:
        """Reset policy alignment and latches."""
        self._movement_enabled = self._start_enabled_default
        self._success_until = 0.0
        self._failure_until = 0.0
        self._last_movement_enabled = self._movement_enabled
        self._alignment_done = not (self.alignment_enabled and self._movement_enabled)
        self._last_alignment_print_t = 0.0

    def shutdown(self) -> None:
        """Stop FACTR leader hardware loop."""
        self._movement_enabled = False
        self._alignment_done = not self.alignment_enabled
        self.leader.shutdown()

    def _poll_keyboard(self) -> None:
        if not self.use_keyboard:
            return
        try:
            rlist, _, _ = select.select([sys.stdin], [], [], 0.0)
        except Exception:
            return
        now = time.time()
        while rlist:
            line = sys.stdin.readline()
            if not line:
                return
            key = line.strip().lower()
            if key == self.start_key:
                self._movement_enabled = True
                self._alignment_done = not self.alignment_enabled
                self._last_alignment_print_t = 0.0
            elif key == self.stop_key:
                self._movement_enabled = False
                self._alignment_done = not (self.alignment_enabled and self._movement_enabled)
            elif key == self.success_key:
                self._success_until = now + self._keypress_latch_s
            elif key == self.failure_key:
                self._failure_until = now + self._keypress_latch_s
            try:
                rlist, _, _ = select.select([sys.stdin], [], [], 0.0)
            except Exception:
                return

    def get_info(self) -> Dict[str, object]:
        """Return controller status consumed by CollectTrajectoryBase."""
        self._poll_keyboard()
        now = time.time()
        diagnostics = self.leader.diagnostics()
        return {
            "success": now < self._success_until,
            "failure": now < self._failure_until,
            "movement_enabled": bool(self._movement_enabled),
            "controller_on": bool(diagnostics["running"]),
            "factr_ready": bool(diagnostics["ready"]),
            "factr_initial_alignment": bool(
                self._movement_enabled and self.alignment_enabled and not self._alignment_done
            ),
            "factr_loop_overruns": diagnostics["loop_overruns"],
            "factr_last_loop_duration": diagnostics["last_loop_duration"],
        }

    def _map_joints(self, state: FactrLeaderState) -> np.ndarray:
        leader_q = state.joint_positions[self.joint_indices]
        return self.joint_signs * leader_q + self.joint_offsets

    def _map_gripper(self, state: FactrLeaderState) -> float:
        denom = self.gripper_closed_raw - self.gripper_open_raw
        if abs(denom) < 1e-9:
            return 0.0
        value = (state.gripper_position - self.gripper_open_raw) / denom
        if self.gripper_invert:
            value = 1.0 - value
        if abs(value) < self.gripper_deadband:
            value = 0.0
        if abs(1.0 - value) < self.gripper_deadband:
            value = 1.0
        return float(np.clip(value, 0.0, 1.0))

    def _update_force_feedback(self, obs_dict: dict) -> None:
        robot_state = obs_dict.get("robot_state", {})
        if self.feedback_source == "estimated_external_torque":
            measured = robot_state.get("motor_torques_measured")
            computed = robot_state.get("joint_torques_computed")
            if measured is not None and computed is not None:
                torque = np.asarray(measured, dtype=np.float64) - np.asarray(
                    computed, dtype=np.float64
                )
            else:
                torque = None
        else:
            torque = robot_state.get(self.feedback_source)
        if torque is None and not self.feedback_fallback_zero:
            return
        if torque is None:
            torque = np.zeros(7, dtype=np.float64)
        self.leader.set_follower_external_torque(np.asarray(torque, dtype=np.float64))

    def _get_robot_joints(self, obs_dict: dict, fallback: np.ndarray) -> np.ndarray:
        robot_state = obs_dict.get("robot_state", {})
        robot_q = np.asarray(robot_state.get("joint_positions", fallback), dtype=np.float64)
        if robot_q.shape[0] < 7:
            return fallback
        return robot_q[:7]

    def _get_robot_gripper(self, obs_dict: dict, fallback: float) -> float:
        robot_state = obs_dict.get("robot_state", {})
        try:
            return float(np.clip(robot_state.get("gripper_position", fallback), 0.0, 1.0))
        except Exception:
            return float(np.clip(fallback, 0.0, 1.0))

    def _hold_current_robot_action(
        self,
        obs_dict: dict,
        state: FactrLeaderState,
    ) -> Tuple[np.ndarray, Dict[str, object]]:
        robot_state = obs_dict.get("robot_state", {})
        robot_q_raw = robot_state.get("joint_positions")
        if robot_q_raw is not None and len(robot_q_raw) >= 7:
            target_q = np.asarray(robot_q_raw[:7], dtype=np.float64)
        else:
            raise RuntimeError(
                "Cannot hold Franka safely: current robot_state.joint_positions is missing"
            )

        last_gripper = float(self._last_action_info.get("target_gripper_position", 0.0))
        target_gripper = self._get_robot_gripper(obs_dict, last_gripper)
        action = np.concatenate([target_q, [target_gripper]]).astype(np.float64)

        self._alignment_done = not self.alignment_enabled

        info: Dict[str, object] = {
            "factr_ready": False,
            "factr_phase": "factr_not_ready",
            "factr_initial_alignment": False,
            "exclude_from_trajectory": True,
            "target_joint_position": target_q,
            "target_gripper_position": target_gripper,
            "factr_joint_positions": state.joint_positions,
            "factr_joint_velocities": state.joint_velocities,
            "factr_gripper_position": state.gripper_position,
            "factr_gripper_position_raw": state.gripper_position_raw,
        }

        self._last_action_info = info
        return action, info

    def _apply_initial_alignment(
        self,
        mapped_q: np.ndarray,
        obs_dict: dict,
    ) -> Tuple[np.ndarray, Dict[str, object]]:
        """
        Slowly move Franka from its current joint pose to the absolute FACTR target.

        This is not relative teleop. The final target is still exactly the mapped
        FACTR joint pose; we only clip the transient command during startup to
        avoid a sudden jump.
        """
        info: Dict[str, object] = {
            "factr_phase": "teleop",
            "factr_initial_alignment": False,
            "exclude_from_trajectory": False,
        }

        just_enabled = self._movement_enabled and not self._last_movement_enabled
        self._last_movement_enabled = self._movement_enabled
        if just_enabled and self.alignment_enabled:
            self._alignment_done = False

        if not self._movement_enabled:
            self._alignment_done = not self.alignment_enabled
            info["factr_phase"] = "idle"
            return mapped_q, info

        if not self.alignment_enabled or self._alignment_done:
            return mapped_q, info

        robot_q = self._get_robot_joints(obs_dict, mapped_q)
        diff = mapped_q - robot_q
        max_abs_diff = float(np.max(np.abs(diff)))

        if max_abs_diff <= self.alignment_threshold:
            self._alignment_done = True
            print(
                "FACTR initial alignment complete. "
                f"max_abs_joint_diff={max_abs_diff:.4f} rad"
            )
            return mapped_q, info

        clipped_diff = np.clip(
            diff,
            -self.alignment_max_joint_step,
            self.alignment_max_joint_step,
        )
        target_q = robot_q + clipped_diff

        now = time.time()
        if now - self._last_alignment_print_t >= self.alignment_print_period_s:
            self._last_alignment_print_t = now
            print(
                "FACTR initial alignment active: moving Franka toward FACTR target "
                f"(max_abs_joint_diff={max_abs_diff:.4f} rad, "
                f"threshold={self.alignment_threshold:.4f} rad)"
            )

        info.update(
            {
                "factr_phase": "initial_alignment",
                "factr_initial_alignment": True,
                "exclude_from_trajectory": self.exclude_alignment_from_trajectory,
                "alignment_max_abs_joint_diff": max_abs_diff,
                "alignment_joint_diff": diff,
                "alignment_clipped_joint_diff": clipped_diff,
            }
        )
        return target_q, info

    def forward(
        self,
        obs_dict: dict,
        include_info: bool = False,
    ) -> Union[np.ndarray, Tuple[np.ndarray, dict]]:
        """Compute an 8D joint_position action from cached FACTR leader state."""
        self._update_force_feedback(obs_dict)
        state = self.leader.get_latest_state()

        if not state.ready:
            action, info = self._hold_current_robot_action(obs_dict, state)
            return (action, info) if include_info else action

        mapped_q = self._map_joints(state)
        target_q, alignment_info = self._apply_initial_alignment(mapped_q, obs_dict)
        target_gripper = self._map_gripper(state)
        action = np.concatenate([target_q, [target_gripper]]).astype(np.float64)

        info = {
            "factr_ready": True,
            "factr_joint_positions": state.joint_positions,
            "factr_joint_velocities": state.joint_velocities,
            "factr_gripper_position": state.gripper_position,
            "factr_gripper_position_raw": state.gripper_position_raw,
            "factr_mapped_joint_position": mapped_q,
            "target_joint_position": target_q,
            "target_gripper_position": target_gripper,
        }
        info.update(alignment_info)
        self._last_action_info = info
        if include_info:
            return action, info
        return action
