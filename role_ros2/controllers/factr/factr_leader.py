"""High-rate FACTR leader arm interface.

This module owns the FACTR Dynamixel hardware loop only. It does not command
the Franka follower directly; policy/collector code reads cached leader state
and sends follower commands through RobotEnv.
"""

import os
import subprocess
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Optional, Tuple

import numpy as np

from role_ros2.controllers.factr.config import (
    load_factr_config,
    resolve_urdf_path,
)


@dataclass
class FactrLeaderState:
    """Thread-safe snapshot of FACTR leader state."""

    joint_positions: np.ndarray
    joint_velocities: np.ndarray
    gripper_position_raw: float
    gripper_position: float
    gripper_velocity: float
    timestamp: float
    sequence: int
    ready: bool


def find_ttyusb(port_name: str) -> str:
    """Locate the ttyUSB device behind a /dev/serial/by-id name."""
    base_path = "/dev/serial/by-id/"
    full_path = os.path.join(base_path, port_name)
    if not os.path.exists(full_path):
        raise FileNotFoundError(f"Port '{port_name}' does not exist in {base_path}")
    resolved_path = os.readlink(full_path)
    actual_device = os.path.basename(resolved_path)
    if not actual_device.startswith("ttyUSB"):
        raise RuntimeError(
            f"Port '{port_name}' does not correspond to ttyUSB; it links to {resolved_path}"
        )
    return actual_device


class FactrLeaderInterface:
    """High-frequency FACTR leader controller with cached non-blocking reads."""

    CALIBRATION_RANGE_MULTIPLIER = 20

    def __init__(self, config_path: Optional[str] = None, autostart: bool = False):
        self.config = load_factr_config(config_path)

        self.running = False
        self._control_thread: Optional[threading.Thread] = None
        self._state_lock = threading.Lock()
        self._feedback_lock = threading.Lock()
        self._shutdown_lock = threading.Lock()

        self.driver = None
        self.pin = None
        self.pin_model = None
        self.pin_data = None

        self._sequence = 0
        self._ready = False
        self._last_loop_time = 0.0
        self._last_loop_duration = 0.0
        self._loop_overruns = 0
        self._external_torque_latest = np.zeros(7, dtype=np.float64)

        try:
            self._setup_parameters()

            self._leader_arm_pos_latest = np.zeros(self.num_arm_joints, dtype=np.float64)
            self._leader_arm_vel_latest = np.zeros(self.num_arm_joints, dtype=np.float64)
            self._leader_gripper_raw_latest = 0.0
            self._leader_gripper_pos_latest = 0.0
            self._leader_gripper_vel_latest = 0.0

            self._prepare_dynamixel()
            self._prepare_inverse_dynamics()
            self._calibrate_system()
        except Exception:
            self.stop()
            raise

        if autostart:
            self.start()

    def _setup_parameters(self) -> None:
        cfg = self.config
        self.name = cfg.get("name", "factr_leader")

        controller_cfg = cfg.get("controller", {})
        self.frequency = float(controller_cfg.get("frequency", 500.0))
        self.dt = 1.0 / self.frequency

        arm_cfg = cfg.get("arm_teleop", {})
        self.num_arm_joints = int(arm_cfg.get("num_arm_joints", 7))
        safety_margin = float(arm_cfg.get("arm_joint_limits_safety_margin", 0.0))
        self.arm_joint_limits_max = (
            np.asarray(arm_cfg.get("arm_joint_limits_max", [np.inf] * self.num_arm_joints), dtype=float)
            - safety_margin
        )
        self.arm_joint_limits_min = (
            np.asarray(arm_cfg.get("arm_joint_limits_min", [-np.inf] * self.num_arm_joints), dtype=float)
            + safety_margin
        )
        init_cfg = arm_cfg.get("initialization", {})
        self.calibration_joint_pos = np.asarray(
            init_cfg.get("calibration_joint_pos", [0.0] * self.num_arm_joints),
            dtype=float,
        )
        
        self.search_dynamixel_offsets = bool(init_cfg.get("search_dynamixel_offsets", True))
        configured_joint_offsets = init_cfg.get("factr_joint_offsets", None)
        self.configured_joint_offsets = (
            None
            if configured_joint_offsets is None
            else np.asarray(configured_joint_offsets, dtype=np.float64)
        )
        self.initial_joint_pos = init_cfg.get("initial_joint_pos", None)
        if self.initial_joint_pos is not None:
            self.initial_joint_pos = np.asarray(self.initial_joint_pos, dtype=np.float64)

        gripper_cfg = cfg.get("gripper_teleop", {})
        self.gripper_limit_min = 0.0
        self.gripper_limit_max = float(gripper_cfg.get("actuation_range", 0.8))
        self.gripper_pos_prev = 0.0
        self.gripper_pos = 0.0
        self.leader_gripper_raw_rad = 0.0

        gravity_cfg = controller_cfg.get("gravity_comp", {})
        self.enable_gravity_comp = bool(gravity_cfg.get("enable", True))
        self.gravity_comp_modifier = float(gravity_cfg.get("gain", 1.0))
        self.tau_g = np.zeros(self.num_arm_joints, dtype=np.float64)

        friction_cfg = controller_cfg.get("static_friction_comp", {})
        self.stiction_comp_enable_speed = float(friction_cfg.get("enable_speed", 0.9))
        self.stiction_comp_gain = float(friction_cfg.get("gain", 0.0))
        self.stiction_dither_flag = np.ones(self.num_arm_joints, dtype=bool)

        limit_cfg = controller_cfg.get("joint_limit_barrier", {})
        self.joint_limit_kp = float(limit_cfg.get("kp", 0.0))
        self.joint_limit_kd = float(limit_cfg.get("kd", 0.0))

        null_cfg = controller_cfg.get("null_space_regulation", {})
        self.null_space_joint_target = np.asarray(
            null_cfg.get("null_space_joint_target", [0.0] * self.num_arm_joints),
            dtype=float,
        )
        self.null_space_kp = float(null_cfg.get("kp", 0.0))
        self.null_space_kd = float(null_cfg.get("kd", 0.0))

        feedback_cfg = controller_cfg.get("torque_feedback", {})
        self.enable_torque_feedback = bool(feedback_cfg.get("enable", False))
        self.torque_feedback_gain = float(feedback_cfg.get("gain", 0.0))
        self.torque_feedback_motor_scalar = float(feedback_cfg.get("motor_scalar", 1.0))
        self.torque_feedback_damping = float(feedback_cfg.get("damping", 0.0))

    def _prepare_dynamixel(self) -> None:
        from role_ros2.controllers.factr.dynamixel_driver import DynamixelDriver

        dyn_cfg = self.config.get("dynamixel", {})
        self.servo_types = dyn_cfg.get("servo_types", [])
        self.num_motors = len(self.servo_types)
        if self.num_motors < self.num_arm_joints + 1:
            raise ValueError(
                f"Expected at least {self.num_arm_joints + 1} FACTR motors, got {self.num_motors}"
            )

        self.joint_signs = np.asarray(dyn_cfg.get("joint_signs", [1.0] * self.num_motors), dtype=float)
        if len(self.joint_signs) != self.num_motors:
            raise ValueError("dynamixel.joint_signs length must match dynamixel.servo_types")

        port_name = dyn_cfg.get("dynamixel_port", "")
        if not port_name:
            raise ValueError("FACTR config missing dynamixel.dynamixel_port")
        self.dynamixel_port = port_name if port_name.startswith("/") else "/dev/serial/by-id/" + port_name

        self._warn_if_latency_timer_is_slow(Path(self.dynamixel_port).name)

        joint_ids = (np.arange(self.num_motors) + 1).tolist()
        self.driver = DynamixelDriver(joint_ids, self.servo_types, self.dynamixel_port)
        self.driver.set_torque_mode(False)
        if self.enable_gravity_comp:
            self.driver.set_operating_mode(0)  # current control mode
            self.driver.set_torque_mode(True)

    def _warn_if_latency_timer_is_slow(self, port_name: str) -> None:
        try:
            ttyusb = find_ttyusb(port_name)
            latency_path = f"/sys/bus/usb-serial/devices/{ttyusb}/latency_timer"
            result = subprocess.run(
                ["cat", latency_path],
                capture_output=True,
                text=True,
                check=True,
            )
            latency = int(result.stdout)
            if latency != 1:
                print(
                    f"Warning: FACTR latency timer of {ttyusb} is {latency}; "
                    f"run: echo 1 | sudo tee {latency_path}"
                )
        except Exception as exc:
            print(f"Warning: could not verify FACTR USB latency timer: {exc}")

    def _prepare_inverse_dynamics(self) -> None:
        try:
            import pinocchio as pin
        except ImportError as exc:
            raise ImportError("pinocchio is required for FACTR leader gravity compensation") from exc
        self.pin = pin
        urdf_path = resolve_urdf_path(self.config)
        self.pin_model = pin.buildModelFromUrdf(str(urdf_path))
        self.pin_data = self.pin_model.createData()

    def _calibrate_system(self) -> None:
        if self.driver is None:
            raise RuntimeError("FACTR Dynamixel driver not initialized")
        
        for _ in range(10):
            self.driver.get_positions_and_velocities()

        expected_offset_count = self.num_arm_joints + 1
        if not self.search_dynamixel_offsets:
            if self.configured_joint_offsets is None:
                raise ValueError(
                    "arm_teleop.initialization.search_dynamixel_offsets is false, "
                    "but arm_teleop.initialization.factr_joint_offsets is missing"
                )
            if len(self.configured_joint_offsets) != expected_offset_count:
                raise ValueError(
                    "arm_teleop.initialization.factr_joint_offsets must contain "
                    f"{expected_offset_count} values ({self.num_arm_joints} arm + 1 gripper), "
                    f"got {len(self.configured_joint_offsets)}"
                )
                
            self.joint_offsets = self.configured_joint_offsets.copy()
            
            # loop through all joints and add +- 2pi to the joint offsets to get the closest to start joints
            new_joint_offsets = []
            curr_joints, _ = self.driver.get_positions_and_velocities()
            start_joints = self.initial_joint_pos
            assert curr_joints.shape == start_joints.shape
            
            for idx, (c_joint, s_joint, offset, joint_sign) in enumerate(
                zip(curr_joints, start_joints, self.joint_offsets, self.joint_signs)
            ):
                best_bias = 0.0
                best_error = np.inf
                for bias in [0.0, -2 * np.pi, 2 * np.pi]:
                    
                    joint_i = joint_sign * (c_joint - offset - bias)
                    error = abs(joint_i - s_joint)
                    if error < best_error:
                        best_error = error
                        best_bias = bias
                new_joint_offsets.append(offset+best_bias)
                    
            self.joint_offsets = np.array(new_joint_offsets)
            
            
            print(
                "FACTR using configured Dynamixel joint offsets: "
                # f"{[round(float(x), 8) for x in self.joint_offsets]}"
            )
            curr_joints_cld = (curr_joints - self.joint_offsets) * self.joint_signs
            print(f"current_joints: {curr_joints_cld}")
            return

        curr_joints, _ = self.driver.get_positions_and_velocities()
        joint_offsets = []
        for i in range(self.num_arm_joints):
            best_offset = 0.0
            best_error = np.inf
            joint_sign_i = self.joint_signs[i]
            
            calibration_step_count =  4 * self.CALIBRATION_RANGE_MULTIPLIER + 1
            # the flange is devided by 8 instead of 4 for joint 1 and 3, unless you install it aligned gear mark
            if i==1 or i==3:
                calibration_step_count = 8 * self.CALIBRATION_RANGE_MULTIPLIER + 1
            
                        
            for offset in np.linspace(
                -self.CALIBRATION_RANGE_MULTIPLIER * np.pi,
                self.CALIBRATION_RANGE_MULTIPLIER * np.pi,
                calibration_step_count,
            ):
                joint_i = joint_sign_i * (curr_joints[i] - offset)
                error = abs(joint_i - self.calibration_joint_pos[i])
                if error < best_error:
                    best_error = error
                    best_offset = offset
            joint_offsets.append(best_offset)

        joint_offsets.append(curr_joints[-1])
        self.joint_offsets = np.asarray(joint_offsets, dtype=np.float64)
        print("FACTR searched Dynamixel joint offsets.")
        print(f"  factr_joint_offsets: {[round(float(x), 8) for x in self.joint_offsets]}")

    def start(self) -> None:
        """Start the high-frequency leader loop."""
        if self.running:
            return
        self.running = True
        self._control_thread = threading.Thread(
            target=self._control_loop,
            daemon=True,
            name="factr_leader_500hz",
        )
        self._control_thread.start()

    def stop(self) -> None:
        """Stop the leader loop and disable leader torques."""
        with self._shutdown_lock:
            self.running = False
            with self._state_lock:
                self._ready = False
            if self._control_thread is not None and self._control_thread.is_alive():
                self._control_thread.join(timeout=2.0)
            self._control_thread = None
            if self.driver is not None:
                try:
                    num_arm_joints = int(getattr(self, "num_arm_joints", 7))
                    self.set_leader_joint_torque(np.zeros(num_arm_joints), 0.0)
                except Exception:
                    pass
                try:
                    self.driver.set_torque_mode(False)
                    self.driver.close()
                except Exception:
                    pass
                self.driver = None
            with self._state_lock:
                self._ready = False

    shutdown = stop

    def _control_loop(self) -> None:
        next_t = time.perf_counter()
        while self.running:
            loop_start = time.perf_counter()
            try:
                self.control_loop_step()
            except Exception as exc:
                with self._state_lock:
                    self._ready = False
                print(f"[FACTR] leader control loop error: {exc}")

            self._last_loop_duration = time.perf_counter() - loop_start
            next_t += self.dt
            sleep_time = next_t - time.perf_counter()
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                self._loop_overruns += 1
                next_t = time.perf_counter()

    def get_leader_joint_states(self) -> Tuple[np.ndarray, np.ndarray, float, float]:
        """Read current leader joint positions/velocities from Dynamixel."""
        if self.driver is None:
            raise RuntimeError("FACTR Dynamixel driver not initialized")
        self.gripper_pos_prev = self.gripper_pos
        joint_pos, joint_vel = self.driver.get_positions_and_velocities()

        joint_pos_arm = (
            joint_pos[: self.num_arm_joints] - self.joint_offsets[: self.num_arm_joints]
        ) * self.joint_signs[: self.num_arm_joints]
        joint_vel_arm = joint_vel[: self.num_arm_joints] * self.joint_signs[: self.num_arm_joints]

        self.leader_gripper_raw_rad = float(joint_pos[-1])
        self.gripper_pos = (joint_pos[-1] - self.joint_offsets[-1]) * self.joint_signs[-1]
        gripper_vel = (self.gripper_pos - self.gripper_pos_prev) / self.dt
        return joint_pos_arm, joint_vel_arm, float(self.gripper_pos), float(gripper_vel)

    def set_leader_joint_torque(self, arm_torque: np.ndarray, gripper_torque: float) -> None:
        """Apply torque to FACTR leader arm and gripper motors."""
        if self.driver is None:
            raise RuntimeError("FACTR Dynamixel driver not initialized")
        arm_gripper_torque = np.append(arm_torque, gripper_torque)
        self.driver.set_torque((arm_gripper_torque * self.joint_signs).tolist())

    def control_loop_step(self) -> None:
        """Run one high-frequency FACTR leader control step."""
        leader_q, leader_qd, leader_gripper, leader_gripper_vel = self.get_leader_joint_states()

        if not self.running:
            return

        with self._state_lock:
            if not self.running:
                return
            self._leader_arm_pos_latest[:] = leader_q
            self._leader_arm_vel_latest[:] = leader_qd
            self._leader_gripper_raw_latest = float(self.leader_gripper_raw_rad)
            self._leader_gripper_pos_latest = float(leader_gripper)
            self._leader_gripper_vel_latest = float(leader_gripper_vel)
            self._last_loop_time = time.time()
            self._sequence += 1
            self._ready = True

        torque_arm = np.zeros(self.num_arm_joints, dtype=np.float64)
        torque_l, torque_gripper = self.joint_limit_barrier(
            leader_q, leader_qd, leader_gripper, leader_gripper_vel
        )
        torque_arm += torque_l
        torque_arm += self.null_space_regulation(leader_q, leader_qd)

        if self.enable_gravity_comp:
            torque_arm += self.gravity_compensation(leader_q, leader_qd)
            torque_arm += self.friction_compensation(leader_qd)

        if self.enable_torque_feedback:
            with self._feedback_lock:
                external_torque = self._external_torque_latest.copy()
            torque_arm += self.torque_feedback(external_torque, leader_qd)

        if self.enable_gravity_comp:
            self.set_leader_joint_torque(torque_arm, torque_gripper)

    def joint_limit_barrier(
        self,
        arm_joint_pos: np.ndarray,
        arm_joint_vel: np.ndarray,
        gripper_joint_pos: float,
        gripper_joint_vel: float,
    ) -> Tuple[np.ndarray, float]:
        """Compute repulsive torques near leader joint limits."""
        exceed_max = arm_joint_pos > self.arm_joint_limits_max
        tau_l = (
            -self.joint_limit_kp * (arm_joint_pos - self.arm_joint_limits_max)
            - self.joint_limit_kd * arm_joint_vel
        ) * exceed_max

        exceed_min = arm_joint_pos < self.arm_joint_limits_min
        tau_l += (
            -self.joint_limit_kp * (arm_joint_pos - self.arm_joint_limits_min)
            - self.joint_limit_kd * arm_joint_vel
        ) * exceed_min

        if gripper_joint_pos > self.gripper_limit_max:
            tau_gripper = (
                -self.joint_limit_kp * (gripper_joint_pos - self.gripper_limit_max)
                - self.joint_limit_kd * gripper_joint_vel
            )
        elif gripper_joint_pos < self.gripper_limit_min:
            tau_gripper = (
                -self.joint_limit_kp * (gripper_joint_pos - self.gripper_limit_min)
                - self.joint_limit_kd * gripper_joint_vel
            )
        else:
            tau_gripper = 0.0
        return tau_l, float(tau_gripper)

    def gravity_compensation(self, arm_joint_pos: np.ndarray, arm_joint_vel: np.ndarray) -> np.ndarray:
        """Compute gravity compensation torque with Pinocchio inverse dynamics."""
        self.tau_g = self.pin.rnea(
            self.pin_model,
            self.pin_data,
            arm_joint_pos,
            arm_joint_vel,
            np.zeros_like(arm_joint_vel),
        )
        self.tau_g *= self.gravity_comp_modifier
        return self.tau_g

    def friction_compensation(self, arm_joint_vel: np.ndarray) -> np.ndarray:
        """Compute static friction compensation torque."""
        tau_ss = np.zeros(self.num_arm_joints, dtype=np.float64)
        for i in range(self.num_arm_joints):
            if abs(arm_joint_vel[i]) < self.stiction_comp_enable_speed:
                sign = 1.0 if self.stiction_dither_flag[i] else -1.0
                tau_ss[i] += sign * self.stiction_comp_gain * abs(self.tau_g[i])
                self.stiction_dither_flag[i] = ~self.stiction_dither_flag[i]
        return tau_ss

    def null_space_regulation(self, arm_joint_pos: np.ndarray, arm_joint_vel: np.ndarray) -> np.ndarray:
        """Compute FACTR null-space regulation torque."""
        if self.null_space_kp == 0.0 and self.null_space_kd == 0.0:
            return np.zeros(self.num_arm_joints, dtype=np.float64)
        jacobian = self.pin.computeJointJacobian(
            self.pin_model,
            self.pin_data,
            arm_joint_pos,
            self.num_arm_joints,
        )
        jacobian_pinv = np.linalg.pinv(jacobian)
        null_projector = np.eye(self.num_arm_joints) - jacobian_pinv @ jacobian
        q_error = arm_joint_pos - self.null_space_joint_target[: self.num_arm_joints]
        return null_projector @ (-self.null_space_kp * q_error - self.null_space_kd * arm_joint_vel)

    def torque_feedback(self, external_torque: np.ndarray, arm_joint_vel: np.ndarray) -> np.ndarray:
        """Map follower external torque into FACTR leader force feedback."""
        tau_ff = -self.torque_feedback_gain / self.torque_feedback_motor_scalar * external_torque
        tau_ff -= self.torque_feedback_damping * arm_joint_vel
        return tau_ff

    def set_follower_external_torque(self, torque: np.ndarray) -> None:
        """Update cached follower torque without blocking the high-rate leader loop."""
        torque_arr = np.zeros(self.num_arm_joints, dtype=np.float64)
        raw = np.asarray(torque, dtype=np.float64).flatten()
        n = min(len(raw), self.num_arm_joints)
        torque_arr[:n] = raw[:n]
        with self._feedback_lock:
            self._external_torque_latest = torque_arr

    def get_latest_state(self) -> FactrLeaderState:
        """Return the latest cached leader state; never touches Dynamixel hardware."""
        with self._state_lock:
            return FactrLeaderState(
                joint_positions=self._leader_arm_pos_latest.copy(),
                joint_velocities=self._leader_arm_vel_latest.copy(),
                gripper_position_raw=float(self._leader_gripper_raw_latest),
                gripper_position=float(self._leader_gripper_pos_latest),
                gripper_velocity=float(self._leader_gripper_vel_latest),
                timestamp=float(self._last_loop_time),
                sequence=int(self._sequence),
                ready=bool(self._ready),
            )

    def is_ready(self) -> bool:
        with self._state_lock:
            return bool(self._ready)

    def diagnostics(self) -> Dict[str, Any]:
        """Return lightweight loop diagnostics."""
        return {
            "running": self.running,
            "ready": self.is_ready(),
            "frequency": self.frequency,
            "last_loop_duration": self._last_loop_duration,
            "loop_overruns": self._loop_overruns,
            "sequence": self._sequence,
        }
