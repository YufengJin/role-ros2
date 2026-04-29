"""Local Dynamixel driver used by FACTR leader teleoperation.

This keeps FACTR leader hardware access inside role_ros2 instead of importing
the GELLO/FACTR package tree. It is intentionally small: only the operations
needed by FactrLeaderInterface are implemented.
"""

from threading import Lock
from typing import Sequence

import numpy as np

try:
    from dynamixel_sdk.group_sync_read import GroupSyncRead
    from dynamixel_sdk.group_sync_write import GroupSyncWrite
    from dynamixel_sdk.packet_handler import PacketHandler
    from dynamixel_sdk.port_handler import PortHandler
    from dynamixel_sdk.robotis_def import (
        COMM_SUCCESS,
        DXL_HIBYTE,
        DXL_LOBYTE,
    )
except ImportError as exc:  # pragma: no cover - depends on FACTR hardware environment
    raise ImportError(
        "dynamixel_sdk is required for FACTR leader control. Install it in the "
        "environment used to run scripts/collect_trajectory_factr_franka.py."
    ) from exc


ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_CURRENT = 102
LEN_GOAL_CURRENT = 2
ADDR_PRESENT_POSITION = 132
LEN_PRESENT_POSITION = 4
ADDR_PRESENT_VELOCITY = 128
LEN_PRESENT_VELOCITY = 4
ADDR_OPERATING_MODE = 11

TORQUE_ENABLE = 1
TORQUE_DISABLE = 0

CURRENT_CONTROL_MODE = 0
POSITION_CONTROL_MODE = 3

TORQUE_TO_CURRENT_MAPPING = {
    "XC330_T288_T": 1158.73,
    "XM430_W210_T": 1000 / 2.69,
}


class DynamixelDriver:
    """Minimal current-control driver for the FACTR Dynamixel leader."""

    def __init__(
        self,
        ids: Sequence[int],
        servo_types: Sequence[str],
        port: str = "/dev/ttyUSB0",
        baudrate: int = 4_000_000,
    ):
        if len(ids) != len(servo_types):
            raise ValueError("ids and servo_types must have the same length")

        unknown = [servo for servo in servo_types if servo not in TORQUE_TO_CURRENT_MAPPING]
        if unknown:
            raise ValueError(f"Unknown FACTR Dynamixel servo type(s): {unknown}")

        self._ids = list(ids)
        self._lock = Lock()
        self._positions = None
        self._velocities = None

        self._port_handler = PortHandler(port)
        self._packet_handler = PacketHandler(2.0)
        self._group_sync_read = GroupSyncRead(
            self._port_handler,
            self._packet_handler,
            ADDR_PRESENT_VELOCITY,
            LEN_PRESENT_POSITION + LEN_PRESENT_VELOCITY,
        )
        self._group_sync_write = GroupSyncWrite(
            self._port_handler,
            self._packet_handler,
            ADDR_GOAL_CURRENT,
            LEN_GOAL_CURRENT,
        )

        if not self._port_handler.openPort():
            raise RuntimeError(f"Failed to open Dynamixel port: {port}")
        if not self._port_handler.setBaudRate(baudrate):
            raise RuntimeError(f"Failed to set Dynamixel baudrate: {baudrate}")

        for dxl_id in self._ids:
            if not self._group_sync_read.addParam(dxl_id):
                raise RuntimeError(f"Failed to add sync-read parameter for Dynamixel ID {dxl_id}")

        self.torque_to_current_map = np.asarray(
            [TORQUE_TO_CURRENT_MAPPING[servo] for servo in servo_types],
            dtype=np.float64,
        )
        self._torque_enabled = False
        self.set_torque_mode(False)

    @property
    def torque_enabled(self) -> bool:
        return self._torque_enabled

    def close(self) -> None:
        self._port_handler.closePort()

    def set_torque_mode(self, enable: bool) -> None:
        torque_value = TORQUE_ENABLE if enable else TORQUE_DISABLE
        with self._lock:
            for dxl_id in self._ids:
                comm_result, dxl_error = self._packet_handler.write1ByteTxRx(
                    self._port_handler,
                    dxl_id,
                    ADDR_TORQUE_ENABLE,
                    torque_value,
                )
                if comm_result != COMM_SUCCESS or dxl_error != 0:
                    raise RuntimeError(f"Failed to set torque mode for Dynamixel ID {dxl_id}")
        self._torque_enabled = enable

    def set_operating_mode(self, mode: int) -> None:
        with self._lock:
            for dxl_id in self._ids:
                comm_result, dxl_error = self._packet_handler.write1ByteTxRx(
                    self._port_handler,
                    dxl_id,
                    ADDR_OPERATING_MODE,
                    mode,
                )
                if comm_result != COMM_SUCCESS or dxl_error != 0:
                    raise RuntimeError(f"Failed to set operating mode for Dynamixel ID {dxl_id}")

    def get_positions_and_velocities(self, tries: int = 10):
        """Return positions in radians and velocities in rad/s."""
        positions = np.zeros(len(self._ids), dtype=np.int64)
        velocities = np.zeros(len(self._ids), dtype=np.int64)

        comm_result = self._group_sync_read.txRxPacket()
        if comm_result != COMM_SUCCESS:
            if tries > 0:
                return self.get_positions_and_velocities(tries=tries - 1)
            raise RuntimeError(f"Dynamixel sync read failed: {comm_result}")

        for i, dxl_id in enumerate(self._ids):
            if self._group_sync_read.isAvailable(
                dxl_id,
                ADDR_PRESENT_VELOCITY,
                LEN_PRESENT_VELOCITY,
            ):
                velocity = self._group_sync_read.getData(
                    dxl_id,
                    ADDR_PRESENT_VELOCITY,
                    LEN_PRESENT_VELOCITY,
                )
                if velocity > 0x7FFFFFFF:
                    velocity -= 0x100000000
                velocities[i] = velocity
            else:
                raise RuntimeError(f"Failed to read velocity for Dynamixel ID {dxl_id}")

            if self._group_sync_read.isAvailable(
                dxl_id,
                ADDR_PRESENT_POSITION,
                LEN_PRESENT_POSITION,
            ):
                position = self._group_sync_read.getData(
                    dxl_id,
                    ADDR_PRESENT_POSITION,
                    LEN_PRESENT_POSITION,
                )
                if position > 0x7FFFFFFF:
                    position -= 0x100000000
                positions[i] = position
            else:
                raise RuntimeError(f"Failed to read position for Dynamixel ID {dxl_id}")

        self._positions = positions
        self._velocities = velocities
        positions_rad = positions / 2048.0 * np.pi
        velocities_rad_s = velocities * 0.229 * 2 * np.pi / 60.0
        return positions_rad, velocities_rad_s

    def set_current(self, currents: Sequence[float]) -> None:
        if len(currents) != len(self._ids):
            raise ValueError("currents length must match number of Dynamixel IDs")
        if not self._torque_enabled:
            raise RuntimeError("Torque must be enabled before setting current")

        currents = np.clip(np.asarray(currents, dtype=np.float64), -900, 900)
        with self._lock:
            for dxl_id, current in zip(self._ids, currents):
                current_value = int(current)
                param_goal_current = [
                    DXL_LOBYTE(current_value),
                    DXL_HIBYTE(current_value),
                ]
                if not self._group_sync_write.addParam(dxl_id, param_goal_current):
                    raise RuntimeError(f"Failed to set current for Dynamixel ID {dxl_id}")
            comm_result = self._group_sync_write.txPacket()
            self._group_sync_write.clearParam()
            if comm_result != COMM_SUCCESS:
                raise RuntimeError(f"Dynamixel sync write current failed: {comm_result}")

    def set_torque(self, torques: Sequence[float]) -> None:
        currents = self.torque_to_current_map * np.asarray(torques, dtype=np.float64)
        self.set_current(currents)
