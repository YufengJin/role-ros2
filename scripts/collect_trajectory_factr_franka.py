#!/usr/bin/env python3
"""
Collect Trajectory - Single-arm Franka with FACTR leader teleoperation.

FACTR owns only the high-frequency leader gravity-compensation loop. Franka
commands still go through RobotEnv/FrankaRobot and the existing ROS2 Polymetis
interface.
"""

import argparse
import sys

from role_ros2.controllers.factr import FACTRPolicy
from role_ros2.trajectory_utils.collect_trajectory_base import (
    CollectTrajectoryBase,
    add_common_args,
    run_collector,
)


class CollectTrajectoryFactrFranka(CollectTrajectoryBase):
    """Single-arm Franka trajectory collection using FACTRPolicy."""

    def __init__(self, args):
        self.factr_config = args.factr_config
        super().__init__(args)

    def _create_robot(self, node):
        return None

    def _create_controller(self):
        return FACTRPolicy(config_path=self.factr_config, autostart=True)

    def _get_movement_enabled(self, controller_info: dict) -> bool:
        return controller_info.get("movement_enabled", False)

    def shutdown(self):
        if hasattr(self, "controller") and hasattr(self.controller, "shutdown"):
            try:
                self.controller.shutdown()
            except Exception as exc:
                self._print(f"FACTR shutdown warning: {exc}")
        super().shutdown()


def _arg_was_provided(flag: str) -> bool:
    return any(arg == flag or arg.startswith(flag + "=") for arg in sys.argv[1:])


def parse_args():
    parser = argparse.ArgumentParser(
        description="Collect single-arm Franka trajectories with FACTR leader teleoperation",
    )
    add_common_args(parser)
    parser.add_argument(
        "--factr-config",
        type=str,
        default=None,
        help="Path to FACTR teleop config. Defaults to config/factr_teleop_config.yaml.",
    )

    args = parser.parse_args()
    if not _arg_was_provided("--action-space"):
        args.action_space = "joint_position"
    if not _arg_was_provided("--control-hz"):
        args.control_hz = 30.0
    return args


if __name__ == "__main__":
    run_collector(CollectTrajectoryFactrFranka, parse_args())
