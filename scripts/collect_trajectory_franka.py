#!/usr/bin/env python3
"""
Collect Trajectory - Single-arm Franka with VR controller (VRPolicy).

Author: Chaser Robotics Team

Usage:
    python3 collect_trajectory_franka.py --task pick_and_place --viz
    python3 collect_trajectory_franka.py --task pick_and_place --left-controller
"""

import argparse

from role_ros2.controllers.oculus_controller import VRPolicy
from role_ros2.trajectory_utils.collect_trajectory_base import (
    CollectTrajectoryBase,
    add_common_args,
    run_collector,
)


class CollectTrajectoryFranka(CollectTrajectoryBase):
    """Single-arm Franka trajectory collection using VRPolicy."""

    def __init__(self, args):
        self.right_controller = args.right_controller
        super().__init__(args)

    def _create_robot(self, node):
        return None

    def _create_controller(self):
        return VRPolicy(
            right_controller=self.right_controller,
            pos_vel_scale=self.pos_vel_scale,
            rot_vel_scale=self.rot_vel_scale,
            mirror_xy=self.mirror,
        )

    def _get_movement_enabled(self, controller_info: dict) -> bool:
        return controller_info.get("movement_enabled", False)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Collect single-arm Franka trajectories with VR controller",
    )
    add_common_args(parser)

    parser.add_argument("--right-controller", action="store_true", default=True,
                        help="Use right VR controller (default)")
    parser.add_argument("--left-controller", dest="right_controller",
                        action="store_false", help="Use left VR controller")

    return parser.parse_args()


if __name__ == "__main__":
    run_collector(CollectTrajectoryFranka, parse_args())
