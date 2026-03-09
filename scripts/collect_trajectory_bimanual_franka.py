#!/usr/bin/env python3
"""
Collect Trajectory - Bimanual Franka with VR controller (VRBimanPolicy).

Author: Chaser Robotics Team

Usage:
    python3 collect_trajectory_bimanual_franka.py --task pick_and_place --viz
"""

import argparse

from role_ros2.controllers.oculus_controller import VRBimanPolicy
from role_ros2.robot.franka.bimanual_robot import BimanualFrankaRobot
from role_ros2.trajectory_utils.collect_trajectory_base import (
    CollectTrajectoryBase,
    add_common_args,
    run_collector,
)


class CollectTrajectoryBimanualFranka(CollectTrajectoryBase):
    """Bimanual Franka trajectory collection using VRBimanPolicy."""

    def _create_robot(self, node):
        return BimanualFrankaRobot(node=node)

    def _create_controller(self):
        return VRBimanPolicy(
            pos_vel_scale=self.pos_vel_scale,
            rot_vel_scale=self.rot_vel_scale,
            mirror_xy=self.mirror,
            mirror_arms=self.mirror,
        )

    def _get_movement_enabled(self, controller_info: dict) -> bool:
        return (
            controller_info.get("movement_enabled_left", False)
            or controller_info.get("movement_enabled_right", False)
        )


def parse_args():
    parser = argparse.ArgumentParser(
        description="Collect bimanual Franka trajectories with VR controller",
    )
    add_common_args(parser)
    return parser.parse_args()


if __name__ == "__main__":
    run_collector(CollectTrajectoryBimanualFranka, parse_args())
