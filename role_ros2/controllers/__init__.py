"""
Controllers module for role_ros2.

This module contains controller implementations for robot control,
including VR controllers for teleoperation.
"""

from role_ros2.controllers.base_controller import BaseController

__all__ = ['BaseController', 'VRPolicy', 'VRBimanPolicy', 'FACTRPolicy']


def __getattr__(name):
    if name in ('VRPolicy', 'VRBimanPolicy'):
        from role_ros2.controllers.oculus_controller import VRPolicy, VRBimanPolicy

        return {'VRPolicy': VRPolicy, 'VRBimanPolicy': VRBimanPolicy}[name]
    if name == 'FACTRPolicy':
        from role_ros2.controllers.factr import FACTRPolicy

        return FACTRPolicy
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
