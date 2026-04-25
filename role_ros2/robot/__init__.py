"""Robot interfaces for role-ros2.

Each robot implementation is imported defensively: a missing optional
dependency in one robot's stack (e.g. dm_control for the Franka IK solver)
must not break import for the others. If a robot fails to import, accessing
its name will raise the original ImportError lazily.
"""

from role_ros2.robot.base_robot import BaseRobot

__all__ = ['BaseRobot']

try:
    from role_ros2.robot.franka.robot import FrankaRobot
    from role_ros2.robot.franka.bimanual_robot import BimanualFrankaRobot
    __all__ += ['FrankaRobot', 'BimanualFrankaRobot']
except ImportError as _franka_err:  # pragma: no cover
    _franka_import_error = _franka_err

    class _FrankaUnavailable:
        def __init__(self, *args, **kwargs):
            raise ImportError(
                "FrankaRobot is unavailable in this environment: "
                f"{_franka_import_error}"
            )

    FrankaRobot = _FrankaUnavailable  # type: ignore[assignment]
    BimanualFrankaRobot = _FrankaUnavailable  # type: ignore[assignment]

try:
    from role_ros2.robot.xarm.robot import XArmRobot
    __all__ += ['XArmRobot']
except ImportError as _xarm_err:  # pragma: no cover
    _xarm_import_error = _xarm_err

    class _XArmUnavailable:
        def __init__(self, *args, **kwargs):
            raise ImportError(
                "XArmRobot is unavailable in this environment: "
                f"{_xarm_import_error}"
            )

    XArmRobot = _XArmUnavailable  # type: ignore[assignment]
