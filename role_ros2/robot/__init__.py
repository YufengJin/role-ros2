"""Robot interfaces for role-ros2."""

from role_ros2.robot.base_robot import BaseRobot
from role_ros2.robot.franka.robot import FrankaRobot
from role_ros2.robot.franka.bimanual_robot import BimanualFrankaRobot

__all__ = ['BaseRobot', 'FrankaRobot', 'BimanualFrankaRobot']
