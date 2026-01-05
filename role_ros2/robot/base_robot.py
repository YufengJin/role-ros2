"""Base robot interface for role-ros2."""

from abc import ABC, abstractmethod
from typing import Dict, Optional, Tuple


class BaseRobot(ABC):
    """
    Base class for robot interfaces.
    
    All robot implementations should inherit from this class and implement
    the required methods.
    """
    
    @abstractmethod
    def get_robot_state(self) -> Tuple[Dict, Dict]:
        """
        Get current robot state.
        
        Returns:
            Tuple[dict, dict]: (state_dict, timestamp_dict)
                - state_dict: Dictionary containing robot state (joint_positions, cartesian_position, etc.)
                - timestamp_dict: Dictionary containing timestamps for state readings
        """
        pass
    
    @abstractmethod
    def update_command(
        self,
        command,
        action_space: str = "cartesian_velocity",
        gripper_action_space: Optional[str] = None,
        blocking: bool = False
    ) -> Dict:
        """
        Update robot command (arm + gripper).
        
        Args:
            command: Command array (7 for arm + 1 for gripper, or 6 for cartesian + 1 gripper)
            action_space: "cartesian_position", "joint_position", "cartesian_velocity", "joint_velocity"
            gripper_action_space: "position" or "velocity" (optional)
            blocking: Whether to wait for command completion
            
        Returns:
            dict: Action dictionary containing joint_position, gripper_position, etc.
        """
        pass
    
    @abstractmethod
    def create_action_dict(
        self,
        action,
        action_space: str = "cartesian_velocity",
        gripper_action_space: Optional[str] = None
    ) -> Dict:
        """
        Create action dictionary from command.
        
        Args:
            action: Command array
            action_space: "cartesian_position", "joint_position", "cartesian_velocity", "joint_velocity"
            gripper_action_space: "position" or "velocity" (optional)
            
        Returns:
            dict: Action dictionary containing joint_position, gripper_position, etc.
        """
        pass
    
    @abstractmethod
    def reset(self, randomize: bool = False, wait_for_completion: bool = True, wait_time_sec: float = 20.0):
        """
        Reset robot to home position.
        
        Args:
            randomize: If True, add random noise to reset position
            wait_for_completion: Whether to wait for reset to complete
            wait_time_sec: Maximum time to wait for reset completion (seconds)
        """
        pass
