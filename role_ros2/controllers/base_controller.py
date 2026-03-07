"""
Abstract base class for robot controllers.

Any controller used with CollectTrajectoryBase (or similar pipelines) must
implement the three methods defined here: reset_state, get_info, forward.
"""

from abc import ABC, abstractmethod
from typing import Union, Tuple, Dict

import numpy as np


class BaseController(ABC):
    """
    Abstract interface that every robot controller must satisfy.

    Concrete subclasses include VR controllers (VRPolicy, VRBimanPolicy),
    but the interface is generic enough for keyboard, gamepad, or
    autonomous policies.

    Required ``get_info()`` keys
    ----------------------------
    * ``success``       (bool) -- user signals trajectory success
    * ``failure``       (bool) -- user signals trajectory failure
    * ``controller_on`` (bool) -- device / policy is connected / active

    Movement-enabled signalling is controller-specific:
      - single-arm:  ``movement_enabled``           (bool)
      - bimanual:    ``movement_enabled_left/right`` (bool)

    The trajectory collector calls ``_get_movement_enabled(info)`` (which
    the *script* implements) to normalise this into a single bool.
    """

    @abstractmethod
    def reset_state(self) -> None:
        """Reset internal controller state (origins, history, etc.)."""

    @abstractmethod
    def get_info(self) -> Dict[str, object]:
        """
        Return a status dict for the current controller state.

        Must contain at least:
            success       : bool
            failure       : bool
            controller_on : bool
        """

    @abstractmethod
    def forward(
        self,
        obs_dict: dict,
        include_info: bool = False,
    ) -> Union[np.ndarray, Tuple[np.ndarray, dict]]:
        """
        Compute an action from the current observation.

        Args:
            obs_dict: Observation dictionary (contents depend on the robot).
            include_info: When True, return (action, info_dict) instead of
                          just action.

        Returns:
            np.ndarray of shape (DoF,), or (np.ndarray, dict) when
            include_info is True.
        """
