"""
Calibration configuration parameters for role-ros2.

This module contains Charuco board parameters and other calibration settings.
"""

try:
    from cv2 import aruco
except ImportError:
    # Fallback for OpenCV 4.7+ where aruco is in cv2.aruco submodule
    try:
        import cv2
        aruco = cv2.aruco
    except (ImportError, AttributeError):
        raise ImportError(
            "Failed to import aruco from cv2. "
            "Please ensure opencv-contrib-python is installed."
        )

# Charuco Board Parameters
CHARUCOBOARD_ROWCOUNT = 9
CHARUCOBOARD_COLCOUNT = 14
CHARUCOBOARD_CHECKER_SIZE = 0.0285  # meters
CHARUCOBOARD_MARKER_SIZE = 0.0215  # meters
ARUCO_DICT = aruco.Dictionary_get(aruco.DICT_5X5_100)

# Calibration Thresholds
DEFAULT_INLIER_ERROR_THRESHOLD = 3.0
DEFAULT_REPROJECTION_ERROR_THRESHOLD = 3.0
DEFAULT_NUM_IMG_THRESHOLD = 10
DEFAULT_NUM_CORNER_THRESHOLD = 10
DEFAULT_LIN_ERROR_THRESHOLD = 1e-3
DEFAULT_ROT_ERROR_THRESHOLD = 1e-2
DEFAULT_TRAIN_PERCENTAGE = 0.7

# Calibration Trajectory Parameters
DEFAULT_POS_SCALE = 0.1
DEFAULT_ANGLE_SCALE = 0.2
