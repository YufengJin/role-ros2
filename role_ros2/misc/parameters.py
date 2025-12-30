import os
try:
    from cv2 import aruco
except ImportError:
    # Fallback for OpenCV 4.7+ where aruco is in cv2.aruco submodule
    try:
        import cv2
        aruco = cv2.aruco
    except (ImportError, AttributeError):
        raise ImportError("Failed to import aruco from cv2. Please ensure opencv-contrib-python is installed.")

# Robot Params #
nuc_ip = "172.17.0.1"
robot_ip = "172.17.0.2"
laptop_ip = "172.17.0.1"
sudo_password = "220996"
robot_type = "fr3"  # 'panda' or 'fr3'
robot_serial_number = "290102-2320040"

# Camera ID's #
hand_camera_id = "11022812"
varied_camera_1_id = "24285872"       
varied_camera_2_id = "66666666"                     # not used

# Charuco Board Params #
CHARUCOBOARD_ROWCOUNT = 9
CHARUCOBOARD_COLCOUNT = 14
CHARUCOBOARD_CHECKER_SIZE = 0.0285
CHARUCOBOARD_MARKER_SIZE = 0.0215
ARUCO_DICT = aruco.Dictionary_get(aruco.DICT_5X5_100)

# Ubuntu Pro Token (RT PATCH) #
ubuntu_pro_token = ""

# Code Version [DONT CHANGE] #
droid_version = "1.3"

