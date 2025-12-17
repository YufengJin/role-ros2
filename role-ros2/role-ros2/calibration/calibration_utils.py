import json
import os
import time

from role_ros2.misc.parameters import *
from role_ros2.misc.transformations import *


# Prepare Calibration Info #
dir_path = os.path.dirname(os.path.realpath(__file__))
calib_info_filepath = os.path.join(dir_path, "calibration_info.json")


def load_calibration_info(keep_time=False):
    if not os.path.isfile(calib_info_filepath):
        return {}
    with open(calib_info_filepath, "r") as jsonFile:
        calibration_info = json.load(jsonFile)
    if not keep_time:
        calibration_info = {key: data["pose"] for key, data in calibration_info.items()}
    return calibration_info

