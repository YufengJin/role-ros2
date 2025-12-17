# Copyright 2025 DROID Team
#
# Licensed under the Apache License, Version 2.0 (the 'License');
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an 'AS IS' BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

# Enable colored output
os.environ["RCUTILS_COLORIZED_OUTPUT"] = "1"

def generate_launch_description():
    # Get the package share directories
    zed_wrapper_share_dir = get_package_share_directory('zed_wrapper')
    role_ros2_share_dir = get_package_share_directory('role_ros2')
    
    # Paths to parameter override files
    # All camera parameters (resolution, frame rate, depth mode, IMU settings, etc.) 
    # are configured in the YAML files below
    hand_camera_params = os.path.join(role_ros2_share_dir, 'config', 'hand_camera_params.yaml')
    static_camera_params = os.path.join(role_ros2_share_dir, 'config', 'static_camera_params.yaml')
    
    # Hand Camera (mounted on gripper) - ZED-M
    # Configuration: See hand_camera_params.yaml for all parameters including:
    #   - serial_number: 11022812
    #   - resolution, frame rates, depth mode, IMU settings, etc.
    hand_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(zed_wrapper_share_dir, 'launch', 'zed_camera.launch.py')
        ]),
        launch_arguments={
            'camera_model': 'zedm',  # Required launch argument (also specified in YAML)
            'namespace': 'hand_camera',  # Topic namespace: /hand_camera/...
            'ros_params_override_path': hand_camera_params,  # Load all parameters from YAML
            'node_name': 'zed_node',
        }.items()
    )
    
    # Static Camera (fixed position) - ZED 2
    # Configuration: See static_camera_params.yaml for all parameters including:
    #   - serial_number: 24285872
    #   - resolution, frame rates, depth mode, IMU settings, etc.
    static_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(zed_wrapper_share_dir, 'launch', 'zed_camera.launch.py')
        ]),
        launch_arguments={
            'camera_model': 'zed2',  # Required launch argument (also specified in YAML)
            'namespace': 'static_camera',  # Topic namespace: /static_camera/...
            'ros_params_override_path': static_camera_params,  # Load all parameters from YAML
            'node_name': 'zed_node',
        }.items()
    )
    
    return LaunchDescription([
        # Launch both cameras with configurations from YAML files
        hand_camera_launch,
        static_camera_launch,
    ])

