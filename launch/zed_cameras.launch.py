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
import tempfile
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution

# Enable colored output
os.environ["RCUTILS_COLORIZED_OUTPUT"] = "1"


def load_camera_config(config_file):
    """Load camera configuration from YAML file."""
    role_ros2_share_dir = get_package_share_directory('role_ros2')
    config_path = os.path.join(role_ros2_share_dir, 'config', config_file)
    
    if not os.path.exists(config_path):
        print(f"⚠️  Warning: Config file not found: {config_path}")
        print(f"   Falling back to default: zed_cameras_config.yaml")
        config_path = os.path.join(role_ros2_share_dir, 'config', 'zed_cameras_config.yaml')
    
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)
    
    return config.get('cameras', []), config_path


def create_camera_params_file(camera_config):
    """Create a temporary ROS2 parameters file for a camera."""
    # Create temporary file
    temp_file = tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False)
    
    # Convert camera config to ROS2 parameter format
    ros_params = {
        '/**': {
            'ros__parameters': {
                'general': {
                    'camera_name': camera_config['name'],
                    'camera_model': camera_config['camera_model'],
                    'serial_number': camera_config['serial_number'],
                    'grab_resolution': camera_config['grab_resolution'],
                    'grab_frame_rate': camera_config['grab_frame_rate'],
                    'pub_resolution': camera_config['pub_resolution'],
                    'pub_downscale_factor': camera_config['pub_downscale_factor'],
                    'pub_frame_rate': camera_config['pub_frame_rate'],
                },
                'sensors': camera_config['sensors'],
                'depth': camera_config['depth'],
                'pos_tracking': camera_config['pos_tracking'],
                'video': camera_config['video'],
            }
        }
    }
    
    yaml.dump(ros_params, temp_file, default_flow_style=False, sort_keys=False)
    temp_file.close()
    
    return temp_file.name


def generate_camera_launches(context):
    """Generate launch descriptions for cameras based on configuration."""
    # Get the package share directories
    zed_wrapper_share_dir = get_package_share_directory('zed_wrapper')
    role_ros2_share_dir = get_package_share_directory('role_ros2')
    
    # Get config file from launch argument
    config_file = context.perform_substitution(LaunchConfiguration('config_file'))
    
    # Load camera configuration
    cameras, config_path = load_camera_config(config_file)
    
    # Display configuration info
    print("\n" + "="*70)
    print("ZED Cameras Configuration")
    print("="*70)
    print(f"Config file: {config_path}")
    print("-"*70)
    for camera in cameras:
        status = "✅ ACTIVE" if camera.get('activate', False) else "❌ INACTIVE"
        print(f"  {status} - {camera['name']:20s} (Model: {camera['camera_model']:6s}, Serial: {camera['serial_number']})")
    print("="*70 + "\n")
    
    # Generate launch descriptions for activated cameras
    launches = []
    
    for camera in cameras:
        if not camera.get('activate', False):
            print(f"⏭️  Skipping camera '{camera['name']}' (activate=false)")
            continue
        
        print(f"✅ Activating camera '{camera['name']}' (model: {camera['camera_model']}, serial: {camera['serial_number']})")
        print(f"   📋 Serial number will be used to select the correct camera device")
        
        # Create temporary parameters file
        params_file = create_camera_params_file(camera)
        
        # Create launch description for this camera
        # camera_name is used as frame_id prefix (e.g., hand_camera_camera_center, static_camera_camera_center)
        # serial_number must be passed as launch argument to ensure correct camera selection
        camera_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(zed_wrapper_share_dir, 'launch', 'zed_camera.launch.py')
            ]),
            launch_arguments={
                'camera_name': camera['name'],  # Used as frame_id prefix
                'camera_model': camera['camera_model'],
                'namespace': camera['namespace'],
                'serial_number': str(camera['serial_number']),  # Pass serial_number as launch argument (required for multi-camera setup)
                'ros_params_override_path': params_file,
                'node_name': camera['node_name'],
            }.items()
        )
        
        launches.append(camera_launch)
    
    if not launches:
        print("⚠️  Warning: No cameras activated! Check 'activate' flags in config file.")
    
    return launches


def generate_launch_description():
    # Declare launch argument for config file selection
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=TextSubstitution(text='zed_cameras_config.yaml'),
        description='Configuration file name (e.g., zed_cameras_config.yaml). '
                   'File must be in role_ros2/config/ directory. '
                   'Note: ZED camera requires NVIDIA GPU and CUDA to run.'
    )
    
    return LaunchDescription([
        config_file_arg,
        OpaqueFunction(function=generate_camera_launches),
    ])
