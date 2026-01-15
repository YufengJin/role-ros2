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
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

# Enable colored output
os.environ["RCUTILS_COLORIZED_OUTPUT"] = "1"


def load_camera_config(config_file):
    """Load single camera configuration from YAML file.
    
    Only loads from install directory: <package_share>/config/cameras/<config_file>
    """
    role_ros2_share_dir = get_package_share_directory('role_ros2')
    config_path = os.path.join(role_ros2_share_dir, 'config', 'cameras', config_file)
    
    # Check if file exists
    if not os.path.exists(config_path):
        raise FileNotFoundError(
            f"❌ Config file not found: {config_file}\n"
            f"   Expected location: {config_path}\n"
            f"   Please ensure the config file exists in config/cameras/ directory and the package is installed."
        )
    
    print(f"📋 Using config from: {config_path}")
    
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)
    
    # Single camera config: return the camera dict directly (not a list)
    return config, config_path


def create_camera_params_file(camera_config):
    """
    Create a temporary ROS2 parameters file for a camera.
    
    This function converts the camera configuration dictionary into ROS2 parameter format
    that matches zed_wrapper's expected structure. The format follows:
    /**: ros__parameters: {group_name: {parameter_name: value}}
    
    All parameter groups from the config are included to ensure proper configuration.
    """
    # Create temporary file
    temp_file = tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False)
    
    # Start building ROS2 parameters dictionary
    ros_params_dict = {
        '/**': {
            'ros__parameters': {}
        }
    }
    
    # Extract general parameters (mapped from config to ROS2 parameter names)
    ros_params_dict['/**']['ros__parameters']['general'] = {
        'camera_name': str(camera_config['name']),
        'camera_model': str(camera_config['camera_model']),
        'serial_number': int(camera_config['serial_number']),
        'grab_resolution': str(camera_config['grab_resolution']),
        'grab_frame_rate': int(camera_config['grab_frame_rate']),
        'pub_resolution': str(camera_config['pub_resolution']),
        'pub_downscale_factor': float(camera_config['pub_downscale_factor']),
        'pub_frame_rate': float(camera_config['pub_frame_rate']),
    }
    
    # Required parameter groups (must exist in config)
    required_groups = ['sensors', 'depth', 'pos_tracking', 'video']
    
    # Optional parameter groups (may or may not exist)
    optional_groups = ['object_detection', 'body_tracking', 'advanced']
    
    # Add required parameter groups (with validation)
    for group_name in required_groups:
        if group_name not in camera_config:
            raise ValueError(
                f"❌ Missing required parameter group '{group_name}' in camera config for '{camera_config['name']}'"
            )
        ros_params_dict['/**']['ros__parameters'][group_name] = camera_config[group_name]
    
    # Add optional parameter groups if they exist
    for group_name in optional_groups:
        if group_name in camera_config:
            ros_params_dict['/**']['ros__parameters'][group_name] = camera_config[group_name]
    
    # Add any other parameter groups that might exist in the config
    # (excluding launch arguments which are not ROS parameters)
    excluded_keys = {
        'name', 'activate', 'camera_model', 'serial_number', 'namespace', 'node_name',
        'grab_resolution', 'grab_frame_rate', 'pub_resolution', 'pub_downscale_factor', 'pub_frame_rate'
    }
    all_known_groups = set(required_groups + optional_groups)
    
    for key, value in camera_config.items():
        if key not in excluded_keys and key not in all_known_groups:
            ros_params_dict['/**']['ros__parameters'][key] = value
    
    # Write parameters to temporary file
    yaml.dump(ros_params_dict, temp_file, default_flow_style=False, sort_keys=False, allow_unicode=True)
    temp_file.close()
    
    # Debug: Print key information
    print(f"   📄 Generated params file: {temp_file.name}")
    print(f"   ✅ Params: pub_resolution='{ros_params_dict['/**']['ros__parameters']['general']['pub_resolution']}', "
          f"pub_downscale_factor={ros_params_dict['/**']['ros__parameters']['general']['pub_downscale_factor']}")
    
    # Debug: Print pos_tracking parameters to verify
    if 'pos_tracking' in ros_params_dict['/**']['ros__parameters']:
        pos_tracking = ros_params_dict['/**']['ros__parameters']['pos_tracking']
        publish_tf = pos_tracking.get('publish_tf', 'NOT SET')
        publish_map_tf = pos_tracking.get('publish_map_tf', 'NOT SET')
        pos_tracking_enabled = pos_tracking.get('pos_tracking_enabled', 'NOT SET')
        print(f"   🔍 pos_tracking: publish_tf={publish_tf}, publish_map_tf={publish_map_tf}, "
              f"pos_tracking_enabled={pos_tracking_enabled}")
    
    # Debug: List all parameter groups included
    included_groups = list(ros_params_dict['/**']['ros__parameters'].keys())
    print(f"   📋 Parameter groups included: {', '.join(included_groups)}")
    
    return temp_file.name


def generate_camera_launch(context):
    """Generate launch description for a single camera based on configuration."""
    # Get the package share directories
    zed_wrapper_share_dir = get_package_share_directory('zed_wrapper')
    
    # Get config file from launch argument
    config_file_arg = context.perform_substitution(LaunchConfiguration('config_file'))
    
    if not config_file_arg or config_file_arg == '':
        raise ValueError(
            f"❌ No config file specified!\n"
            f"   Use: config_file:=<your_config.yaml>\n"
            f"   Example: config_file:=hand_zed_high_res.yaml"
        )
    
    # Load camera configuration
    camera_config, config_path = load_camera_config(config_file_arg)
    
    # Display configuration info
    print("\n" + "="*70)
    print("ZED Camera Configuration")
    print("="*70)
    print(f"Config file: {config_path}")
    print("-"*70)
    camera_name = camera_config.get('name', 'N/A')
    camera_model = camera_config.get('camera_model', 'N/A')
    serial = camera_config.get('serial_number', 'N/A')
    print(f"  Camera: {camera_name:20s} (Model: {camera_model:6s}, Serial: {serial})")
    print("="*70 + "\n")
    
    print(f"✅ Activating camera '{camera_name}' (model: {camera_model}, serial: {serial})")
    print(f"   📋 Serial number will be used to select the correct camera device")
    print(f"   🔧 Resolution: {camera_config['grab_resolution']} @ {camera_config['grab_frame_rate']}Hz")
    print(f"   📊 Publish: {camera_config['pub_resolution']} (downscale: {camera_config['pub_downscale_factor']}x) @ {camera_config['pub_frame_rate']}Hz")
    
    # Create temporary parameters file
    params_file = create_camera_params_file(camera_config)
    
    # Extract pos_tracking parameters for launch arguments
    # Note: zed_wrapper launch file uses launch arguments to override YAML parameters
    # Launch arguments have higher priority than ros_params_override_path
    pos_tracking = camera_config.get('pos_tracking', {})
    publish_tf = 'true' if pos_tracking.get('publish_tf', False) else 'false'
    publish_map_tf = 'true' if pos_tracking.get('publish_map_tf', False) else 'false'
    
    # Extract sensors parameters for launch arguments
    sensors = camera_config.get('sensors', {})
    publish_imu_tf = 'true' if sensors.get('publish_imu_tf', False) else 'false'
    
    # Create launch description for this camera
    # camera_name is used as frame_id prefix (e.g., hand_camera_camera_center, static_camera_camera_center)
    # serial_number must be passed as launch argument to ensure correct camera selection
    # publish_tf and publish_map_tf are passed as launch arguments to override defaults
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(zed_wrapper_share_dir, 'launch', 'zed_camera.launch.py')
        ]),
        launch_arguments={
            'camera_name': camera_config['name'],  # Used as frame_id prefix
            'camera_model': camera_config['camera_model'],
            'namespace': camera_config['namespace'],
            'serial_number': str(camera_config['serial_number']),  # Pass serial_number as launch argument (required for multi-camera setup)
            'ros_params_override_path': params_file,
            'node_name': camera_config['node_name'],
            # Pass pos_tracking parameters as launch arguments (highest priority)
            'publish_tf': publish_tf,  # Overrides default 'true' in zed_wrapper launch
            'publish_map_tf': publish_map_tf,  # Overrides default 'true' in zed_wrapper launch
            'publish_imu_tf': publish_imu_tf,  # Overrides default 'false' in zed_wrapper launch
        }.items()
    )
    
    # Create calibration TF publisher node
    # This node checks if the camera is calibrated and publishes static TF if found
    # Delay by 2 seconds to allow camera to start first
    calibration_tf_node = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='role_ros2',
                executable='camera_calibration_tf_publisher',
                name=f'camera_calibration_tf_publisher_{camera_config["name"]}',
                output='screen',
                arguments=[str(camera_config['serial_number'])],
            )
        ]
    )
    
    return [camera_launch, calibration_tf_node]


def generate_launch_description():
    # Declare launch argument for config file selection
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=TextSubstitution(text=''),
        description='Configuration file name (e.g., hand_zed_high_res.yaml). '
                   'File must be in role_ros2/config/cameras/ directory. '
                   'Note: ZED camera requires NVIDIA GPU and CUDA to run.'
    )
    
    return LaunchDescription([
        config_file_arg,
        OpaqueFunction(function=generate_camera_launch),
    ])
