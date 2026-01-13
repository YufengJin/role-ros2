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

# Enable colored output
os.environ["RCUTILS_COLORIZED_OUTPUT"] = "1"


def load_camera_config(config_file):
    """Load camera configuration from YAML file."""
    role_ros2_share_dir = get_package_share_directory('role_ros2')
    config_path = os.path.join(role_ros2_share_dir, 'config', config_file)
    
    # Also try source directory if not found in install directory
    if not os.path.exists(config_path):
        # Try source directory
        role_ros2_src_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
        src_config_path = os.path.join(role_ros2_src_dir, 'config', config_file)
        if os.path.exists(src_config_path):
            config_path = src_config_path
            print(f"📋 Using config from source directory: {config_path}")
        else:
            raise FileNotFoundError(
                f"❌ Config file not found: {config_file}\n"
                f"   Tried:\n"
                f"   - {config_path}\n"
                f"   - {src_config_path}\n"
                f"   Please ensure the config file exists and the package is installed."
            )
    
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)
    
    return config.get('cameras', []), config_path


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


def generate_camera_launches(context):
    """Generate launch descriptions for cameras based on configuration."""
    # Get the package share directories
    zed_wrapper_share_dir = get_package_share_directory('zed_wrapper')
    role_ros2_share_dir = get_package_share_directory('role_ros2')
    
    # Get mode from launch argument
    mode = context.perform_substitution(LaunchConfiguration('mode'))
    
    # Get config file from launch argument
    config_file_arg = context.perform_substitution(LaunchConfiguration('config_file'))
    
    # Determine config file based on mode (mode takes priority over config_file)
    if mode == 'calibration':
        # High resolution mode for calibration: HD1080 at 15Hz
        config_file = 'zed_cameras_config_high_res.yaml'
    elif mode == 'learning':
        # Low resolution mode for robot learning: HD720/2 at 30Hz
        config_file = 'zed_cameras_config_low_res.yaml'
    else:
        # Use explicit config file if provided
        if config_file_arg and config_file_arg != '':
            config_file = config_file_arg
        else:
            raise ValueError(
                f"❌ No config file specified!\n"
                f"   Mode is '{mode}' but no config_file provided.\n"
                f"   Use: mode:=calibration OR mode:=learning OR config_file:=<your_config.yaml>"
            )
    
    # Load camera configuration
    cameras, config_path = load_camera_config(config_file)
    
    # Display configuration info
    print("\n" + "="*70)
    print("ZED Cameras Configuration")
    print("="*70)
    print(f"Mode: {mode}")
    print(f"Config file: {config_path}")
    print("-"*70)
    for camera in cameras:
        status = "✅ ACTIVE" if camera.get('activate', False) else "❌ INACTIVE"
        print(f"  {status} - {camera['name']:20s} (Model: {camera['camera_model']:6s}, Serial: {camera['serial_number']})")
    print("="*70 + "\n")
    
    # Generate launch descriptions for activated cameras
    launches = []
    camera_index = 0  # Track index of activated cameras
    
    for camera in cameras:
        if not camera.get('activate', False):
            print(f"⏭️  Skipping camera '{camera['name']}' (activate=false)")
            continue
        
        print(f"✅ Activating camera '{camera['name']}' (model: {camera['camera_model']}, serial: {camera['serial_number']})")
        print(f"   📋 Serial number will be used to select the correct camera device")
        print(f"   🔧 Resolution: {camera['grab_resolution']} @ {camera['grab_frame_rate']}Hz")
        print(f"   📊 Publish: {camera['pub_resolution']} (downscale: {camera['pub_downscale_factor']}x) @ {camera['pub_frame_rate']}Hz")
        
        # Create temporary parameters file
        params_file = create_camera_params_file(camera)
        
        # Extract pos_tracking parameters for launch arguments
        # Note: zed_wrapper launch file uses launch arguments to override YAML parameters
        # Launch arguments have higher priority than ros_params_override_path
        pos_tracking = camera.get('pos_tracking', {})
        publish_tf = 'true' if pos_tracking.get('publish_tf', False) else 'false'
        publish_map_tf = 'true' if pos_tracking.get('publish_map_tf', False) else 'false'
        
        # Extract sensors parameters for launch arguments
        sensors = camera.get('sensors', {})
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
                'camera_name': camera['name'],  # Used as frame_id prefix
                'camera_model': camera['camera_model'],
                'namespace': camera['namespace'],
                'serial_number': str(camera['serial_number']),  # Pass serial_number as launch argument (required for multi-camera setup)
                'ros_params_override_path': params_file,
                'node_name': camera['node_name'],
                # Pass pos_tracking parameters as launch arguments (highest priority)
                'publish_tf': publish_tf,  # Overrides default 'true' in zed_wrapper launch
                'publish_map_tf': publish_map_tf,  # Overrides default 'true' in zed_wrapper launch
                'publish_imu_tf': publish_imu_tf,  # Overrides default 'false' in zed_wrapper launch
            }.items()
        )
        
        # Add delay for second and subsequent cameras to avoid device enumeration conflicts
        # This helps when multiple ZED cameras are connected simultaneously
        if camera_index > 0:
            delay_seconds = 3.0  # 3 second delay for subsequent cameras
            print(f"   ⏱️  Delaying launch by {delay_seconds} seconds to avoid device conflicts")
            delayed_launch = TimerAction(
                period=delay_seconds,
                actions=[camera_launch]
            )
            launches.append(delayed_launch)
        else:
            launches.append(camera_launch)
        
        camera_index += 1
    
    if not launches:
        print("⚠️  Warning: No cameras activated! Check 'activate' flags in config file.")
    
    return launches


def generate_launch_description():
    # Declare launch argument for mode selection
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value=TextSubstitution(text='default'),
        description='Camera configuration mode. '
                   'Options: "calibration" (HD1080 @ 15Hz), "learning" (HD720/2 @ 30Hz), or "default" (uses config_file).'
    )
    
    # Declare launch argument for config file selection (used when mode='default')
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=TextSubstitution(text=''),
        description='Configuration file name (e.g., zed_cameras_config_low_res.yaml). '
                   'File must be in role_ros2/config/ directory. '
                   'Used when mode is "default" or empty. '
                   'If mode is "calibration" or "learning", this parameter is ignored. '
                   'Note: ZED camera requires NVIDIA GPU and CUDA to run.'
    )
    
    return LaunchDescription([
        mode_arg,
        config_file_arg,
        OpaqueFunction(function=generate_camera_launches),
    ])
