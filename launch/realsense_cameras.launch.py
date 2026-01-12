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
    """Create a temporary ROS2 parameters file for a camera.
    
    RealSense expects a flat YAML format (no /** or ros__parameters wrapper).
    The format matches examples/launch_params_from_file/config/config.yaml
    """
    # Create temporary file
    temp_file = tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False)
    
    # Convert camera config to RealSense flat parameter format
    # RealSense parameters use flattened keys (e.g., rgb_camera.color_profile)
    # No /** or ros__parameters wrapper needed - just flat key-value pairs
    ros_params = {}
    
    # Add all parameters from camera_config (excluding metadata fields)
    metadata_fields = ['name', 'activate', 'serial_number', 'namespace']
    for key, value in camera_config.items():
        if key not in metadata_fields:
            if isinstance(value, dict):
                # Nested parameters (e.g., rgb_camera, depth_module, pointcloud, align_depth, etc.)
                for sub_key, sub_value in value.items():
                    param_name = f'{key}.{sub_key}'
                    ros_params[param_name] = sub_value
            else:
                # Direct parameters (e.g., enable_color, enable_depth, publish_tf, etc.)
                ros_params[key] = value
    
    # Write flat YAML format (no /** or ros__parameters wrapper)
    yaml.dump(ros_params, temp_file, default_flow_style=False, sort_keys=False, allow_unicode=True)
    temp_file.close()
    
    # Debug: Print the generated params file path and key parameters
    rgb_profile = camera_config.get('rgb_camera', {}).get('color_profile', 'N/A')
    depth_profile = camera_config.get('depth_module', {}).get('depth_profile', 'N/A')
    print(f"   📄 Generated params file: {temp_file.name}")
    print(f"   ✅ Params: RGB profile='{rgb_profile}', Depth profile='{depth_profile}'")
    
    return temp_file.name


def generate_camera_launches(context):
    """Generate launch descriptions for cameras based on configuration."""
    # Get the package share directories
    realsense_camera_share_dir = get_package_share_directory('realsense2_camera')
    role_ros2_share_dir = get_package_share_directory('role_ros2')
    
    # Get mode from launch argument
    mode = context.perform_substitution(LaunchConfiguration('mode'))
    
    # Get config file from launch argument
    config_file_arg = context.perform_substitution(LaunchConfiguration('config_file'))
    
    # Determine config file based on mode (mode takes priority over config_file)
    if mode == 'calibration':
        # High resolution mode for calibration: HD (1280x720) at 30Hz
        config_file = 'realsense_cameras_config_high_res.yaml'
    elif mode == 'learning':
        # Low resolution mode for robot learning: VGA (640x480) at 30Hz
        config_file = 'realsense_cameras_config_low_res.yaml'
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
    print("RealSense Cameras Configuration")
    print("="*70)
    print(f"Mode: {mode}")
    print(f"Config file: {config_path}")
    print("-"*70)
    for camera in cameras:
        status = "✅ ACTIVE" if camera.get('activate', False) else "❌ INACTIVE"
        serial = camera.get('serial_number', 'N/A')
        print(f"  {status} - {camera['name']:20s} (Serial: {serial})")
    print("="*70 + "\n")
    
    # Generate launch descriptions for activated cameras
    launches = []
    camera_index = 0  # Track index of activated cameras
    
    for camera in cameras:
        if not camera.get('activate', False):
            print(f"⏭️  Skipping camera '{camera['name']}' (activate=false)")
            continue
        
        print(f"✅ Activating camera '{camera['name']}' (serial: {camera['serial_number']})")
        print(f"   📋 Serial number will be used to select the correct camera device")
        
        # Display resolution info if available
        if 'rgb_camera' in camera and 'color_profile' in camera['rgb_camera']:
            profile = camera['rgb_camera']['color_profile']
            print(f"   🔧 RGB Profile: {profile}")
        if 'depth_module' in camera and 'depth_profile' in camera['depth_module']:
            profile = camera['depth_module']['depth_profile']
            print(f"   📊 Depth Profile: {profile}")
        
        # Create temporary parameters file
        params_file = create_camera_params_file(camera)
        
        # Create launch description for this camera
        # Use IncludeLaunchDescription to include rs_launch.py
        # Note: ROS2 has a known issue where numeric strings are auto-converted to integers
        # RealSense workaround: prefix serial_no with '_' to prevent type conversion
        # The node will automatically remove the '_' prefix (see realsense_node_factory.cpp:337)
        serial_no_str = str(camera['serial_number'])  # Ensure string type
        if serial_no_str and serial_no_str[0] != '_':
            serial_no_str = '_' + serial_no_str  # Add underscore prefix to prevent integer conversion
        camera_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(realsense_camera_share_dir, 'launch', 'rs_launch.py')
            ]),
            launch_arguments={
                'camera_name': camera['name'],  # Used for frame_id prefix
                'camera_namespace': camera['namespace'],
                'serial_no': serial_no_str,  # Pass serial_number with '_' prefix to prevent integer conversion
                'config_file': params_file,  # Use generated params file
            }.items()
        )
        
        # Add delay for second and subsequent cameras to avoid device enumeration conflicts
        # This helps when multiple RealSense cameras are connected simultaneously
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
                   'Options: "calibration" (HD 1280x720 @ 30Hz), "learning" (VGA 640x480 @ 30Hz), or "default" (uses config_file).'
    )
    
    # Declare launch argument for config file selection (used when mode='default')
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=TextSubstitution(text=''),
        description='Configuration file name (e.g., realsense_cameras_config_low_res.yaml). '
                   'File must be in role_ros2/config/ directory. '
                   'Used when mode is "default" or empty. '
                   'If mode is "calibration" or "learning", this parameter is ignored. '
                   'Note: RealSense camera requires USB connection.'
    )
    
    return LaunchDescription([
        mode_arg,
        config_file_arg,
        OpaqueFunction(function=generate_camera_launches),
    ])
