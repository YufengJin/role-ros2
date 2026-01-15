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


def generate_camera_launch(context):
    """Generate launch description for a single camera based on configuration."""
    # Get the package share directories
    realsense_camera_share_dir = get_package_share_directory('realsense2_camera')
    
    # Get config file from launch argument
    config_file_arg = context.perform_substitution(LaunchConfiguration('config_file'))
    
    if not config_file_arg or config_file_arg == '':
        raise ValueError(
            f"❌ No config file specified!\n"
            f"   Use: config_file:=<your_config.yaml>\n"
            f"   Example: config_file:=hand_realsense_high_res.yaml"
        )
    
    # Load camera configuration
    camera_config, config_path = load_camera_config(config_file_arg)
    
    # Display configuration info
    print("\n" + "="*70)
    print("RealSense Camera Configuration")
    print("="*70)
    print(f"Config file: {config_path}")
    print("-"*70)
    camera_name = camera_config.get('name', 'N/A')
    serial = camera_config.get('serial_number', 'N/A')
    print(f"  Camera: {camera_name:20s} (Serial: {serial})")
    print("="*70 + "\n")
    
    # Display resolution info if available
    if 'rgb_camera' in camera_config and 'color_profile' in camera_config['rgb_camera']:
        profile = camera_config['rgb_camera']['color_profile']
        print(f"✅ Activating camera '{camera_name}' (serial: {serial})")
        print(f"   📋 Serial number will be used to select the correct camera device")
        print(f"   🔧 RGB Profile: {profile}")
    if 'depth_module' in camera_config and 'depth_profile' in camera_config['depth_module']:
        profile = camera_config['depth_module']['depth_profile']
        print(f"   📊 Depth Profile: {profile}")
    
    # Create temporary parameters file
    params_file = create_camera_params_file(camera_config)
    
    # Create launch description for this camera
    # Use IncludeLaunchDescription to include rs_launch.py
    # Note: ROS2 has a known issue where numeric strings are auto-converted to integers
    # RealSense workaround: prefix serial_no with '_' to prevent type conversion
    # The node will automatically remove the '_' prefix (see realsense_node_factory.cpp:337)
    serial_no_str = str(camera_config['serial_number'])  # Ensure string type
    if serial_no_str and serial_no_str[0] != '_':
        serial_no_str = '_' + serial_no_str  # Add underscore prefix to prevent integer conversion
    
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(realsense_camera_share_dir, 'launch', 'rs_launch.py')
        ]),
        launch_arguments={
            'camera_name': camera_config['name'],  # Used for frame_id prefix
            'camera_namespace': camera_config['namespace'],
            'serial_no': serial_no_str,  # Pass serial_number with '_' prefix to prevent integer conversion
            'config_file': params_file,  # Use generated params file
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
        description='Configuration file name (e.g., hand_realsense_high_res.yaml). '
                   'File must be in role_ros2/config/cameras/ directory. '
                   'Note: RealSense camera requires USB connection.'
    )
    
    return LaunchDescription([
        config_file_arg,
        OpaqueFunction(function=generate_camera_launch),
    ])
