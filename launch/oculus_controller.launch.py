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
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def load_config_yaml(package_name, config_file):
    """
    Load launch configuration YAML file.
    
    This is a plain YAML file for launch configuration, NOT a ROS2 node parameter file.
    The format is simple key-value pairs at the root level.
    
    Args:
        package_name: ROS2 package name
        config_file: Config file name (e.g., 'oculus_controller_config.yaml')
    
    Returns:
        dict: Configuration dictionary, or empty dict if file not found
    """
    try:
        package_share_dir = get_package_share_directory(package_name)
        config_path = os.path.join(package_share_dir, 'config', config_file)
        
        if os.path.exists(config_path):
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)
                return config if config else {}
    except Exception as e:
        print(f"Warning: Could not load config file {config_file}: {e}")
    
    return {}


def generate_launch_description():
    # Load default values from YAML config file
    config = load_config_yaml('role_ros2', 'oculus_reader_config.yaml')
    
    # Helper function to convert config values to strings for launch arguments
    def get_default(key, default_value):
        value = config.get(key, default_value)
        # Convert boolean to string
        if isinstance(value, bool):
            return 'true' if value else 'false'
        # Handle None values for optional parameters
        if value is None:
            return ''
        return str(value)
    
    return LaunchDescription([
        # Launch arguments (defaults from config file)
        DeclareLaunchArgument(
            'publish_rate',
            default_value=get_default('publish_rate', '50.0'),
            description='Publishing rate in Hz. Default from config/oculus_reader_config.yaml'
        ),
        DeclareLaunchArgument(
            'publish_tf',
            default_value=get_default('publish_tf', 'true'),
            description='Publish TF transforms for controllers. Default from config/oculus_reader_config.yaml'
        ),
        DeclareLaunchArgument(
            'oculus_ip_address',
            default_value=get_default('oculus_ip_address', ''),
            description='Oculus Quest IP address (empty for USB connection). Default from config/oculus_reader_config.yaml'
        ),
        DeclareLaunchArgument(
            'oculus_port',
            default_value=get_default('oculus_port', '5555'),
            description='ADB port for network connection. Default from config/oculus_reader_config.yaml'
        ),
        DeclareLaunchArgument(
            'frame_id',
            default_value=get_default('frame_id', 'oculus_base'),
            description='Frame ID for published poses and TF. Default from config/oculus_reader_config.yaml'
        ),
        DeclareLaunchArgument(
            'publish_markers',
            default_value=get_default('publish_markers', 'false'),
            description='Publish visualization markers for RViz. Default from config/oculus_reader_config.yaml'
        ),
        
        # Oculus Reader Node
        Node(
            package='role_ros2',
            executable='oculus_reader_node',
            name='oculus_reader_node',
            output='screen',
            parameters=[{
                'publish_rate': LaunchConfiguration('publish_rate'),
                'publish_tf': LaunchConfiguration('publish_tf'),
                'publish_markers': LaunchConfiguration('publish_markers'),
                'oculus_ip_address': LaunchConfiguration('oculus_ip_address'),
                'oculus_port': LaunchConfiguration('oculus_port'),
                'frame_id': LaunchConfiguration('frame_id'),
            }]
        ),
    ])

