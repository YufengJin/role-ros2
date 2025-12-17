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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'publish_rate',
            default_value='50.0',
            description='Publishing rate in Hz (default: 50.0)'
        ),
        DeclareLaunchArgument(
            'publish_tf',
            default_value='true',
            description='Publish TF transforms for controllers (default: true)'
        ),
        DeclareLaunchArgument(
            'oculus_ip_address',
            default_value='',
            description='Oculus Quest IP address (empty for USB connection)'
        ),
        DeclareLaunchArgument(
            'oculus_port',
            default_value='5555',
            description='ADB port for network connection (default: 5555)'
        ),
        DeclareLaunchArgument(
            'frame_id',
            default_value='oculus_base',
            description='Frame ID for published poses and TF (default: oculus_base)'
        ),
        DeclareLaunchArgument(
            'publish_markers',
            default_value='false',
            description='Publish visualization markers for RViz (default: false)'
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

