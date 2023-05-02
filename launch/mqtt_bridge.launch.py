# Copyright [2023] [wavem-reidlo]
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

# function for ROS2 launch rclnodejs application
# @author wavem-reidlo
# @version 1.0.0
# @since 2023.03.31
def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    share_directory = get_package_share_directory('rclnodejs_mqtt_client')

    start = os.path.join(
        share_directory,
        'dist/ros2/launch',
        'main.launch.js'
    )

    start_javascript_node = Node(
        executable='node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[start],
        cwd=share_directory)

    ld = LaunchDescription()
    ld.add_action(start_javascript_node)

    return ld