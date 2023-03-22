
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    share_directory = get_package_share_directory('ros2_mqtt_client')

    start = os.path.join(
        share_directory,
        'dist',
        'overall.js'
    )

    start_javascript_node = Node(
        executable='node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            start
        ],
        cwd=share_directory)

    ld = LaunchDescription()
    ld.add_action(start_javascript_node)

    return ld
