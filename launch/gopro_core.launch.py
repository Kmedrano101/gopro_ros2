"""Launch core GoPro streaming (headless, no visualization)."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('gopro_ros2'),
        'config',
        'gopro_cameras.yaml',
    )

    gopro_node = Node(
        package='gopro_ros2',
        executable='gopro_camera_node',
        name='gopro_driver',
        parameters=[config_file],
        output='screen',
    )

    return LaunchDescription([gopro_node])
