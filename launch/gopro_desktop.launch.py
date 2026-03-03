"""Launch visualization for GoPro cameras on a desktop PC.

Assumes the gopro_camera_node is already running on the Jetson and
publishing image topics over the network. Launches RViz and two
rqt_image_view windows for quick inspection.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    rviz_config = os.path.join(
        get_package_share_directory('gopro_ros2'),
        'rviz',
        'gopro_cameras.rviz',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
    )

    image_view_front = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='image_view_front',
        arguments=['/gopro/camera_0/image_raw'],
    )

    image_view_back = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='image_view_back',
        arguments=['/gopro/camera_1/image_raw'],
    )

    return LaunchDescription([
        rviz_node,
        image_view_front,
        image_view_back,
    ])
