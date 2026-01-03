import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('edge_rtc'),
        'config',
        'image_topics.yaml'
    )
    
    return LaunchDescription([
        Node(
            package="edge_rtc",
            executable="webrtc_ros2_client",
            name="webrtc_ros2_client",
            output="screen",
            parameters=[config_file]
        )
    ])
