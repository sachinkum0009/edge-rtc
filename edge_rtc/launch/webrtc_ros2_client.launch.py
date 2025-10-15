from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='edge_rtc',
            executable='webrtc_ros2_client',
            name='webrtc_ros2_client',
            output='screen'
        )
    ])