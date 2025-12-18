import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()

    image_params = os.path.join(
        get_package_share_directory("edge_rtc"), "config", "image_topics.yaml"
    )
    edge_rtc_node = Node(
        package="edge_rtc",
        executable="webrtc_video_server",
        name="webrtc_video_server",
        output="screen",
        parameters=[image_params],
    )
    ld.add_action(edge_rtc_node)
    return ld
