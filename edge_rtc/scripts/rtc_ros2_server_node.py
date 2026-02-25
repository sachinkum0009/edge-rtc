#!/usr/bin/env python3

from edge_rtc.rtc_ros2_server import RtcRos2Server
from edge_rtc.utils import EdgeRTCConfig
import rclpy
from rclpy.executors import MultiThreadedExecutor
import threading

def main(args=None):
    """Main function to run the RtcRos2Server."""
    rclpy.init(args=args)
    config = EdgeRTCConfig(
        video_device="/dev/video2",
        framerate=30,
        resolution="640x480",
        bitrate=5000000,
        host="0.0.0.0",
        port=8081,
    )
    image_topics = ["/camera/color", "/camera/depth"]
    server = RtcRos2Server(config, image_topics)
    executor = MultiThreadedExecutor()
    executor.add_node(server)

    # Start ROS2 spin in a separate thread
    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()

    try:
        # Run the web server (blocking)
        server.run()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        server.destroy_node()
        rclpy.shutdown()
        ros_thread.join(timeout=2.0)

if __name__ == "__main__":
    main()