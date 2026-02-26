#!/usr/bin/env python3


from ament_index_python.packages import get_package_share_directory
from edge_rtc.rtc_ros2_server import RtcRos2Server
from edge_rtc.utils import EdgeRTCConfig
import rclpy
from rclpy.executors import MultiThreadedExecutor
import threading
import yaml


def main(args=None):
    """Main function to run the RtcRos2Server."""
    rclpy.init(args=args)
    pkg_path = get_package_share_directory("edge_rtc")
    config_file = f"{pkg_path}/config/server.yaml"
    with open(config_file, "r") as f:
        config_data = yaml.safe_load(f)
    # if not isinstance(config_data, dict):
    #     config_data = {}
    if isinstance(config_data, dict):
        config = EdgeRTCConfig(**config_data)
    else:
        raise ValueError(f"Invalid configuration format in {config_file}")
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
