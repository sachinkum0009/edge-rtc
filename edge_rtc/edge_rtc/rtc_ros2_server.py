"""
RTC ROS2 Server
"""

import cv2
from cv_bridge import CvBridge, CvBridgeError
from edge_rtc.utils import EdgeRTCConfig
import numpy as np
import os
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rtc_server import RtcServer
from sensor_msgs.msg import Image
import threading


class RtcRos2Server(Node, RtcServer):
    """ROS2 Node that integrates the RTC Server functionality."""

    def __init__(self, config: EdgeRTCConfig, image_topics: list[str]):
        """Initialize the RtcRos2Server with the given configuration and image topics."""
        Node.__init__(self, "rtc_ros2_server")
        RtcServer.__init__(self, config, image_topics)
        self.callback_group = ReentrantCallbackGroup()
        self.get_logger().info("RtcRos2Server initialized with ROS2 integration")

        self.callback_group = ReentrantCallbackGroup()
        self.bridge = CvBridge()

        self.lock = threading.Lock()

        # Create placeholder image (640x480 black image with text)
        self.placeholder_image = self.create_placeholder_image()

        self.image_sub = self.create_subscription(
            Image,
            self.image_topics[0],  # Assuming one topic for simplicity
            self.image_callback,
            10,
            callback_group=self.callback_group,
        )

    def create_placeholder_image(self):
        """Create a placeholder image when no data is available."""
        img = (
            cv2.imread("1341848067.862836.png") if os.path.exists("1341848067.862836.png") else None
            # cv2.imread("1341848067.862808.png") if os.path.exists("1341848067.862808.png") else None
        )
        print(f"type of placeholder image: {type(img)}, shape: {img.shape if img is not None else 'N/A'}")
        if img is None:
            # Create black image with text
            img = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(
                img,
                "Waiting for ROS2 images...",
                (50, 240),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (255, 255, 255),
                2,
            )
        return img

    def image_callback(self, msg: Image):
        """Callback function for incoming ROS2 image messages."""
        # try:
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        with self.lock:
            self.latest_images[self.image_topics[0]] = cv_image
        self.get_logger().debug(f"Received image on topic {self.image_topics[0]}")
        # except CvBridgeError as e:
        #     self.get_logger().error(f"Failed to convert ROS Image message: {e}")

    def get_latest_image(self, topic_name: str):
        """Returns the latest processed image for a topic or a placeholder if none available."""
        with self.lock:
            image = self.latest_images.get(topic_name)
            if image is not None:
                return image
            self.get_logger().debug(
                f"No image available for topic {topic_name}, using placeholder"
            )
            return self.placeholder_image

    def get_available_topics(self):
        """Returns list of available image topics."""
        return self.image_topics


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
    image_topics = ["/depth"]
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