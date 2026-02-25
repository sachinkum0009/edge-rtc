"""
RTC ROS2 Server
"""

import cv2
from cv_bridge import CvBridge, CvBridgeError
from edge_rtc.utils import EdgeRTCConfig
from edge_rtc.rtc_server import RtcServer
import numpy as np
from numpy.typing import NDArray
import os
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
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

        self.last_times = {}  # Track last update time per topic
        self.image_receive_count = {}  # Track number of images received per topic

        # Initialize storage for each topic
        for topic in self.image_topics:
            self.latest_images[topic] = None
            self.last_times[topic] = 0
            self.image_receive_count[topic] = 0

        for image_topic in self.image_topics:
            self.get_logger().info(f"Subscribing to: {image_topic}")
            self.create_subscription(
                Image,
                image_topic,
                lambda msg, t=image_topic: self.image_callback(msg, t),  # type: ignore
                10,
                callback_group=self.callback_group,
            )

    def image_callback(self, msg: Image, topic_name: str):
        """Callback function for incoming ROS2 image messages."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            with self.lock:
                self.latest_images[topic_name] = cv_image
                self.image_receive_count[topic_name] += 1
        except CvBridgeError as e:
            self.get_logger().error(
                f"CvBridge error processing image from {topic_name}: {e}"
            )
        except Exception as e:
            self.get_logger().error(f"Error processing image from {topic_name}: {e}")

    def create_placeholder_image(self):
        """Create a placeholder image when no data is available."""
        img = (
            cv2.imread("1341848067.862836.png")
            if os.path.exists("1341848067.862836.png")
            else None
            # cv2.imread("1341848067.862808.png") if os.path.exists("1341848067.862808.png") else None
        )
        print(
            f"type of placeholder image: {type(img)}, shape: {img.shape if img is not None else 'N/A'}"
        )
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

    def get_latest_image(self, topic_name: str) -> NDArray:
        """Returns the latest processed image for a topic or a placeholder if none available."""
        with self.lock:
            image = self.latest_images.get(topic_name)
            if image is not None:
                return image
            self.get_logger().debug(
                f"No image available for topic {topic_name}, using placeholder"
            )
            return self.placeholder_image

    def get_available_topics(self) -> list[str]:
        """Returns list of available image topics."""
        return self.image_topics
