#!/usr/bin/env python3

"""Utils for Edge RTC."""

from dataclasses import dataclass
import time
from rclpy.node import Node
import queue
import threading
from rclpy.publisher import Publisher
import cv2
import numpy as np
from cv_bridge import CvBridge
import logging


logger = logging.getLogger(__name__)

@dataclass
class EdgeRTCConfig:
    video_device: str
    framerate: int
    resolution: str
    bitrate: int
    host: str = "0.0.0.0"
    port: int = 8080


@dataclass
class ImageType:
    COLOR = 0
    DEPTH = 1

class TopicHandler:
    """Handles a single topic's WebRTC connection and ROS2 publishing."""

    def __init__(
        self, topic_name: str, publisher: Publisher, bridge: CvBridge, node: Node
    ):
        self.topic_name = topic_name
        self.publisher = publisher
        self.bridge = bridge
        self.node = node
        self.frame_queue = queue.Queue(maxsize=2)
        self.running = True
        self.frame_count = 0
        self.last_time = time.time()

        # Start publishing thread for this topic
        self.pub_thread = threading.Thread(
            target=self._publish_loop,
            args=(
                ImageType.DEPTH if "depth" in topic_name.lower() else ImageType.COLOR,
            ),
            daemon=True,
            name=f"pub_thread_{topic_name}",
        )
        self.pub_thread.start()
        logger.info(f"TopicHandler initialized for {topic_name}")

    def _publish_loop(self, image_type: ImageType):
        """Continuously publish frames from queue for this topic."""
        while self.running:
            try:
                img = self.frame_queue.get(timeout=1.0)
                # Convert incoming image to single-channel 16-bit (16UC1)
                if img is None:
                    continue

                if image_type == ImageType.DEPTH:
                    # If input is color, convert to grayscale first
                    if hasattr(img, "ndim") and img.ndim == 3 and img.shape[2] == 3:
                        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                    elif hasattr(img, "ndim") and img.ndim == 2:
                        gray = img
                    else:
                        logger.error(
                            f"[{self.topic_name}] Unexpected image shape: {getattr(img, 'shape', None)}"
                        )
                        continue

                    # Convert to uint16. Map 0-255 -> 0-65535 using multiplier 257 for even spread.
                    if gray.dtype != np.uint16:
                        gray16 = gray.astype(np.uint16) * 257
                    else:
                        gray16 = gray

                    ros_img = self.bridge.cv2_to_imgmsg(gray16, encoding="16UC1")
                else:
                    ros_img = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")

                ros_img.header.stamp = self.node.get_clock().now().to_msg()
                ros_img.header.frame_id = self.topic_name
                self.publisher.publish(ros_img)
                self.frame_count += 1

                # Log FPS
                current_time = time.time()
                if self.frame_count % 30 == 0:
                    fps = 30 / (current_time - self.last_time)
                    logger.debug(f"[{self.topic_name}] Publishing at {fps:.1f} Hz")
                    self.last_time = current_time
            except queue.Empty:
                continue
            except Exception as e:
                logger.error(f"[{self.topic_name}] Error in publish loop: {e}")

    def queue_frame(self, img):
        """Queue frame for publishing (non-blocking)."""
        try:
            self.frame_queue.put_nowait(img)
        except queue.Full:
            # Drop frame if queue is full to maintain real-time performance
            pass

    def stop(self):
        """Stop the handler."""
        self.running = False