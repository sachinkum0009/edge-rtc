#! /usr/bin/env python3

"""WebRTC Video Streamer Server for ROS2 Topics
Streams video from ROS2 image topic to connected WebRTC clients.
"""

import asyncio
import json
import os
import threading
import time
from typing import Optional, Set
from numpy.typing import NDArray

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
import yaml
from aiohttp import web
from aiortc import (
    RTCPeerConnection,
    RTCRtpSender,
    RTCSessionDescription,
)
from aiortc.contrib.media import MediaPlayer, MediaRelay
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import Image

from edge_rtc.utils import EdgeRTCConfig
from edge_rtc.image_video_track import ImageVideoTrack


class Ros2WebrtcServer(Node):
    """ROS2 Node to stream video from image topic via WebRTC."""

    pcs: Set[RTCPeerConnection] = set()
    relay: Optional[MediaRelay] = None
    video_source: Optional[MediaPlayer] = None
    latest_images: dict[str, Optional[NDArray]]

    def __init__(self):
        super().__init__("ros2_webrtc_server")
        self.declare_parameter(
            "image_topics",
            value=["image_raw"],
            descriptor=ParameterDescriptor(
                description="List of image topics to stream"
            ),
        )

        self.image_topics = list(
            self.get_parameter("image_topics").get_parameter_value().string_array_value
        )

        self.callback_group = ReentrantCallbackGroup()

        pkg_path = get_package_share_directory("edge_rtc")
        server_config = os.path.join(pkg_path, "config/server.yaml")
        with open(server_config) as f:
            config_data = yaml.safe_load(f)
        if config_data is None:
            config_data = {}
        self.config = EdgeRTCConfig(**config_data)
        self.bridge = CvBridge()

        # Image buffering
        self.lock = threading.Lock()
        self.latest_images = {}  # Dictionary to store latest image per topic
        self.last_times = {}  # Track last update time per topic
        self.image_receive_count = {}  # Track number of images received per topic
        self.fps = 30  # Target FPS for WebRTC streaming

        # Create placeholder image (640x480 black image with text)
        self.placeholder_image = self.create_placeholder_image()
        
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
                lambda msg, t=image_topic: self.image_callback(msg, t),
                10,
                callback_group=self.callback_group,
            )

        self.get_logger().info("ROS2 WebRTC Server Node Initialized")

    def create_placeholder_image(self):
        """Create a placeholder image when no data is available."""
        img = (
            cv2.imread("placeholder.jpg") if os.path.exists("placeholder.jpg") else None
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

    def image_callback(self, msg: Image, topic_name: str):
        """Callback to handle incoming image messages."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            
            # Ensure image is in BGR format for WebRTC
            if len(cv_image.shape) == 2:  # Grayscale
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)
            elif cv_image.shape[2] == 4:  # RGBA
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGBA2BGR)
            
            with self.lock:
                self.latest_images[topic_name] = cv_image
                self.image_receive_count[topic_name] += 1
                
            # Log periodically
            if self.image_receive_count[topic_name] % 30 == 0:
                self.get_logger().info(
                    f"Received {self.image_receive_count[topic_name]} images from {topic_name}, "
                    f"size: {cv_image.shape}"
                )
        except Exception as e:
            self.get_logger().error(f"Error processing image from {topic_name}: {e}")

    def get_latest_image(self, topic_name: str):
        """Returns the latest processed image for a topic or a placeholder if none available."""
        with self.lock:
            image = self.latest_images.get(topic_name)
            if image is not None:
                return image
            self.get_logger().debug(f"No image available for topic {topic_name}, using placeholder")
            return self.placeholder_image
    
    def get_available_topics(self):
        """Returns list of available image topics."""
        return self.image_topics

    # TODO(Sachin): Remove this later
    @staticmethod
    async def index(request: web.Request) -> web.Response:
        """Serve a simple HTML page for testing."""
        html_content = """
        <!DOCTYPE html>
        <html>
        <head>
            <title>WebRTC Video Server</title>
            <style>
                body {
                    font-family: Arial, sans-serif;
                    max-width: 800px;
                    margin: 50px auto;
                    padding: 20px;
                }
                h1 { color: #333; }
                .status {
                    padding: 10px;
                    background: #f0f0f0;
                    border-radius: 5px;
                    margin: 20px 0;
                }
                code {
                    background: #e0e0e0;
                    padding: 2px 6px;
                    border-radius: 3px;
                }
            </style>
        </head>
        <body>
            <h1>WebRTC Video Streaming Server</h1>
            <div class="status">
                <p><strong>Status:</strong> Server is running</p>
                <p><strong>Endpoint:</strong> POST to <code>/offer</code> with SDP offer</p>
            </div>
            <h2>Usage</h2>
            <p>Use the Python client to connect:</p>
            <code>python webrtc_video_client.py --server-url http://localhost:8080</code>
        </body>
        </html>
        """
        return web.Response(content_type="text/html", text=html_content)

    async def offer(self, request: web.Request) -> web.Response:
        """Handle WebRTC offer from client and return answer."""
        params = await request.json()
        offer = RTCSessionDescription(sdp=params["sdp"], type=params["type"])
        
        # Get requested topic from params (default to first topic if not specified)
        requested_topic = params.get("topic", self.image_topics[0])
        
        # Validate requested topic
        if requested_topic not in self.image_topics:
            self.get_logger().warning(
                f"Requested topic '{requested_topic}' not available. "
                f"Available topics: {self.image_topics}"
            )
            return web.Response(
                status=400,
                content_type="application/json",
                text=json.dumps({
                    "error": f"Topic '{requested_topic}' not available",
                    "available_topics": self.image_topics
                })
            )

        # Create new peer connection
        pc = RTCPeerConnection()
        self.pcs.add(pc)

        self.get_logger().info(f"Received WebRTC offer for topic: {requested_topic}")

        @pc.on("connectionstatechange")
        async def on_connectionstatechange() -> None:
            self.get_logger().info(f"Connection state: {pc.connectionState}")
            if pc.connectionState == "failed":
                await pc.close()
                self.pcs.discard(pc)
            elif pc.connectionState == "connected":
                self.get_logger().info("Client connected successfully!")

        @pc.on("datachannel")
        def on_datachannel(channel):
            """Handle data channel for latency/RTT measurements."""

            @channel.on("message")
            def on_message(message):
                if isinstance(message, str) and message.startswith("ping"):
                    self.get_logger().debug(f"Received ping: {message}")
                    channel.send("pong" + message[4:])
                elif isinstance(message, str) and message.startswith("latency"):
                    rtt = int(message[7:])
                    self.get_logger().info(f"RTT: {rtt}ms")

        # Create and add video track for the requested topic
        image_track = ImageVideoTrack(self, requested_topic)

        # Force H.264 codec if available
        h264_codecs = [
            codec
            for codec in RTCRtpSender.getCapabilities("video").codecs
            if codec.mimeType == "video/H264"
        ]

        if h264_codecs:
            transceiver = pc.addTransceiver("video")
            transceiver.setCodecPreferences(h264_codecs)
            transceiver.sender.replaceTrack(image_track)
            self.get_logger().info("Using H.264 codec")
        else:
            pc.addTrack(image_track)
            self.get_logger().warning("H.264 not available, using default codec")

        # Set remote description (offer from client)
        await pc.setRemoteDescription(offer)

        # Create and set local description (answer to client)
        answer = await pc.createAnswer()
        await pc.setLocalDescription(answer)

        self.get_logger().info("Sending answer to client")

        return web.Response(
            content_type="application/json",
            text=json.dumps(
                {"sdp": pc.localDescription.sdp, "type": pc.localDescription.type}
            ),
        )

    async def topics(self, request: web.Request) -> web.Response:
        """Return list of available image topics."""
        topics_data = {
            "topics": self.image_topics,
            "count": len(self.image_topics),
            "active_topics": [
                topic for topic, img in self.latest_images.items() if img is not None
            ]
        }
        self.get_logger().debug(f"Topics requested: {topics_data}")
        return web.Response(
            content_type="application/json",
            text=json.dumps(topics_data)
        )

    def run(self):
        """Run the WebRTC video server."""
        app = web.Application()
        app.on_shutdown.append(self.on_shutdown)
        app.router.add_get("/", self.index)
        app.router.add_get("/topics", self.topics)
        app.router.add_post("/offer", self.offer)
        self.get_logger().info(
            f"Starting server on {self.config.host}:{self.config.port}"
        )

        web.run_app(
            app,
            host=self.config.host,
            port=self.config.port,
            access_log_format='%a %t "%r" %s %b',
        )

    async def on_shutdown(self, app: web.Application):
        """Cleanup on server shutdown."""
        coros = [pc.close() for pc in self.pcs]
        await asyncio.gather(*coros)
        self.pcs.clear()
        self.get_logger().info("Server shutdown complete")


def main(args=None):
    rclpy.init(args=args)
    server = Ros2WebrtcServer()

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
