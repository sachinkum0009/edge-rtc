#! /usr/bin/env python3

"""
WebRTC Video Streamer Server for ROS2 Topics
Streams video from ROS2 image topic to connected WebRTC clients
"""

import os
import sys
import logging
import yaml
import json
import asyncio
import time
import threading
from fractions import Fraction

from aiohttp import web
from aiortc import (
    MediaStreamTrack,
    RTCPeerConnection,
    RTCSessionDescription,
)
from aiortc.contrib.media import MediaPlayer, MediaRelay
from aiortc import RTCRtpSender, RTCRtpCodecCapability
from av import VideoFrame

from edge_rtc.utils import EdgeRTCConfig

from typing import List, Set, Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class ImageVideoTrack(MediaStreamTrack):
    """
    MediaStreamTrack for video, streaming images from the ROS node.
    """
    kind = "video"

    def __init__(self, ros2_server):
        super().__init__()
        self.start_time = time.time()
        self.frames = 0
        self.framerate = 30
        self.ros2_server = ros2_server

    async def next_timestamp(self):
        """
        Calculates the timestamp for the next frame.
        """
        self.frames += 1
        next_time = self.start_time + (self.frames / self.framerate)
        await asyncio.sleep(max(0, next_time - time.time()))
        return int((next_time - self.start_time) * 1000)

    async def recv(self):
        """
        Receives the next video frame to be sent to the peer.
        """
        frame = await self.get_frame()
        image_frame = VideoFrame.from_ndarray(frame, format="bgr24")
        image_frame.pts = await self.next_timestamp()
        image_frame.time_base = Fraction(1, 1000)
        return image_frame

    async def get_frame(self):
        """
        Retrieves the latest image frame from the ROS2 server.
        """
        latest_frame = self.ros2_server.get_latest_image()
        await asyncio.sleep(1.0 / self.framerate)
        return latest_frame


class Ros2WebrtcServer(Node):
    """ROS2 Node to stream video from image topic via WebRTC"""
    pcs : Set[RTCPeerConnection] = set()
    relay: Optional[MediaRelay] = None
    video_source: Optional[MediaPlayer] = None
    
    def __init__(self):
        super().__init__("ros2_webrtc_server")
        # TODO: (Sachin) Get absolute path using ros2 api
        with open('/home/asus/zzzzz/ros2/k3s/colcon_ws/src/edge-rtc/edge_rtc/config/server.yaml', 'r') as f:
            config_data = yaml.safe_load(f)
        self.config = EdgeRTCConfig(**config_data)
        self.bridge = CvBridge()
        
        # Image buffering
        self.lock = threading.Lock()
        self.latest_image = None
        self.new_image = None
        self.fps = 30
        self.last_time = time.time()
        
        # Create placeholder image (640x480 black image with text)
        self.placeholder_image = self.create_placeholder_image()
        
        self.subscription = self.create_subscription(
            Image,
            'image_raw',
            self.image_callback,
            10
        )
        self.get_logger().info("ROS2 WebRTC Server Node Initialized")
    
    def create_placeholder_image(self):
        """Create a placeholder image when no data is available"""
        img = cv2.imread("placeholder.jpg") if os.path.exists("placeholder.jpg") else None
        if img is None:
            # Create black image with text
            img = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(img, "Waiting for ROS2 images...", (50, 240),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        return img
        
    def image_callback(self, msg: Image):
        """Callback to handle incoming image messages"""
        current_time = time.time()
        if current_time - self.last_time >= 1.0 / self.fps:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            with self.lock:
                self.new_image = cv_image
            self.last_time = current_time

    def get_latest_image(self):
        """
        Returns the latest processed image or a placeholder if none available.
        """
        with self.lock:
            return self.new_image if self.new_image is not None else self.placeholder_image

    @staticmethod
    async def index(request: web.Request) -> web.Response:
        """Serve a simple HTML page for testing"""
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
        """
        Handle WebRTC offer from client and return answer
        """
        params = await request.json()
        offer = RTCSessionDescription(sdp=params["sdp"], type=params["type"])

        # Create new peer connection
        pc = RTCPeerConnection()
        self.pcs.add(pc)

        self.get_logger().info("Received WebRTC offer")

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
            """Handle data channel for latency/RTT measurements"""
            @channel.on("message")
            def on_message(message):
                if isinstance(message, str) and message.startswith("ping"):
                    self.get_logger().debug(f"Received ping: {message}")
                    channel.send("pong" + message[4:])
                elif isinstance(message, str) and message.startswith("latency"):
                    rtt = int(message[7:])
                    self.get_logger().info(f"RTT: {rtt}ms")

        # Create and add video track
        image_track = ImageVideoTrack(self)
        
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
            text=json.dumps({
                "sdp": pc.localDescription.sdp,
                "type": pc.localDescription.type
            }),
        )

    def run(self):
        """Run the WebRTC video server"""
        app = web.Application()
        app.on_shutdown.append(self.on_shutdown)
        app.router.add_get("/", self.index)
        app.router.add_post("/offer", self.offer)
        self.get_logger().info(f"Starting server on {self.config.host}:{self.config.port}")

        web.run_app(
            app,
            host=self.config.host,
            port=self.config.port,
            access_log_format='%a %t "%r" %s %b',
        )

    async def on_shutdown(self, app: web.Application):
        """Cleanup on server shutdown"""
        coros = [pc.close() for pc in self.pcs]
        await asyncio.gather(*coros)
        self.pcs.clear()
        self.get_logger().info("Server shutdown complete")


def main(args=None):
    rclpy.init(args=args)
    server = Ros2WebrtcServer()
    
    # Start ROS2 spin in a separate thread
    ros_thread = threading.Thread(target=lambda: rclpy.spin(server), daemon=True)
    ros_thread.start()
    
    try:
        # Run the web server (blocking)
        server.run()
    except KeyboardInterrupt:
        pass
    finally:
        server.destroy_node()
        rclpy.shutdown()
        ros_thread.join(timeout=2.0)


if __name__ == "__main__":
    main()
