"""
RTC Server
"""

from aiohttp import web
from aiortc import (
    RTCPeerConnection,
    RTCRtpSender,
    RTCSessionDescription,
)
from aiortc.contrib.media import MediaRelay
from edge_rtc.image_video_track import ImageVideoTrack
from edge_rtc.utils import EdgeRTCConfig
import logging
import json
import asyncio

import numpy as np
from numpy.typing import NDArray
from typing import Optional

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class RtcServer:
    config: Optional[EdgeRTCConfig] = None
    image_topics: list[str] = []
    latest_images: dict[str, NDArray | None] = {}
    pcs: set[RTCPeerConnection] = set()
    video_sources: Optional[MediaRelay] = None

    def __init__(self, config: EdgeRTCConfig, image_topics: list[str]):
        self.config = config
        self.image_topics = image_topics

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
        if not self.image_topics:
            return web.Response(
                status=500,
                content_type="application/json",
                text=json.dumps({"error": "No image topics configured"}),
            )
        requested_topic = params.get("topic", self.image_topics[0])

        # Validate requested topic
        if requested_topic not in self.image_topics:
            logger.warning(
                f"Requested topic '{requested_topic}' not available. "
                f"Available topics: {self.image_topics}"
            )
            return web.Response(
                status=400,
                content_type="application/json",
                text=json.dumps(
                    {
                        "error": f"Topic '{requested_topic}' not available",
                        "available_topics": self.image_topics,
                    }
                ),
            )

        # Create new peer connection
        pc = RTCPeerConnection()
        self.pcs.add(pc)

        logger.info(f"Received WebRTC offer for topic: {requested_topic}")

        @pc.on("connectionstatechange")
        async def on_connectionstatechange() -> None:
            logger.info(f"Connection state: {pc.connectionState}")
            if pc.connectionState == "failed":
                await pc.close()
                self.pcs.discard(pc)
            elif pc.connectionState == "connected":
                logger.info("Client connected successfully!")

        @pc.on("datachannel")
        def on_datachannel(channel):
            """Handle data channel for latency/RTT measurements."""

            @channel.on("message")
            def on_message(message):
                if isinstance(message, str) and message.startswith("ping"):
                    logger.debug(f"Received ping: {message}")
                    channel.send("pong" + message[4:])
                elif isinstance(message, str) and message.startswith("latency"):
                    rtt = int(message[7:])
                    logger.info(f"RTT: {rtt}ms")

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
            logger.info("Using H.264 codec")
        else:
            pc.addTrack(image_track)
            logger.warning("H.264 not available, using default codec")

        # Set remote description (offer from client)
        await pc.setRemoteDescription(offer)

        # Create and set local description (answer to client)
        answer = await pc.createAnswer()
        await pc.setLocalDescription(answer)

        logger.info("Sending answer to client")

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
            ],
        }
        logger.debug(f"Topics requested: {topics_data}")
        return web.Response(
            content_type="application/json", text=json.dumps(topics_data)
        )

    def run(self):
        """Run the WebRTC video server."""
        app = web.Application()
        app.on_shutdown.append(self.on_shutdown)
        app.router.add_get("/", self.index)
        app.router.add_get("/topics", self.topics)
        app.router.add_post("/offer", self.offer)
        if self.config:
            logger.info(f"Starting server on {self.config.host}:{self.config.port}")

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
        logger.info("Server shutdown complete")

    def get_latest_image(self, topic_name: str) -> NDArray:
        """Returns the latest processed image for a topic or a placeholder if none available."""
        image = self.latest_images.get(topic_name)
        if image is not None:
            return image
        logger.debug(f"No image available for topic {topic_name}, using placeholder")
        # Return a black image as placeholder
        return np.zeros((480, 640, 3), dtype=np.uint8)
