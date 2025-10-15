#!/home/asus/zzzzz/ros2/k3s/colcon_ws/venvs/webrtc/bin/python
"""
WebRTC Video Streaming Server
Streams video from webcam or video file to connected clients
"""
import os
import sys
import logging
import yaml
import json

from aiohttp import web
from aiortc import (
    MediaStreamTrack,
    RTCPeerConnection,
    RTCSessionDescription,
)
from aiortc.contrib.media import MediaPlayer, MediaRelay
from aiortc import RTCRtpSender, RTCRtpCodecCapability

from edge_rtc.utils import EdgeRTCConfig

from typing import List, Set, Optional

logger = logging.getLogger("webrtc_server")


class WebrtcVideoServer:
    """
    WebRTC Video Server Class which manages peer connections and video tracks
    """
    pcs : Set[RTCPeerConnection] = set()
    relay: Optional[MediaRelay] = None
    video_source: Optional[MediaPlayer] = None
    def __init__(self):
        # TODO: (Sachin) Get absolute path using ros2 api
        with open('/home/asus/zzzzz/ros2/k3s/colcon_ws/src/edge-rtc/edge_rtc/config/server.yaml', 'r') as f:
            config_data = yaml.safe_load(f)
        self.config = EdgeRTCConfig(**config_data)

    def run(self):
        """Run the WebRTC video server"""
        app = web.Application()
        app.on_shutdown.append(self.on_shutdown)
        app.router.add_get("/", self.index)
        app.router.add_post("/offer", self.offer)
        logger.info(f"Starting server on {self.config.host}:{self.config.port}")

        web.run_app(
            app,
            host=self.config.host,
            port=self.config.port,
            access_log_format='%a %t "%r" %s %b',
        )

    def create_video_track(self) -> Optional[MediaStreamTrack]:
        """
        Create a video track from webcam or file source
        
        Args:
            source_path: Path to video file, or None for webcam
            use_webcam: Whether to use webcam if no source_path
            use_nvidia: Whether to use NVIDIA hardware encoding
        
        Returns:
            MediaStreamTrack for video
        """
        # Play from webcam using MediaRelay for multiple clients
        if self.relay is None:
            options = {"framerate": f"{self.config.framerate}", "video_size": f"{self.config.resolution}"}
            if not os.path.exists(self.config.video_device):
                logger.error(f"Video device {self.config.video_device} does not exist")
                sys.exit(1)

            logger.info(f"Using video device: {self.config.video_device}")
            self.video_source = MediaPlayer(self.config.video_device, format="v4l2", options=options)

            self.relay = MediaRelay()

        if self.video_source and self.video_source.video:
            return self.relay.subscribe(self.video_source.video)
        return None
    

    async def offer(self, request: web.Request) -> web.Response:
        """
        Handle WebRTC offer from client and return answer
        """
        params = await request.json()
        offer = RTCSessionDescription(sdp=params["sdp"], type=params["type"])

        # Create new peer connection
        pc = RTCPeerConnection()
        self.pcs.add(pc)

        logger.info(f"New connection from {request.remote}")

        @pc.on("connectionstatechange")
        async def on_connectionstatechange() -> None:
            logger.info(f"Connection state: {pc.connectionState}")
            if pc.connectionState == "failed":
                await pc.close()
                self.pcs.discard(pc)
            elif pc.connectionState == "connected":
                logger.info("Client connected successfully!")

        # Add video track
        video_track = self.create_video_track()
        if video_track:
            # --- ✅ Force H.264 codec if available ---
            h264_codecs = [
                codec
                for codec in RTCRtpSender.getCapabilities("video").codecs
                if codec.mimeType == "video/H264"
            ]
            if h264_codecs:
                transceiver = pc.addTransceiver("video")
                transceiver.setCodecPreferences(h264_codecs)
                transceiver.sender.replaceTrack(video_track)
            else:
                pc.addTrack(video_track)
                logger.warning("H.264 not available, using default codec")
        else:
            logger.error("Failed to create video track")

        # Set remote description (offer from client)
        await pc.setRemoteDescription(offer)

        # Create and set local description (answer to client)
        answer = await pc.createAnswer()
        await pc.setLocalDescription(answer)

        logger.info("Sending answer to client")

        return web.Response(
            content_type="application/json",
            text=json.dumps({
                "sdp": pc.localDescription.sdp,
                "type": pc.localDescription.type
            }),
        )

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

    async def on_shutdown(self, app: web.Application):
        """Cleanup on server shutdown"""
        pass

def main():
    logging.basicConfig(level=logging.INFO)
    webrtc_server = WebrtcVideoServer()
    try:
        webrtc_server.run()
    except Exception as e:
        logger.error(f"Error running server: {e}")

if __name__ == "__main__":
    main()
