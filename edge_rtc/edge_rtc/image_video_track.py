"""
ImageVideoTrack for getting the frames and converting it to numpy array
"""

from av import VideoFrame
import asyncio
from aiortc import MediaStreamTrack
from fractions import Fraction
import time
import logging
# from rtc_server import RtcServer
from numpy.typing import NDArray
import numpy as np
import cv2

logger = logging.getLogger(__name__)

class ImageVideoTrack(MediaStreamTrack):
    """MediaStreamTrack for video, streaming images from the ROS node."""

    def __init__(self, rtc_server, topic_name: str):
        super().__init__()
        self.start_time = time.time()
        self.frames = 0
        self.framerate = 30
        self.rtc_server = rtc_server
        self.topic_name = topic_name
        self.send_count = 0
        logger.info(f"ImageVideoTrack created for topic: {topic_name}")

    async def next_timestamp(self):
        """Calculates the timestamp for the next frame."""
        self.frames += 1
        next_time = self.start_time + (self.frames / self.framerate)
        await asyncio.sleep(max(0, next_time - time.time()))
        return int((next_time - self.start_time) * 1000)

    async def recv(self):
        """Receives the next video frame to be sent to the peer."""
        frame = await self.get_frame()
        image_frame = VideoFrame.from_ndarray(frame, format="bgr24")
        image_frame.pts = await self.next_timestamp()
        image_frame.time_base = Fraction(1, 1000)
        
        self.send_count += 1
        if self.send_count % 30 == 0:
            logger.info(f"[{self.topic_name}] Sent {self.send_count} frames to WebRTC peer")
        
        return image_frame

    async def get_frame(self) -> NDArray:
        """Retrieves the latest image frame from the RTC server for the specified topic."""
        latest_frame = self.rtc_server.get_latest_image(self.topic_name)

        # Ensure we have a numpy array; if not, create a black placeholder
        if latest_frame is None:
            latest_frame = np.zeros((480, 640, 3), dtype=np.uint8)

        # If grayscale (2D array), convert to BGR
        if hasattr(latest_frame, "ndim") and latest_frame.ndim == 2:
            latest_frame = cv2.cvtColor(latest_frame, cv2.COLOR_GRAY2BGR)

        # If not uint8, attempt to convert safely
        if latest_frame.dtype != np.uint8:
            if latest_frame.dtype == np.uint16:
                # scale 16-bit -> 8-bit
                latest_frame = (latest_frame >> 8).astype(np.uint8)
            else:
                # fallback: normalize/clamp into uint8 range
                latest_frame = np.clip(latest_frame, 0, 255).astype(np.uint8)

        await asyncio.sleep(1.0 / self.framerate)
        return latest_frame
