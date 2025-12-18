"""
ImageVideoTrack for getting the frames and converting it to numpy array
"""

from av import VideoFrame
import asyncio
from aiortc import MediaStreamTrack
from fractions import Fraction
import time

class ImageVideoTrack(MediaStreamTrack):
    """MediaStreamTrack for video, streaming images from the ROS node."""

    def __init__(self, ros2_server):
        super().__init__()
        self.start_time = time.time()
        self.frames = 0
        self.framerate = 30
        self.ros2_server = ros2_server

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
        return image_frame

    async def get_frame(self):
        """Retrieves the latest image frame from the ROS2 server."""
        latest_frame = self.ros2_server.get_latest_image()
        await asyncio.sleep(1.0 / self.framerate)
        return latest_frame
