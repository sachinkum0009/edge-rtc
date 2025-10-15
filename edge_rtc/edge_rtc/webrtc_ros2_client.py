#!/usr/bin/env python3

"""
WebRTC Video Client
Receives video stream from WebRTC server and displays using OpenCV
"""

import argparse
import asyncio
import logging
from typing import Optional
import time
import queue
import threading

import aiohttp
import cv2
from aiortc import RTCPeerConnection, RTCSessionDescription
from aiortc.rtcconfiguration import RTCConfiguration, RTCIceServer
from av import VideoFrame

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

logger = logging.getLogger("webrtc_client")

class WebrtcVideoClient(Node):
    """WebRTC Video Client Class to handle connection and video reception"""
    
    def __init__(self):
        super().__init__("webrtc_video_client")
        self.running = True
        self.window_created = False
        self.frame_count = 0
        self.bridge = CvBridge()
        self.img_pub = self.create_publisher(Image, 'webrtc_video_frames', 10)
        self.last_time = time.time()
        self.frame_queue = queue.Queue(maxsize=2)  # Small queue to avoid lag
        
        # Start publishing thread
        self.pub_thread = threading.Thread(target=self._publish_loop, daemon=True)
        self.pub_thread.start()
        
    def _publish_loop(self):
        """Continuously publish frames from queue"""
        while self.running:
            try:
                img = self.frame_queue.get(timeout=1.0)
                ros_img = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
                ros_img.header.stamp = self.get_clock().now().to_msg()
                self.img_pub.publish(ros_img)
                self.frame_count += 1
                
                # Log FPS
                current_time = time.time()
                if self.frame_count % 30 == 0:
                    fps = 30 / (current_time - self.last_time)
                    logger.info(f"Publishing at {fps:.1f} Hz")
                    self.last_time = current_time
            except queue.Empty:
                continue
            except Exception as e:
                logger.error(f"Error in publish loop: {e}")
                

    def queue_frame(self, img):
        """Queue frame for publishing (non-blocking)"""
        try:
            self.frame_queue.put_nowait(img)
        except queue.Full:
            # Drop frame if queue is full to maintain real-time performance
            pass

        
    def cleanup(self):
        """Clean up resources"""
        self.running = False
        cv2.destroyAllWindows()

async def app():
    rclpy.init()
    webrtc_client = WebrtcVideoClient()
    
    # Create executor in separate thread
    executor = MultiThreadedExecutor()
    executor.add_node(webrtc_client)
    
    # Run ROS2 executor in background thread
    import threading
    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()
    
    pc = RTCPeerConnection()

    @pc.on("track")
    async def on_track(track):
        logger.info(f"Track {track.kind} received")
        if track.kind == "video":
            while webrtc_client.running:
                try:
                    frame = await track.recv()
                    # Convert to numpy array in async context
                    img = frame.to_ndarray(format="bgr24")
                    # Queue frame for publishing (non-blocking)
                    webrtc_client.queue_frame(img)
                except Exception as e:
                    logger.error(f"Error receiving frame: {e}")
                    break
    
    @pc.on("connectionstatechange")
    async def on_connectionstatechange():
        logger.info(f"Connection state: {pc.connectionState}")
        if pc.connectionState == "failed":
            await pc.close()
            webrtc_client.running = False

    # create offer
    logger.info("Creating offer")
    pc.addTransceiver("video", direction="recvonly")
    offer = await pc.createOffer()
    await pc.setLocalDescription(offer)
    logger.info(f"Created offer: {offer.sdp}")

    logger.info("Gathering ICE candidates")
    while pc.iceGatheringState != "complete":
        await asyncio.sleep(0.1)

    # send offer to server
    offer_url = "http://localhost:8080/offer"  # Change to your server URL

    async with aiohttp.ClientSession() as session:
        async with session.post(
            offer_url,
            json={
                "sdp": pc.localDescription.sdp,
                "type": pc.localDescription.type,
            },
            headers={"Content-Type": "application/json"},
        ) as resp:
            if resp.status != 200:
                logger.error(f"Failed to send offer: {resp.status}")
                return
            answer_data = await resp.json()
            logger.info("Received answer")

    answer = RTCSessionDescription(
        sdp=answer_data["sdp"], type=answer_data["type"]
    )
    await pc.setRemoteDescription(answer)
    logger.info("Waiting for video...")

    try:
        while webrtc_client.running and pc.connectionState != "failed":
            await asyncio.sleep(1)
    except KeyboardInterrupt:
        logger.info("Keyboard interrupt received")
    finally:
        webrtc_client.cleanup()
        await pc.close()
        executor.shutdown()
        rclpy.shutdown()
        logger.info("Connection closed")

def main():
    logging.basicConfig(level=logging.INFO)
    asyncio.run(app())

if __name__ == "__main__":
    main()
