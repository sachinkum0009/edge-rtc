#!/usr/bin/env python3

"""
WebRTC Video Client
Receives video stream from WebRTC server and displays using OpenCV
"""

import argparse
import asyncio
import logging
from typing import Optional

import aiohttp
import cv2
from aiortc import RTCPeerConnection, RTCSessionDescription
from aiortc.rtcconfiguration import RTCConfiguration, RTCIceServer
from av import VideoFrame

logger = logging.getLogger("webrtc_client")

class WebrtcVideoClient:
    """WebRTC Video Client Class to handle connection and video reception"""
    
    def __init__(self):
        self.running = True
        self.window_created = False
        self.frame_count = 0

    async def process_frame(self, frame: VideoFrame):
        """Process a single video frame"""
        # Convert VideoFrame to numpy array
        img = frame.to_ndarray(format="bgr24")
        
        self.frame_count += 1
        
        # Display frame
        # Create named window on first frame to set it to the correct resolution
        if not self.window_created:
            height, width = img.shape[:2]
            cv2.namedWindow("WebRTC Video Stream", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("WebRTC Video Stream", width, height)
            self.window_created = True
            logger.info(f"Created window with resolution: {width}x{height}")
        
        cv2.imshow("WebRTC Video Stream", img)
        # Wait 1ms and check for 'q' key to quit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.running = False
            logger.info("User requested quit")
        
    def cleanup(self):
        """Clean up resources"""
        cv2.destroyAllWindows()

async def main():
    webrtc_client = WebrtcVideoClient()
    pc = RTCPeerConnection()

    @pc.on("track")
    async def on_track(track):
        logger.info(f"Track {track.kind} received")
        if track.kind == "video":
            while webrtc_client.running:
                frame = await track.recv()
                await webrtc_client.process_frame(frame)
    
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
        logger.info("Connection closed")

if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    asyncio.run(main())
