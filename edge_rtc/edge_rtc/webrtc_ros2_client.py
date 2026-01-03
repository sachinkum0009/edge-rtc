#!/usr/bin/env python3

"""WebRTC Video Client
Receives video stream from WebRTC server and displays using OpenCV.
"""

import asyncio
import logging
import os
import queue
import threading
import time
from typing import Dict, Optional

import aiohttp
import cv2
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from aiortc import RTCPeerConnection, RTCSessionDescription
from cv_bridge import CvBridge
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.publisher import Publisher
from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import Image

logger = logging.getLogger("webrtc_client")


class TopicHandler:
    """Handles a single topic's WebRTC connection and ROS2 publishing."""
    
    def __init__(self, topic_name: str, publisher: Publisher, bridge: CvBridge, node: Node):
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
            daemon=True,
            name=f"pub_thread_{topic_name}"
        )
        self.pub_thread.start()
        logger.info(f"TopicHandler initialized for {topic_name}")
    
    def _publish_loop(self):
        """Continuously publish frames from queue for this topic."""
        while self.running:
            try:
                img = self.frame_queue.get(timeout=1.0)
                ros_img = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
                ros_img.header.stamp = self.node.get_clock().now().to_msg()
                ros_img.header.frame_id = self.topic_name
                self.publisher.publish(ros_img)
                self.frame_count += 1

                # Log FPS
                current_time = time.time()
                if self.frame_count % 30 == 0:
                    fps = 30 / (current_time - self.last_time)
                    logger.info(f"[{self.topic_name}] Publishing at {fps:.1f} Hz")
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


class WebrtcVideoClient(Node):
    """WebRTC Video Client Class to handle multiple topic connections and video reception."""

    def __init__(self):
        super().__init__("webrtc_video_client")
        self.running = True
        self.bridge = CvBridge()
        
        # Declare ROS2 parameter for image topics
        self.declare_parameter(
            "image_topics",
            value=["image_raw"],
            descriptor=ParameterDescriptor(
                description="List of image topics to stream"
            ),
        )
        
        # Get image topics from ROS2 parameter
        self.topics = list(
            self.get_parameter("image_topics").get_parameter_value().string_array_value
        )
        
        # Load server configuration from YAML file
        pkg_path = get_package_share_directory("edge_rtc")
        client_config = os.path.join(pkg_path, "config/client.yaml")
        
        with open(client_config) as f:
            config_data = yaml.safe_load(f)
        
        if config_data is None or not isinstance(config_data, dict):
            config_data = {}
        
        # Get host and port from config file
        self.host = config_data.get("host", "localhost")
        self.port = config_data.get("port", 8080)
        self.server_url = f"http://{self.host}:{self.port}"
        
        logger.info(f"Loaded config - Server: {self.server_url}, Topics: {self.topics}")
        
        self.topic_handlers: Dict[str, TopicHandler] = {}
        
        # Create a publisher and handler for each topic
        for topic in self.topics:
            # Create output topic name by replacing '/' with '_' and adding prefix
            output_topic = f"webrtc{topic.replace('/', '_')}"
            publisher = self.create_publisher(Image, output_topic, 10)
            handler = TopicHandler(topic, publisher, self.bridge, self)
            self.topic_handlers[topic] = handler
            logger.info(f"Created publisher for {topic} -> {output_topic}")

    def get_handler(self, topic_name: str) -> Optional[TopicHandler]:
        """Get the handler for a specific topic."""
        return self.topic_handlers.get(topic_name)

    def cleanup(self):
        """Clean up resources."""
        self.running = False
        for handler in self.topic_handlers.values():
            handler.stop()
        cv2.destroyAllWindows()



async def create_peer_connection(server_url: str, topic: str, webrtc_client: WebrtcVideoClient):
    """Create and manage a WebRTC peer connection for a specific topic."""
    pc = RTCPeerConnection()
    handler = webrtc_client.get_handler(topic)
    
    if not handler:
        logger.error(f"No handler found for topic {topic}")
        return None

    @pc.on("track")
    async def on_track(track):
        logger.info(f"[{topic}] Track {track.kind} received")
        if track.kind == "video":
            frame_count = 0
            while webrtc_client.running:
                try:
                    frame = await track.recv()
                    # Convert to numpy array in async context
                    img = frame.to_ndarray(format="bgr24")
                    # Queue frame for publishing (non-blocking)
                    handler.queue_frame(img)
                    frame_count += 1
                    
                    # Log every 30 frames
                    if frame_count % 30 == 0:
                        logger.info(f"[{topic}] Received {frame_count} frames, shape: {img.shape}")
                except asyncio.CancelledError:
                    logger.info(f"[{topic}] Track recv cancelled")
                    break
                except Exception as e:
                    logger.error(f"[{topic}] Error receiving frame: {e}", exc_info=True)
                    break
            logger.info(f"[{topic}] Track recv loop ended, total frames: {frame_count}")

    @pc.on("connectionstatechange")
    async def on_connectionstatechange():
        logger.info(f"[{topic}] Connection state: {pc.connectionState}")
        if pc.connectionState == "failed":
            await pc.close()

    # Create offer
    logger.info(f"[{topic}] Creating offer")
    pc.addTransceiver("video", direction="recvonly")
    offer = await pc.createOffer()
    await pc.setLocalDescription(offer)

    logger.info(f"[{topic}] Gathering ICE candidates")
    while pc.iceGatheringState != "complete":
        await asyncio.sleep(0.1)

    # Send offer to server
    offer_url = f"{server_url}/offer"

    async with aiohttp.ClientSession() as session:
        async with session.post(
            offer_url,
            json={
                "sdp": pc.localDescription.sdp,
                "type": pc.localDescription.type,
                "topic": topic,
            },
            headers={"Content-Type": "application/json"},
        ) as resp:
            if resp.status != 200:
                error_text = await resp.text()
                logger.error(f"[{topic}] Failed to send offer: {resp.status} - {error_text}")
                return None
            answer_data = await resp.json()
            logger.info(f"[{topic}] Received answer")

    answer = RTCSessionDescription(
        sdp=answer_data["sdp"], type=answer_data["type"]
    )
    await pc.setRemoteDescription(answer)
    logger.info(f"[{topic}] Waiting for video...")

    return pc


async def app():
    """Main application to handle multiple WebRTC connections."""
    rclpy.init()
    webrtc_client = WebrtcVideoClient()

    # Create executor in separate thread
    executor = MultiThreadedExecutor()
    executor.add_node(webrtc_client)

    # Run ROS2 executor in background thread
    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()

    # Get topics and server URL from client config
    topics = webrtc_client.topics
    server_url = webrtc_client.server_url

    # Create peer connections for all topics
    peer_connections = []
    for topic in topics:
        pc = await create_peer_connection(server_url, topic, webrtc_client)
        if pc:
            peer_connections.append(pc)
            logger.info(f"Successfully connected to topic: {topic}")
        else:
            logger.warning(f"Failed to connect to topic: {topic}")

    if not peer_connections:
        logger.error("No peer connections established. Exiting.")
        webrtc_client.cleanup()
        executor.shutdown()
        rclpy.shutdown()
        return

    logger.info(f"All connections established. Streaming {len(peer_connections)} topics.")

    # Keep running until interrupted
    try:
        while webrtc_client.running:
            # Check if any connection failed
            active_connections = sum(
                1 for pc in peer_connections if pc.connectionState not in ["failed", "closed"]
            )
            if active_connections == 0:
                logger.warning("All connections lost. Exiting.")
                break
            await asyncio.sleep(1)
    except KeyboardInterrupt:
        logger.info("Keyboard interrupt received")
    finally:
        webrtc_client.cleanup()
        # Close all peer connections
        for pc in peer_connections:
            await pc.close()
        executor.shutdown()
        rclpy.shutdown()
        logger.info("All connections closed")


def main():
    logging.basicConfig(level=logging.INFO)
    logger.info("Starting WebRTC Video Client - Multi-Topic Support")
    asyncio.run(app())

if __name__ == "__main__":
    main()
