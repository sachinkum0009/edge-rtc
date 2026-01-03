"""
This script is used to publish the image from the camera
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge, CvBridgeError


class CamImg(Node):
    def __init__(self):
        super().__init__("cam_img_node")
        self.cam_img_pub = self.create_publisher(Image, "image_raw", 10)
        self.timer = self.create_timer(0.1, self.timer_cb)
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("Could not open camera.")

    def timer_cb(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning("Failed to capture image from camera.")
            return
        try:
            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.cam_img_pub.publish(img_msg)
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge Error: {e}")

    def destroy_node(self):
        if hasattr(self, "cap") and self.cap.isOpened():
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CamImg()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
