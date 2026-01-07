#!/usr/bin/env python3
import os
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

TOPICS = {
    "0-FRONT.jpg":        "/camera/front/image_raw",
    "1-FRONT_RIGHT.jpg":  "/camera/front_right/image_raw",
    "2-FRONT_LEFT.jpg":   "/camera/front_left/image_raw",
    "3-BACK.jpg":         "/camera/back/image_raw",
    "4-BACK_LEFT.jpg":    "/camera/back_left/image_raw",
    "5-BACK_RIGHT.jpg":   "/camera/back_right/image_raw",
}

class MultiCamPublisher(Node):
    def __init__(self):
        super().__init__("multi_cam_publisher")
        self.declare_parameter("data_dir", "/root/autodl-tmp/CUDA-FastBEV/example-data")
        self.declare_parameter("fps", 10.0)
        self.declare_parameter("encoding", "bgr8")  # FastBEV一般用BGR就行，必要时改rgb8

        self.data_dir = self.get_parameter("data_dir").get_parameter_value().string_value
        self.fps = float(self.get_parameter("fps").get_parameter_value().double_value)
        self.encoding = self.get_parameter("encoding").get_parameter_value().string_value

        self.bridge = CvBridge()
        self.pub_map = {fn: self.create_publisher(Image, topic, 10) for fn, topic in TOPICS.items()}

        # 预读图片
        self.images = {}
        for fn in TOPICS.keys():
            path = os.path.join(self.data_dir, fn)
            img = cv2.imread(path, cv2.IMREAD_COLOR)
            if img is None:
                raise RuntimeError(f"Failed to read {path}")
            self.images[fn] = img

        period = 1.0 / max(self.fps, 0.1)
        self.timer = self.create_timer(period, self.tick)
        self.get_logger().info(f"Publishing {len(self.images)} camera images from {self.data_dir} at {self.fps} FPS")

    def tick(self):
        stamp = self.get_clock().now().to_msg()
        for fn, pub in self.pub_map.items():
            img = self.images[fn]
            msg = self.bridge.cv2_to_imgmsg(img, encoding=self.encoding)
            msg.header.stamp = stamp
            msg.header.frame_id = fn.replace(".jpg","")
            pub.publish(msg)

def main():
    rclpy.init()
    node = MultiCamPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
