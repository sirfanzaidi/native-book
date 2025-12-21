#!/usr/bin/env python3
"""
NVIDIA Isaac Object Detection Node
Runs GPU-accelerated object detection on camera images
Simulates YOLO detection for real-time perception
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import BoundingBox2D
import time


class IsaacDetectionNode(Node):
    def __init__(self):
        super().__init__('isaac_detection_node')

        # Subscribe to camera images from robot
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)

        # Publisher for detection results (bounding boxes)
        self.detection_pub = self.create_publisher(
            BoundingBox2D,
            '/detections',
            10)

        # Performance monitoring
        self.frame_count = 0
        self.fps_start = time.time()
        self.get_logger().info('Isaac detection node started')
        self.get_logger().info('Listening on /camera/image_raw')

    def image_callback(self, msg):
        """Process camera image with GPU detection"""
        # In real Isaac, this would:
        # 1. Convert image to GPU format
        # 2. Run YOLO model on GPU
        # 3. Extract detections
        # 4. Filter by confidence

        # For this example, simulate detection at (center, 1/4 size)
        image_width = msg.width
        image_height = msg.height

        detection = BoundingBox2D()
        detection.center.x = float(image_width // 2)
        detection.center.y = float(image_height // 2)
        detection.size_x = float(image_width // 4)
        detection.size_y = float(image_height // 4)

        # Publish detection result
        self.detection_pub.publish(detection)

        # Track and log FPS (indicator of real-time performance)
        self.frame_count += 1
        elapsed = time.time() - self.fps_start

        if elapsed >= 1.0:
            fps = self.frame_count / elapsed
            self.get_logger().info(
                f'FPS: {fps:.1f} | Image: {image_width}x{image_height}')
            self.frame_count = 0
            self.fps_start = time.time()


def main(args=None):
    rclpy.init(args=args)
    detection_node = IsaacDetectionNode()
    rclpy.spin(detection_node)
    detection_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
