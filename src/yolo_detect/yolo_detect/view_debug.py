"""
Simple Debug Image Viewer
=========================
Subscribes to /yolo_debug and displays it using OpenCV.
Press 'q' to quit.

Usage:
    python view_debug.py
    (Run from a terminal with ROS 2 sourced)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
import cv2


class DebugViewer(Node):
    def __init__(self):
        super().__init__('debug_viewer')
        
        self.subscription = self.create_subscription(
            Image,
            '/yolo_debug',
            self.image_callback,
            10
        )
        self.get_logger().info("Debug Viewer started. Waiting for images on /yolo_debug...")
        self.get_logger().info("Press 'q' in the image window to quit.")

    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV format (reverse of what we did in yolo_detect)
        # Reshape the flat byte array back into an image
        img = np.frombuffer(msg.data, dtype=np.uint8)
        img = img.reshape((msg.height, msg.width, 3))  # 3 channels for BGR
        
        # Display the image
        cv2.imshow('YOLO Debug View', img)
        
        # Wait 1ms for key press - 'q' quits
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.get_logger().info("Quitting...")
            rclpy.shutdown()


def main():
    rclpy.init()
    node = DebugViewer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()


if __name__ == '__main__':
    main()

