#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('arducam_publisher')
        self.publisher_ = self.create_publisher(Image, 'camera/image_raw', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 FPS
        self.cap = cv2.VideoCapture(0)  # Use /dev/video0 or adjust index
        self.bridge = CvBridge()

        if not self.cap.isOpened():
            self.get_logger().error("Cannot open Arducam IMX477!")
            raise RuntimeError("Camera failed")

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            # Convert OpenCV frame to ROS2 Image message
            ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            self.publisher_.publish(ros_image)
            self.get_logger().info('Publishing video frame')
        else:
            self.get_logger().warn("Frame capture failed")

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
