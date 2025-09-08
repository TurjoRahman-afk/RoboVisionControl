#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        
        # Initialize YOLOv8 model
        self.model = YOLO('yolov8n.pt')  # Using nano version, change as needed
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # Create subscriber
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',  # Change this to your image topic
            self.image_callback,
            10)
        self.subscription  # Prevent unused variable warning
        
        self.get_logger().info("Image subscriber node has started and is listening for images...")

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Run YOLOv8 inference
            results = self.model(cv_image)
            
            # Visualize results
            annotated_frame = results[0].plot()
            
            # Display the annotated frame
            cv2.imshow("YOLOv8 Inference", annotated_frame)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f"Error processing image: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    try:
        rclpy.spin(image_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        image_subscriber.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
