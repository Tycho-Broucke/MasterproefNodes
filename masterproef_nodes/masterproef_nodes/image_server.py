# This code runs an image server node: the node captures images from the usb camera and publishes it once a trigger request from the client is received. 
# the node publishes the image to the captured_image topic

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger  # Using Trigger service
from sensor_msgs.msg import Image as ROSImage
import cv2
from cv_bridge import CvBridge
from builtin_interfaces.msg import Time

class ImageServer(Node):
    def __init__(self):
        super().__init__('image_server')
        self.srv = self.create_service(Trigger, 'capture_image', self.handle_image_request)
        self.bridge = CvBridge()
        self.get_logger().info('Image server is ready to capture from USB camera.')
        self.pub = self.create_publisher(ROSImage, 'captured_image', 10)

    def handle_image_request(self, request, response):
        self.get_logger().info('Image capture triggered.')

        # Open the USB camera (usually at index 0)
        cap = cv2.VideoCapture(0)

        if not cap.isOpened():
            response.success = False
            response.message = 'Failed to open USB camera.'
            self.get_logger().error(response.message)
            return response

        ret, frame = cap.read()
        cap.release()

        if not ret:
            response.success = False
            response.message = 'Failed to capture image.'
            self.get_logger().error(response.message)
            return response

        # Convert OpenCV image to ROS Image message
        now = self.get_clock().now().to_msg()
        ros_image = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        ros_image.header.stamp = now
        self.pub.publish(ros_image)
        
        response.success = True
        response.message = 'Image captured and published successfully.'
        self.get_logger().info(response.message)
        return response

def main(args=None):
    rclpy.init(args=args)
    node = ImageServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()