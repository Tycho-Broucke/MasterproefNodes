# this code runs a yolo node that requests a stitched image from the image server with the custom GetStitchedImage service
# once the requested image is received, the user can click a location on the screen to point out the charging location for the robot
# this location is saved to a csv file for later use (to navigate the robot to the location when needed)

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as ROSImage
from cv_bridge import CvBridge
from masterproef_interfaces.srv import GetStitchedImage
import cv2
import csv

class ChargeLocationSelector(Node):
    def __init__(self):
        super().__init__('charge_location_selector')
        self.cli = self.create_client(GetStitchedImage, 'get_stitched_image')
        self.bridge = CvBridge()
        self.csv_path = "/home/tycho/pi4_ws/data/charge_location.csv" # CHANGE THIS PATH TO THE CORRECT PATH ON THE JETSON
        self.point = None

        # Wait for the stitched image service to become available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for stitched image service...')

        self.get_logger().info('Charge Location Selector ready.')
        self.send_request()

    def send_request(self):
        """Sends a request to get the stitched image."""
        request = GetStitchedImage.Request()
        future = self.cli.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()

        if response.success:
            self.get_logger().info("Received stitched image.")
            self.display_image(response.image)
        else:
            self.get_logger().error(f"Failed to get stitched image: {response.message}")

    def mark_point(self, event, x, y, flags, param):
        """Handles marking a point on the stitched image."""
        if event == cv2.EVENT_LBUTTONDOWN and self.point is None:
            self.point = (x, y)
            cv2.circle(self.image, self.point, 5, (0, 255, 0), -1)
            cv2.imshow('Select Charge Location', self.image)
            self.get_logger().info(f'Point selected at: ({x}, {y})')
            self.save_point_to_csv()
            cv2.waitKey(1000)
            cv2.destroyAllWindows()

    def save_point_to_csv(self):
        """Saves the selected point to a CSV file."""
        try:
            with open(self.csv_path, mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(['X', 'Y'])
                writer.writerow([self.point[0], self.point[1]])
            self.get_logger().info(f'Point saved to CSV: {self.csv_path}')
        except Exception as e:
            self.get_logger().error(f'Failed to save point to CSV: {str(e)}')

    def display_image(self, ros_image_msg):
        """Converts and shows the stitched image."""
        self.image = self.bridge.imgmsg_to_cv2(ros_image_msg, desired_encoding='bgr8')
        height, width = self.image.shape[:2]
        # self.get_logger().info(f"Stitched image dimensions: {width}x{height} (width x height)")
        cv2.imshow('Select Charge Location', self.image)
        cv2.setMouseCallback('Select Charge Location', self.mark_point)
        cv2.waitKey(0)  # Wait for the user to click
        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = ChargeLocationSelector()
    try:
        rclpy.spin(node)  # Keeps the node alive to process callbacks
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt received. Shutting down node.")
    except Exception as e:
        node.get_logger().error(f"Unhandled exception: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
