# this code runs a node which sends a trigger request to the image_server to get an image on the captured_image topic
# once the image is received, it is displayed and the user can click 4 points on the image to determine the keepout zone
# after clicking, the coordinates of the zone are automatically saved to the csv file

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image as ROSImage
from cv_bridge import CvBridge
import cv2
import csv

class ImageClient(Node):
    def __init__(self):
        super().__init__('image_client')
        self.cli = self.create_client(Trigger, 'capture_image')
        self.sub = self.create_subscription(ROSImage, 'captured_image', self.image_callback, 10)
        self.bridge = CvBridge()
        self.csv_path = "/home/tycho/pi4_ws/data/parameters.csv"
        self.points = []

        # Wait for the image capture service to become available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for the image capture service...')

        self.get_logger().info('Image client ready.')

    def send_request(self):
        """Sends a request to capture an image."""
        request = Trigger.Request()
        future = self.cli.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()

        if response.success:
            self.get_logger().info(response.message)
        else:
            self.get_logger().error(response.message)

    def mark_point(self, event, x, y, flags, param):
        """Handles marking points on the image when clicked."""
        if event == cv2.EVENT_LBUTTONDOWN and len(self.points) < 4:
            self.points.append((x, y))
            cv2.circle(self.current_image, (x, y), 5, (0, 0, 255), -1)
            cv2.imshow('Received Image', self.current_image)  # Update the image with the new point
            cv2.waitKey(1)  # Refresh the window immediately
            self.get_logger().info(f'Point marked at: ({x}, {y})')

            # Once four points are marked, arrange them and save them to CSV
            if len(self.points) == 4:
                self.arrange_points()
                self.save_points_to_csv()
                # self.show_points()  # Show the arranged points on the image

    def arrange_points(self):
        """Sorts and arranges the points in the order of UpperLeft, UpperRight, LowerLeft, LowerRight."""
        # Sort points based on x, then y coordinates
        self.points.sort(key=lambda p: (p[0], p[1]))

        left_points = self.points[:2]
        right_points = self.points[2:]

        # Upper and lower points based on y-coordinate
        upper_left = min(left_points, key=lambda p: p[1])
        lower_left = max(left_points, key=lambda p: p[1])
        upper_right = min(right_points, key=lambda p: p[1])
        lower_right = max(right_points, key=lambda p: p[1])

        # Update points in the correct order
        self.points = [upper_left, upper_right, lower_left, lower_right]

    def save_points_to_csv(self):
        """Saves the arranged points to a CSV file."""
        try:
            with open(self.csv_path, mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(['UpperLeft', 'UpperRight', 'LowerLeft', 'LowerRight'])
                writer.writerow([str(self.points[0]), str(self.points[1]), str(self.points[2]), str(self.points[3])])
                # writer.writerow([f'"{self.points[0]}"', f'"{self.points[1]}"', f'"{self.points[2]}"', f'"{self.points[3]}"'])
            self.get_logger().info(f'Points saved to CSV: {self.csv_path}')
        except Exception as e:
            self.get_logger().error(f'Failed to save points to CSV: {str(e)}')

    #def show_points(self):
    #    """Displays the points on the image and shows the updated image."""
    #    for point in self.points:
    #        cv2.circle(self.current_image, point, 10, (0, 255, 0), 2)  # Highlight points with green circles
    #    cv2.imshow('Received Image', self.current_image)

    def image_callback(self, msg):
        """Callback for the image subscription."""
        self.get_logger().info('Image received from server.')
        self.current_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv2.imshow('Received Image', self.current_image)
        cv2.setMouseCallback('Received Image', self.mark_point)
        
        cv2.waitKey(0)
        cv2.destroyAllWindows()  # Ensure the window is closed after interaction

def main(args=None):
    rclpy.init(args=args)
    client = ImageClient()
    client.send_request()
    rclpy.spin(client)  # Continue running and waiting for events
    client.destroy_node()
    rclpy.shutdown()  # Shutdown once the node is no longer needed

if __name__ == '__main__':
    main()
