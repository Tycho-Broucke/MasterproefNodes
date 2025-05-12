# this code runs a node which retreives a stitched image from the service and lets the user draw 4 points on it
# it also lets the user draw the 4 EXACT same points on the slam map and calculates the transformation between image pixel coordinates and real world slam coordinates
# finally, it saves the transformation matrix to a csv file for later use
# it also listens to an acknowledgment from the target selecter, which lets the user know once the new transformation matrix is correctly updated

import rclpy
from rclpy.node import Node
from masterproef_interfaces.srv import GetStitchedImage
from std_msgs.msg import String
from sensor_msgs.msg import Image as ROSImage
from cv_bridge import CvBridge
import cv2
import numpy as np
import yaml
import csv

class ComputeTransformation(Node):
    def __init__(self):
        super().__init__('compute_transformation')

        # ROS2 Components
        self.cli = self.create_client(GetStitchedImage, 'get_stitched_image')
        self.ack_subscription = self.create_subscription(
            String,
            'ack_transform',
            self.ack_callback,
            10)
        self.bridge = CvBridge()

        # Paths (modify as needed)
        self.slam_map_path = "/home/tycho/pi4_ws/data/slam_example_map.pgm" # CHANGE THIS PATH TO THE CORRECT PATH ON THE JETSON
        self.slam_yaml_path = "/home/tycho/pi4_ws/data/slam_example_map.yaml" # CHANGE THIS PATH TO THE CORRECT PATH ON THE JETSON
        self.csv_path = "/home/tycho/pi4_ws/data/transform.csv" # CHANGE THIS PATH TO THE CORRECT PATH ON THE JETSON

        # Transformation Data
        self.camera_points = []
        self.slam_map_pixels = []
        self.real_world_points = []

        # Load SLAM Map
        self.slam_map = cv2.imread(self.slam_map_path)
        if self.slam_map is None:
            self.get_logger().error("Failed to load SLAM map. Check file path.")
            return

        # Load SLAM YAML Metadata
        self.load_slam_metadata()

        # Wait for service to become available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for stitched image service...')

        self.get_logger().info("Compute Transform Node Ready!")
        self.request_stitched_image()


    def load_slam_metadata(self):
        with open(self.slam_yaml_path, 'r') as file:
            slam_metadata = yaml.safe_load(file)

        self.map_resolution = slam_metadata['resolution']
        self.map_origin = slam_metadata['origin']
        self.map_height = self.slam_map.shape[0]

        self.get_logger().info(f"Loaded SLAM metadata: Resolution={self.map_resolution}, Origin={self.map_origin}")

    def request_stitched_image(self):

        request = GetStitchedImage.Request()
        future = self.cli.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        response = future.result()
        if response and response.success:
            self.get_logger().info("Stitched image received successfully.")
            self.process_image(response.image)
        else:
            self.get_logger().error(f"Failed to receive stitched image: {response.message if response else 'No response'}")

    def process_image(self, ros_image_msg):
        current_image = self.bridge.imgmsg_to_cv2(ros_image_msg, desired_encoding='bgr8')

        # Step 1: Select SLAM Map Points
        self.get_logger().info("Step 1: Select 4 points on SLAM map.")
        self.select_points(self.slam_map.copy(), "SLAM Map", self.slam_map_pixels)

        # Step 2: Select Camera Image Points
        self.get_logger().info("Step 2: Select 4 points on stitched Camera Image.")
        self.select_points(current_image.copy(), "Stitched Image", self.camera_points)

        # Step 3: Convert to real-world coordinates
        self.get_logger().info("Step 3: Convert SLAM pixels to real-world coordinates.")
        self.convert_pixels_to_real_world()

        # Step 4: Compute Transformation
        self.get_logger().info("Step 4: Compute transformation matrix.")
        T = self.compute_transformation()

        if T is not None:
            self.get_logger().info("Transformation matrix computed successfully.")
            self.save_transformation_to_csv(T)
        else:
            self.get_logger().error("Failed to compute transformation.")

    def select_points(self, image, window_name, point_list):
        def mouse_callback(event, x, y, flags, param):
            if event == cv2.EVENT_LBUTTONDOWN and len(point_list) < 4:
                point_list.append((x, y))
                cv2.circle(image, (x, y), 5, (0, 0, 255), -1)
                cv2.imshow(window_name, image)
                self.get_logger().info(f"Point selected: ({x}, {y})")

        cv2.imshow(window_name, image)
        cv2.setMouseCallback(window_name, mouse_callback)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    def convert_pixels_to_real_world(self):
        for (px, py) in self.slam_map_pixels:
            real_x = self.map_origin[0] + (px * self.map_resolution)
            real_y = self.map_origin[1] + ((self.map_height - py) * self.map_resolution)
            self.real_world_points.append((real_x, real_y))
            self.get_logger().info(f"Pixel ({px}, {py}) -> World ({real_x}, {real_y})")

    def compute_transformation(self):
        if len(self.camera_points) < 4 or len(self.real_world_points) < 4:
            self.get_logger().error("Not enough points selected.")
            return None

        camera_pts = np.array(self.camera_points, dtype=np.float32)
        slam_pts = np.array(self.real_world_points, dtype=np.float32)

        T, _ = cv2.findHomography(camera_pts, slam_pts, method=cv2.RANSAC)
        return T

    def save_transformation_to_csv(self, T):
        if T is None:
            self.get_logger().error("Transformation matrix is None. Not saving.")
            return

        with open(self.csv_path, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['T11', 'T12', 'T13', 'T21', 'T22', 'T23', 'T31', 'T32', 'T33'])
            writer.writerow(T.flatten())

        self.get_logger().info(f"Transformation matrix saved to {self.csv_path}")

    def ack_callback(self, msg):
        self.get_logger().info(f"Acknowledgment received: {msg.data}")


def main(args=None):
    rclpy.init(args=args)
    node = ComputeTransformation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt received. Shutting down node.")
    except Exception as e:
        node.get_logger().error(f"Unhandled exception: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
