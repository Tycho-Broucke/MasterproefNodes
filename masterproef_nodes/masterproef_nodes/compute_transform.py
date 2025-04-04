import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image as ROSImage
from cv_bridge import CvBridge
import cv2
import numpy as np
import yaml
import csv

class ComputeTransform(Node):
    def __init__(self):
        super().__init__('compute_transform')

        # ROS2 Components
        self.cli = self.create_client(Trigger, 'capture_image')  # Image request service
        self.sub = self.create_subscription(ROSImage, 'captured_image', self.image_callback, 10)  # Image subscriber
        self.bridge = CvBridge()

        # Paths (Modify as needed)
        self.slam_map_path = "/home/tycho/pi4_ws/data/slam_example_map.pgm"
        self.slam_yaml_path = "/home/tycho/pi4_ws/data/slam_example_map.yaml"
        self.csv_path = "/home/tycho/pi4_ws/data/transform.csv"

        # Transformation Data
        self.camera_points = []
        self.slam_map_pixels = []
        self.real_world_points = []
        self.current_image = None

        # Load SLAM Map
        self.slam_map = cv2.imread(self.slam_map_path)
        if self.slam_map is None:
            self.get_logger().error("Failed to load SLAM map. Check file path.")
            return

        # Load SLAM YAML Metadata
        self.load_slam_metadata()

        self.get_logger().info("Compute Transform Node Ready!")
        self.request_image()  # Initiate image request right away

    def load_slam_metadata(self):
        with open(self.slam_yaml_path, 'r') as file:
            slam_metadata = yaml.safe_load(file)

        self.map_resolution = slam_metadata['resolution']
        self.map_origin = slam_metadata['origin']
        self.map_height = self.slam_map.shape[0]

        self.get_logger().info(f"Loaded SLAM metadata: Resolution={self.map_resolution}, Origin={self.map_origin}")

    def request_image(self):
        if not self.cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("Image capture service unavailable!")
            return

        request = Trigger.Request()
        future = self.cli.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        response = future.result()
        if response and response.success:
            self.get_logger().info("Image capture requested successfully.")
        else:
            self.get_logger().error("Failed to request image capture.")

    def image_callback(self, msg):
        self.get_logger().info("Image received from image server.")
        self.current_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Step 1: Select SLAM Map Points
        self.get_logger().info("Step 1: Select 4 points on SLAM map.")
        self.select_points(self.slam_map, "SLAM Map", self.slam_map_pixels)

        # Step 2: Select Camera Image Points
        self.get_logger().info("Step 2: Select 4 points on Camera Image.")
        self.select_points(self.current_image, "Camera Image", self.camera_points)

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


def main(args=None):
    rclpy.init(args=args)
    node = ComputeTransform()
    rclpy.spin(node)  # Now this is safe since image_callback triggers everything
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
