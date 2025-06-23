import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import yaml
import cv2
import numpy as np
import pandas as pd
import ast

class VisualiseSlam(Node):
    def __init__(self):
        super().__init__('visualise_slam')

        self.slam_map_path = "/home/tycho/pi4_ws/data/slam_example_map.pgm"
        self.slam_yaml_path = "/home/tycho/pi4_ws/data/slam_example_map.yaml"
        self.zone_csv_path = "/home/tycho/pi4_ws/data/zone_coordinates.csv"
        self.transform_csv_path = "/home/tycho/pi4_ws/data/transform.csv"

        self.load_map_metadata()
        self.load_transform_matrix()
        self.load_zone_polygon()

        self.subscription = self.create_subscription(
            PoseStamped,
            'goal_pose',
            self.goal_pose_callback,
            10
        )

        self.get_logger().info("visualise_slam node started, listening to goal_pose topic.")

    def load_map_metadata(self):
        with open(self.slam_yaml_path, 'r') as file:
            config = yaml.safe_load(file)
            self.resolution = config['resolution']
            self.origin = config['origin']  # [x, y, theta]

        self.map_img = cv2.imread(self.slam_map_path, cv2.IMREAD_GRAYSCALE)
        if self.map_img is None:
            raise FileNotFoundError(f"Could not read map image from {self.slam_map_path}")
        self.map_height, self.map_width = self.map_img.shape

    def load_transform_matrix(self):
        df = pd.read_csv(self.transform_csv_path)
        values = df.values.flatten()
        if len(values) != 9:
            raise ValueError("Transform CSV must contain 9 values")
        self.transform_matrix = np.array(values).reshape((3, 3))

    def load_zone_polygon(self):
        df = pd.read_csv(self.zone_csv_path)
        row = df.iloc[0]

        # Parse points from CSV string entries
        upper_left = ast.literal_eval(row['UpperLeft'])
        upper_right = ast.literal_eval(row['UpperRight'])
        lower_right = ast.literal_eval(row['LowerRight'])
        lower_left = ast.literal_eval(row['LowerLeft'])

        ordered_pixel_points = [upper_left, upper_right, lower_right, lower_left]

        self.zone_world_coords = []
        for pt in ordered_pixel_points:
            x_pixel, y_pixel = pt
            homogeneous = np.array([x_pixel, y_pixel, 1])
            real_world = self.transform_matrix @ homogeneous
            real_world /= real_world[2]
            self.zone_world_coords.append((real_world[0], real_world[1]))

    def world_to_map(self, x, y):
        origin_x, origin_y, _ = self.origin
        pixel_x = int((x - origin_x) / self.resolution)
        pixel_y = int(self.map_height - (y - origin_y) / self.resolution)  # Y-axis inversion
        return pixel_x, pixel_y

    def draw_zone_polygon(self, img):
        map_coords = [self.world_to_map(x, y) for (x, y) in self.zone_world_coords]
        polygon = np.array(map_coords, np.int32)
        cv2.polylines(img, [polygon], isClosed=True, color=(0, 255, 0), thickness=2)

    def goal_pose_callback(self, msg: PoseStamped):
        x = msg.pose.position.x
        y = msg.pose.position.y
        self.get_logger().info(f"Received goal_pose: x={x:.2f}, y={y:.2f}")

        px, py = self.world_to_map(x, y)
        vis_map = cv2.cvtColor(self.map_img.copy(), cv2.COLOR_GRAY2BGR)
        self.draw_zone_polygon(vis_map)

        if 0 <= px < self.map_width and 0 <= py < self.map_height:
            cv2.circle(vis_map, (px, py), radius=5, color=(0, 0, 255), thickness=-1)
        else:
            self.get_logger().warn("Pixel coordinates are out of bounds! Goal will not be drawn.")

        cv2.imshow("SLAM Map - Goal Pose + Zone", vis_map)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = VisualiseSlam()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
