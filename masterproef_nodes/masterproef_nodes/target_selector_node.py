# this code runs a node which subscribes to messages of the yolo node. these messages contain the pixel coordiates of the robot and 0-9 people that were detected inside of the zone
# the target selector chooses the person that is closest to the robot in bird's eye view and transforms it's pixel coordinates to real world coordinates on the slam map. 
# these coordinates are then published to gotopose topic so that the robot can move to the location using nav2
# the transformation from pixel to slam coordinates is done by retreiving the transform matrix from the csv_transform_watcher node

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
import math
import time
import pandas as pd
import numpy as np
from geometry_msgs.msg import PoseStamped
import threading

# Global transform matrix and its readiness flag
GLOBAL_TRANSFORM_MATRIX = np.identity(3)
TRANSFORM_READY = threading.Event()

class TargetSelector:
    def __init__(self):
        self.targets = []

    def add_target(self, x, y, target_id):
        self.targets.append({'x': x, 'y': y, 'id': target_id})

    def process_coordinates(self, coordinates, robot_id, person_id, null_id):
        self.targets.clear()
        for coord in coordinates:
            x, y, target_id = coord
            if target_id != null_id:
                self.add_target(x, y, target_id)

        robot_points = [target for target in self.targets if target['id'] == robot_id]
        person_points = [target for target in self.targets if target['id'] == person_id]

        distances = []
        for robot_point in robot_points:
            for person_point in person_points:
                distance = math.sqrt((person_point['x'] - robot_point['x'])**2 + (person_point['y'] - robot_point['y'])**2)
                distances.append((distance, person_point))

        return distances, robot_points

    def get_shortest_distance(self, coordinates, robot_id, person_id, null_id):
        distances, robot_points = self.process_coordinates(coordinates, robot_id, person_id, null_id)

        if not robot_points:
            return None, None, coordinates[0] if coordinates else None

        if not distances:
            return None, None, None

        shortest_distance, closest_person = min(distances, key=lambda x: x[0])
        return shortest_distance, closest_person, None


class TargetSelectorNode(Node):
    def __init__(self):
        super().__init__('target_selector_node')
        #self.transform_ready = threading.Event()

        # Publisher for PoseStamped to goal_pose
        self.goal_publisher_ = self.create_publisher(PoseStamped, 'goal_pose', 10)

        # Publisher for acknowledgment to ack_transform
        self.ack_publisher_ = self.create_publisher(String, 'ack_transform', 10)

        self.subscription = self.create_subscription(
            String, 'coordinates_topic', self.callback, 10)

        # Subscribe to transform CSV data updates
        self.create_subscription(String, 'csv_transform_data', self.csv_transform_callback, 10)

        # Create client to trigger the transform CSV update
        self.client = self.create_client(Trigger, 'csv_transform_trigger')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for csv_transform_trigger service...')

        self.get_logger().info("Transform trigger service available, requesting update...")
        self.request_transform_csv()

        self.get_logger().info("Target Selector Node started, waiting for coordinates...")

    def request_transform_csv(self):
        request = Trigger.Request()
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f"Transform trigger response: {future.result().message}")
        else:
            self.get_logger().error("Failed to trigger transform CSV update.")

    def csv_transform_callback(self, msg):
        try:
            self.get_logger().info(f"Received transform CSV data: {msg.data}")
            df = pd.read_json(msg.data)
            if df.empty:
                self.get_logger().warn("Transform CSV was parsed but empty.")
                return

            # Assumes data is a single row
            row = df.iloc[0].to_numpy().astype(float)
            if len(row) != 9:
                self.get_logger().error("Transform CSV does not contain 9 elements.")
                return

            # Clear the flag BEFORE updating
            TRANSFORM_READY.clear()

            global GLOBAL_TRANSFORM_MATRIX
            GLOBAL_TRANSFORM_MATRIX = np.array(row).reshape((3, 3))
            TRANSFORM_READY.set()
            self.get_logger().info(f"Updated global transform matrix:\n{GLOBAL_TRANSFORM_MATRIX}")

            # Acknowledge successful transformation update
            ack_msg = String()
            ack_msg.data = "Transformation updated successfully in the target selector node"
            self.ack_publisher_.publish(ack_msg)
            # self.get_logger().info("Acknowledgment published to ack_transform")

            #self.transform_matrix = np.array(row).reshape((3, 3))
            #self.get_logger().info(f"Transform matrix update called on object id {id(self)}")
            #self.transform_ready.set()
            #self.get_logger().info(f"Updated transform matrix:\n{self.transform_matrix}")

        except Exception as e:
            self.get_logger().error(f"Failed to parse transform CSV: {e}")

    def apply_homography(self, x, y):
        # self.get_logger().info(f"Applying homography to coordinate: ({x}, {y})")
        global GLOBAL_TRANSFORM_MATRIX
        # self.get_logger().info(f"Using global transform matrix:\n{GLOBAL_TRANSFORM_MATRIX}")
        # """Apply homography transformation to the coordinates."""

        # Convert to homogeneous coordinates (x, y, 1)
        point = np.array([x, y, 1])

        # Apply the homography matrix
        transformed_point = GLOBAL_TRANSFORM_MATRIX @ point
        
        # self.get_logger().info(f"Transformed (before normalization): {transformed_point}")

        # Normalize the result by dividing by the homogeneous coordinate (w)
        x_transformed = transformed_point[0] / transformed_point[2]
        y_transformed = transformed_point[1] / transformed_point[2]

        # self.get_logger().info(f"Normalized transformed coordinate: ({x_transformed}, {y_transformed})")

        return x_transformed, y_transformed

    def create_pose_stamped(self, x, y):
        # Create a PoseStamped message with frame 'map' and z=0.0, yaw=0
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0  # No rotation
        return msg

    def callback(self, data):
        if not TRANSFORM_READY.is_set():
            self.get_logger().warn("Global transform matrix not ready yet. Dropping coordinate message.")
            return

        coordinates = self.parse_coordinates(data.data)
        # self.get_logger().info(f"Received coordinates: {coordinates}")

        selector = TargetSelector()
        shortest_distance, closest_person, fallback_coord = selector.get_shortest_distance(coordinates, 'R', 'P', None)

        msg = String()

        if fallback_coord:
            # Apply the homography transformation to the fallback coordinate
            x_transformed, y_transformed = self.apply_homography(fallback_coord[0], fallback_coord[1])

            # No robot found. Fallback coordinate (transformed)
            pose_msg = self.create_pose_stamped(x_transformed, y_transformed)
            self.goal_publisher_.publish(pose_msg)
        elif shortest_distance is not None and closest_person is not None:
            # Apply the homography transformation to the closest person's coordinates
            x_transformed, y_transformed = self.apply_homography(closest_person['x'], closest_person['y'])

            # Publish the transformed coordinates of the closest person
            pose_msg = self.create_pose_stamped(x_transformed, y_transformed)
            self.goal_publisher_.publish(pose_msg)
        else:
            result = "No valid target found."
            self.get_logger().info(result)
        

    def parse_coordinates(self, data):
        coordinates = eval(data)  # Caution: Assumes data is trusted
        return coordinates


def main(args=None):
    rclpy.init(args=args)
    node = TargetSelectorNode()
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
