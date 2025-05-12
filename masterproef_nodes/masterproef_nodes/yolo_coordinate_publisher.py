import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from ultralytics import YOLO
import cv2
import numpy as np
import pandas as pd
import time

class YoloCoordinatePublisher(Node):
    def __init__(self):
        super().__init__('yolo_coordinate_publisher')

        # Load YOLOv8 model
        self.model = YOLO('/home/tycho/pi4_ws/yolov8n.onnx')
        self.get_logger().info("Loaded YOLOv8 model")

        # Camera capture
        self.cap = cv2.VideoCapture(0)

        # Publisher for detected coordinates
        self.publisher_ = self.create_publisher(String, 'coordinates_topic', 10)

        # Publisher for zone update acknowledgment
        self.ack_publisher = self.create_publisher(String, 'ack_zone', 10)

        # Service client for triggering CSV zone update
        self.client = self.create_client(Trigger, 'csv_zone_trigger')

        # Wait for the service to be available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        
        self.get_logger().info('Service "csv_zone_trigger" is available.')

        # Initialize the zone with default values
        self.upperleft = (0, 0)
        self.upperright = (640, 0)
        self.lowerright = (640, 480)
        self.lowerleft = (0, 480)
        self.zone_contour = self.get_zone_contour()

        # Subscribe to CSV data once the trigger is requested
        self.create_subscription(String, 'csv_zone_data', self.csv_data_callback, 10)

        # Send trigger request to start the process
        self.update_zone_from_service()

        # Timer for object detection
        self.timer = self.create_timer(1.0, self.detect_objects)
        self.get_logger().info("YOLOv8 Node Started")

    def get_zone_contour(self):
        return np.array([self.upperleft, self.upperright, self.lowerright, self.lowerleft], dtype=np.int32)

    def update_zone_from_service(self):
        request = Trigger.Request()
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f"Trigger request response: {future.result().message}")
        else:
            self.get_logger().error("Service call failed.")

    def csv_data_callback(self, msg):
        self.get_logger().info(f"Received CSV data: {msg.data}")
        self.update_zone_from_csv(msg.data)
        self.get_logger().info("csv_data_callback called")

    def update_zone_from_csv(self, csv_data):
        try:
            if not csv_data:
                self.get_logger().warn("Received empty CSV data.")
                return

            self.get_logger().info(f"CSV Data to be parsed: {csv_data}")

            df = pd.read_json(csv_data)
            if df.empty or len(df) == 0:
                self.get_logger().warn("Received empty DataFrame after parsing CSV data.")
                return

            row = df.iloc[0]
            self.upperleft = eval(row['UpperLeft'])
            self.upperright = eval(row['UpperRight'])
            self.lowerleft = eval(row['LowerLeft'])
            self.lowerright = eval(row['LowerRight'])

            self.zone_contour = self.get_zone_contour()
            self.get_logger().info(f"Updated zone: {self.zone_contour.tolist()}")

            ack_msg = String()
            ack_msg.data = "Zone updated successfully"
            self.ack_publisher.publish(ack_msg)
            self.get_logger().info("Published zone update acknowledgment")

        except Exception as e:
            self.get_logger().error(f"Failed to parse CSV zone data: {e}")

    def is_inside_zone(self, x, y):
        point = (int(x), int(y))
        return cv2.pointPolygonTest(self.zone_contour, point, False) >= 0

    def detect_objects(self):
        start_time = time.time()

        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Failed to capture image")
            return

        # Draw the zone contour
        # cv2.polylines(frame, [self.zone_contour], isClosed=True, color=(0, 255, 255), thickness=2)

        results = self.model(frame, verbose=False)[0]
        detections = results.boxes

        coordinates = []
        robot_detected = False
        people_count = 0

        for box in detections:
            cls_id = int(box.cls[0].item())
            class_name = self.model.names[cls_id]

            if class_name not in ['person', 'robot']:
                continue  # Skip anything that's not a person or robot

            xyxy = box.xyxy[0].cpu().numpy()
            x_center = (xyxy[0] + xyxy[2]) / 2
            y_bottom = xyxy[3]
            inside = self.is_inside_zone(x_center, y_bottom)

            # color = (0, 255, 0) if inside else (0, 0, 255)  # Green if inside, red if outside
            # cv2.circle(frame, (int(x_center), int(y_bottom)), radius=5, color=color, thickness=-1)

            if class_name == 'robot' and not robot_detected:
                if inside:
                    coordinates.append((x_center, y_bottom, 'R'))
                    robot_detected = True

            elif class_name == 'person' and people_count < 9:
                if inside:
                    coordinates.append((x_center, y_bottom, 'P'))
                    people_count += 1

        while len(coordinates) < 10:
            coordinates.append((0, 0, None))

        msg = String()
        msg.data = str(coordinates)
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published coordinates: {msg.data}")

        # Show visualization window
        # cv2.imshow("YOLOv8 Zone Visualization", frame)
        # cv2.waitKey(1)
        end_time = time.time()
        elapsed = end_time - start_time
        self.get_logger().info(f"detect_objects() took {elapsed:.3f} seconds")

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = YoloCoordinatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
