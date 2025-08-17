import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image as ROSImage
from masterproef_interfaces.srv import GetStitchedImage
from ultralytics import YOLO
from cv_bridge import CvBridge
import numpy as np
import pandas as pd
import time
import cv2
import os

# Toggle display of images
SHOW_IMAGES = True

class YoloNodeSpeedTest(Node):
    def __init__(self):
        super().__init__('yolo_node_speed_test')

        # YOLO model path
        model_path = '/home/tycho/pi4_ws/yolov8n.onnx'

        if not os.path.isfile(model_path):
            self.get_logger().warn("ONNX model not found, downloading...")
            try:
                model = YOLO('yolov8n.pt')
                model.export(format='onnx')
                os.rename('yolov8n.onnx', model_path)
                self.get_logger().info("ONNX model exported successfully")
            except Exception as e:
                self.get_logger().error(f"Failed to export ONNX model: {e}")
                exit(1)

        self.model = YOLO(model_path)
        self.get_logger().info("Loaded YOLOv8 model")

        self.bridge = CvBridge()

        # Publishers
        self.publisher_ = self.create_publisher(String, 'coordinates_topic', 10)
        self.ack_publisher = self.create_publisher(String, 'ack_zone', 10)

        # Service clients
        self.zone_client = self.create_client(Trigger, 'csv_zone_trigger')
        while not self.zone_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for CSV zone service...')

        self.image_client = self.create_client(GetStitchedImage, 'get_stitched_image')
        while not self.image_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for stitched image service...')

        # Subscriber
        self.create_subscription(String, 'csv_zone_data', self.csv_data_callback, 10)

        # Default zone
        self.upperleft = (0, 0)
        self.upperright = (640, 0)
        self.lowerright = (640, 480)
        self.lowerleft = (0, 480)
        self.zone_contour = self.get_zone_contour()

        self.update_zone_from_service()

        # Speed test variables
        self.warmup_cycles = 10
        self.total_cycles = 200
        self.cycle_counter = 0
        self.inference_times = []
        self.total_times = []
        self.extra_times = []

        # Start image requests
        self.request_image()

    def get_zone_contour(self):
        return np.array([self.upperleft, self.upperright, self.lowerright, self.lowerleft], dtype=np.int32)

    def update_zone_from_service(self):
        request = Trigger.Request()
        future = self.zone_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result():
            self.get_logger().info(f"Trigger response: {future.result().message}")
        else:
            self.get_logger().error("Zone service call failed.")

    def csv_data_callback(self, msg):
        try:
            if not msg.data:
                self.get_logger().warn("Empty CSV zone data received.")
                return

            df = pd.read_json(msg.data)
            if df.empty:
                self.get_logger().warn("Parsed CSV data is empty.")
                return

            row = df.iloc[0]
            self.upperleft = eval(row['UpperLeft'])
            self.upperright = eval(row['UpperRight'])
            self.lowerleft = eval(row['LowerLeft'])
            self.lowerright = eval(row['LowerRight'])
            self.zone_contour = self.get_zone_contour()
            self.get_logger().info(f"Updated zone: {self.zone_contour.tolist()}")

            ack_msg = String()
            ack_msg.data = "Zone updated successfully in yolo node"
            self.ack_publisher.publish(ack_msg)

        except Exception as e:
            self.get_logger().error(f"Failed to update zone from CSV: {e}")

    def is_inside_zone(self, x, y):
        return cv2.pointPolygonTest(self.zone_contour, (int(x), int(y)), False) >= 0

    def request_image(self):
        self.image_request_start = time.time()
        request = GetStitchedImage.Request()
        future = self.image_client.call_async(request)
        future.add_done_callback(self.image_response_callback)

    def image_response_callback(self, future):
        try:
            response = future.result()
            if not response.success:
                self.get_logger().error(f"Failed to get stitched image: {response.message}")
            else:
                frame = self.bridge.imgmsg_to_cv2(response.image, desired_encoding='bgr8')
                self.detect_objects(frame)
        except Exception as e:
            self.get_logger().error(f"Error in image response callback: {e}")
        self.request_image()

    def detect_objects(self, frame):
        self.cycle_counter += 1
        start = time.time()
        results = self.model(frame, verbose=False)[0]
        yolo_duration = time.time() - start

        detections = results.boxes
        coordinates = []
        robot_detected = False
        people_count = 0

        for box in detections:
            cls_id = int(box.cls[0].item())
            class_name = self.model.names[cls_id]
            if class_name not in ['person', 'robot']:
                continue

            xyxy = box.xyxy[0].cpu().numpy()
            x_center = (xyxy[0] + xyxy[2]) / 2
            y_bottom = xyxy[3]
            inside = self.is_inside_zone(x_center, y_bottom)

            if class_name == 'robot' and not robot_detected and inside:
                coordinates.append((x_center, y_bottom, 'R'))
                robot_detected = True
            elif class_name == 'person' and people_count < 9 and inside:
                coordinates.append((x_center, y_bottom, 'P'))
                people_count += 1

        while len(coordinates) < 10:
            coordinates.append((0, 0, None))

        msg = String()
        msg.data = str(coordinates)
        self.publisher_.publish(msg)

        full_duration = time.time() - self.image_request_start

        if self.cycle_counter > self.warmup_cycles and self.cycle_counter <= self.warmup_cycles + self.total_cycles:
            self.inference_times.append(yolo_duration)
            self.total_times.append(full_duration)
            self.extra_times.append(full_duration - yolo_duration)

            if len(self.inference_times) == self.total_cycles:
                self.print_speed_stats()

        if SHOW_IMAGES:
            self.display_frame(frame, coordinates)

    def display_frame(self, frame, coordinates):
        cv2.polylines(frame, [self.zone_contour], isClosed=True, color=(0, 255, 0), thickness=2)
        for x, y, label in coordinates:
            if label == 'R':
                color = (0, 0, 255)
                text = 'Robot'
            elif label == 'P':
                color = (255, 0, 0)
                text = 'Person'
            else:
                continue
            cv2.circle(frame, (int(x), int(y)), 5, color, -1)
            cv2.putText(frame, text, (int(x), int(y) - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        cv2.imshow('YOLO Detection with Zone', frame)
        cv2.waitKey(1)

    def print_speed_stats(self):
        avg_yolo = sum(self.inference_times) / len(self.inference_times)
        avg_total = sum(self.total_times) / len(self.total_times)
        avg_extra = sum(self.extra_times) / len(self.extra_times)
        self.get_logger().info(f"Speed test complete over {self.total_cycles} cycles:")
        self.get_logger().info(f"Average YOLO time: {avg_yolo:.4f}s")
        self.get_logger().info(f"Average total cycle time: {avg_total:.4f}s")
        self.get_logger().info(f"Average extra time: {avg_extra:.4f}s")

    def destroy_node(self):
        if SHOW_IMAGES:
            cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = YoloNodeSpeedTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt received. Shutting down node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

