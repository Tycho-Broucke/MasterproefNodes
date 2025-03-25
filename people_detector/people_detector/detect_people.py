import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
import cv2
import torch
import numpy as np

class PeopleDetectorNode(Node):
    def __init__(self):
        super().__init__('people_detector')
        self.publisher_ = self.create_publisher(Int32, 'people_count', 10)
        self.timer = self.create_timer(0.1, self.detect_people)
        self.cap = cv2.VideoCapture(0)  # Open USB camera
        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)  # Load YOLOv5
        self.get_logger().info("People Detector Node Started")

    def detect_people(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Failed to capture image")
            return

        results = self.model(frame)
        detections = results.pandas().xyxy[0]  # Get detections
        people_count = len(detections[detections['name'] == 'person'])  # Count people

        msg = Int32()
        msg.data = people_count
        self.publisher_.publish(msg)
        self.get_logger().info(f"People detected: {people_count}")

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = PeopleDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
    finally:
        rclpy.shutdown()
