import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import torch
import cv2
import numpy as np

class YoloNode(Node):
    def __init__(self):
        super().__init__('yolo_node')
        self.publisher_ = self.create_publisher(String, 'coordinates_topic', 10)
        self.timer = self.create_timer(1.0, self.detect_objects)  # Timer triggers every 1 second
        self.cap = cv2.VideoCapture(0)  # Open USB camera
        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)  # Load YOLOv5
        self.get_logger().info("YOLO Node Started")

    def detect_objects(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Failed to capture image")
            return

        results = self.model(frame)  # Perform inference on the frame
        detections = results.pandas().xyxy[0]  # Get detection results

        # Initialize list of coordinates (first robot, then people)
        coordinates = []
        
        # Find the robot (assuming 'robot' is detected)
        robot_detected = False
        for _, det in detections.iterrows():
            if det['name'] == 'robot' and not robot_detected:
                # Robot is detected, add its bottom-middle coordinate
                x_center = (det['xmin'] + det['xmax']) / 2
                y_bottom = det['ymax']
                coordinates.append((x_center, y_bottom, 'R'))  # Robot coordinate
                robot_detected = True
                # Draw the robot on the frame, toggle for visualisation
                # cv2.putText(frame, "Robot", (int(x_center), int(y_bottom)), 
                #             cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2, cv2.LINE_AA)
                # cv2.circle(frame, (int(x_center), int(y_bottom)), 5, (0, 255, 0), -1)

        # Find people (add up to 9 people)
        people_count = 0
        for _, det in detections.iterrows():
            if det['name'] == 'person' and people_count < 9:
                # Person is detected, add its bottom-middle coordinate
                x_center = (det['xmin'] + det['xmax']) / 2
                y_bottom = det['ymax']
                coordinates.append((x_center, y_bottom, 'P'))  # Person coordinate
                people_count += 1
                # Draw the person on the frame, toggle for visualisation
                # cv2.putText(frame, "Person", (int(x_center), int(y_bottom)), 
                #             cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2, cv2.LINE_AA)
                # cv2.circle(frame, (int(x_center), int(y_bottom)), 5, (0, 0, 255), -1)

        # If less than 9 people, add (0, 0, None) as placeholder
        while len(coordinates) < 10:  # Total of 10 (1 robot + 9 people)
            coordinates.append((0, 0, None))  # Add null coordinates

        # Convert list of coordinates to string format
        coordinates_str = str(coordinates)
        
        # Publish the coordinates to the topic
        msg = String()
        msg.data = coordinates_str
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published coordinates: {coordinates_str}")

        # Show the frame with the coordinates visualized, toggle for visualisation
        # cv2.imshow('YOLO - Coordinates Visualization', frame)
        # cv2.waitKey(1)  # Add this to keep the window responsive

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()  # Close the OpenCV window
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = YoloNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
