import rclpy
import pandas as pd
import os
import json
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler

class CSVWatcher(Node):
    def __init__(self):
        super().__init__('csv_watcher')

        # Path to the CSV file (UPDATE THIS)
        self.csv_path = "/home/tycho/pi4_ws/data/parameters.csv"

        # Publisher for CSV data
        self.publisher_ = self.create_publisher(String, 'csv_data', 10)

        # Subscriber to listen for acknowledgment messages on the 'ack_zone' topic
        self.create_subscription(String, 'ack_zone', self.ack_zone_callback, 10)

        # Service to handle the trigger for CSV publishing
        self.csv_zone_trigger_service = self.create_service(Trigger, 'csv_zone_trigger', self.csv_zone_trigger_callback)

        # File watcher setup
        self.event_handler = CSVFileHandler(self.csv_path, self.publisher_, self.get_logger())
        self.observer = Observer()
        self.observer.schedule(self.event_handler, os.path.dirname(self.csv_path), recursive=False)
        self.observer.start()

        self.get_logger().info(f"Watching CSV file: {self.csv_path}")

    def ack_zone_callback(self, msg):
        """Callback function to handle the acknowledgment messages."""
        self.get_logger().info(f"Received acknowledgment: {msg.data}")

    def csv_zone_trigger_callback(self, request, response):
        """Callback function for the csv_zone_trigger service."""
        self.get_logger().info("CSV zone trigger service called. Publishing CSV data.")
        
        # Publish the CSV content
        self.publish_csv()

        # Respond with a success message
        response.success = True
        response.message = "CSV data published successfully"
        return response

    def publish_csv(self):
        """Reads CSV and publishes its contents."""
        if not os.path.exists(self.csv_path):
            self.get_logger().error(f"CSV file not found: {self.csv_path}")
            return
        
        try:
            df = pd.read_csv(self.csv_path)
            csv_data = df.to_json()  # Convert to JSON string format
            
            msg = String()
            msg.data = csv_data
            self.publisher_.publish(msg)

            self.get_logger().info(f"CSV file updated! Published new data: {csv_data}")

        except Exception as e:
            self.get_logger().error(f"Error reading CSV: {e}")

    def destroy_node(self):
        """Stop file observer when shutting down."""
        self.observer.stop()
        self.observer.join()
        super().destroy_node()

class CSVFileHandler(FileSystemEventHandler):
    """Handles file change events for the CSV file."""
    def __init__(self, csv_path, publisher, logger):
        self.csv_path = csv_path
        self.publisher = publisher
        self.logger = logger

    def on_modified(self, event):
        """Triggered when the CSV file is modified."""
        if event.src_path == self.csv_path:
            self.publish_csv()

    def publish_csv(self):
        """Reads CSV and publishes its contents."""
        if not os.path.exists(self.csv_path):
            self.logger.error(f"CSV file not found: {self.csv_path}")
            return
        
        try:
            df = pd.read_csv(self.csv_path)
            csv_data = df.to_json()  # Convert to JSON string format
            
            msg = String()
            msg.data = csv_data
            self.publisher.publish(msg)

            self.logger.info(f"CSV file updated! Published new data: {csv_data}")

        except Exception as e:
            self.logger.error(f"Error reading CSV: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = CSVWatcher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
