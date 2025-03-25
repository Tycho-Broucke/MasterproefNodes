import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class CSVSubscriber(Node):
    def __init__(self):
        super().__init__('csv_subscriber')
        self.subscription = self.create_subscription(
            String,
            'csv_data',
            self.listener_callback,
            10
        )
        self.get_logger().info("CSV Subscriber Node Started")

    def listener_callback(self, msg):
        data = json.loads(msg.data)  # Convert JSON string back to dictionary
        self.get_logger().info(f"Received Updated CSV Data: {data}")

def main(args=None):
    rclpy.init(args=args)
    node = CSVSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
