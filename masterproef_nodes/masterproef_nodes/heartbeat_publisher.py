# this code runs a node that publishes a message containing the node name and timestamp at a predefined interval rate, a monitor node should detect these messages and use it for error handling once it does not receive the messages anymore

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class HeartbeatPublisher(Node):
    def __init__(self):
        super().__init__('heartbeat_publisher')
        self.publisher_ = self.create_publisher(String, '/heartbeat', 10)
        self.timer = self.create_timer(1.0, self.publish_heartbeat)  # 1Hz
        self.node_name = self.get_name()

    def publish_heartbeat(self):
        msg = String()
        msg.data = f"{self.node_name},{self.get_clock().now().seconds_nanoseconds()[0]}"  # Node name + timestamp
        self.publisher_.publish(msg)
        self.get_logger().info(f"Sent heartbeat from {self.node_name}")

def main(args=None):
    rclpy.init(args=args)
    node = HeartbeatPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()