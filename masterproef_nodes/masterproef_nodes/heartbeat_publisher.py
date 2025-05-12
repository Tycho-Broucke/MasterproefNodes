# RASPI4 NODE
#
# this code runs a node that publishes a message containing the node name and timestamp at a predefined interval rate
# a monitor node should detect these messages and use it for error handling once it does not receive the messages anymore
#
# the log statements that will be called in each iteration are commented out to keep the memory from filling up

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
        # self.get_logger().info(f"Sent heartbeat from {self.node_name}")

def main(args=None):
    rclpy.init(args=args)
    node = HeartbeatPublisher()
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