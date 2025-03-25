# counter_publisher/counter_publisher.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class CounterPublisher(Node):
    def __init__(self):
        super().__init__('counter_publisher')
        self.publisher_ = self.create_publisher(Int32, 'counter', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.count = 0

    def timer_callback(self):
        msg = Int32()
        msg.data = self.count
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.count += 1

def main(args=None):
    rclpy.init(args=args)
    node = CounterPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
