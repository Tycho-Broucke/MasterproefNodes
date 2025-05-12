# JETSON NODE
#
# this code runs a node that keeps track of the messages received by the heartbeat publisher
# once it does not recieve anymore messages, the connection is considered as lost
# it then gives an error, but this error still has to be handled (emergency stop should be performed)
#
# the log statements that will be called in each iteration are commented out to keep the memory from filling up

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

TIMEOUT = 3  # Seconds before considering a node as "lost"

class HeartbeatMonitor(Node):
    def __init__(self):
        super().__init__('heartbeat_monitor')
        self.subscription = self.create_subscription(String, '/heartbeat', self.heartbeat_callback, 10)
        self.last_heartbeat = {}  # Stores last received heartbeat timestamps
        self.lost_nodes = set()  # Stores nodes that are currently considered lost
        self.timer = self.create_timer(1.0, self.check_heartbeats)  # Check every second

    def heartbeat_callback(self, msg):
        """ Callback for receiving heartbeat messages. """
        node_name, timestamp = msg.data.split(",")
        timestamp = float(timestamp)

        # If the node was previously lost, mark it as reconnected
        if node_name in self.lost_nodes:
            self.get_logger().info(f"✅ Reconnected: {node_name}")
            self.lost_nodes.remove(node_name)  # Remove from lost nodes list

        # If this is a new node, log its first appearance
        if node_name not in self.last_heartbeat:
            self.get_logger().info(f"🔵 New node detected: {node_name}")

        # Update the last known heartbeat time
        self.last_heartbeat[node_name] = timestamp

    def check_heartbeats(self):
        """ Periodically check if nodes are still active. """
        current_time = time.time()
        for node, last_time in list(self.last_heartbeat.items()):
            if last_time is not None and current_time - last_time > TIMEOUT:
                if node not in self.lost_nodes:  # Only log once per lost node
                    self.get_logger().warn(f"⚠️ Lost connection to {node}")
                    self.lost_nodes.add(node)  # Mark as lost

def main(args=None):
    rclpy.init(args=args)
    node = HeartbeatMonitor()
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