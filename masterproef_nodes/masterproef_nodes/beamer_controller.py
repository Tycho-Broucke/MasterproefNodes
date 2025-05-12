# RASPI4 NODE
#
# this code runs a node that subscribes to the quiz topic to receive messages that contain the state of the quiz
# based on the state it receives, it either turns on or off the beamer
# the beamer is connected to the pi4 with an ethernet cable, and is controller with via a socket
# both the beamer and pi4 have static eth ip addresses in the same subnet
#
# the log statements that will be called in each iteration are commented out to keep the memory from filling up

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import socket

PROJECTOR_IP = "192.168.1.20"  # Replace with your projector's IP
PJLINK_PORT = 4352  # PJLink uses port 4352

class BeamerController(Node):
    def __init__(self):
        super().__init__('beamer_controller')
        self.subscription = self.create_subscription(
            String,
            'quiz',
            self.listener_callback,
            10)
        self.get_logger().info('Beamer Controller has been started.')

    def send_command(self, command):
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.connect((PROJECTOR_IP, PJLINK_PORT))
                initial_response = s.recv(1024).decode()

                if "PJLINK 0" in initial_response:  # No authentication required
                    s.sendall(command.encode('utf-8'))
                    response = s.recv(1024).decode()
                    # self.get_logger().info(f'Response: {response}')
                else:
                    self.get_logger().warning('Authentication required, but no password provided.')
        except Exception as e:
            self.get_logger().error(f'Error sending command: {str(e)}')

    def listener_callback(self, msg):
        message = msg.data
        # self.get_logger().info(f'Received message: {message}')

        if message == 'quiz_finished':
            self.send_command('%1POWR 0\r')  # Turn off projector
        elif message == 'drive_to_quiz_location':
            self.send_command('%1POWR 1\r')  # Turn on projector


def main(args=None):
    rclpy.init(args=args)
    node = BeamerController()
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
