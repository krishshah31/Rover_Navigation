#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import termios
import tty

class KeyboardControlNode(Node):
    def __init__(self):
        super().__init__('keyboard_control_node')
        self.publisher_ = self.create_publisher(String, 'motor_command', 10)
        self.get_logger().info('Keyboard Control Node Initialized')
        self.run()

    def get_key(self):
        """Capture a single key press."""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key

    def run(self):
        """Capture keyboard inputs and publish commands."""
        self.get_logger().info("Use W/A/S/D to control the rover. Space to stop. Q to quit.")
        while rclpy.ok():
            key = self.get_key()
            command = None

            if key.lower() == 'w':
                command = "FORWARD"
            elif key.lower() == 's':
                command = "BACKWARD"
            elif key.lower() == 'a':
                command = "LEFT"
            elif key.lower() == 'd':
                command = "RIGHT"
            elif key == 'h':
                command = "STOP"
            elif key.lower() == 'q':
                self.get_logger().info("Exiting keyboard control.")
                break

            if command:
                msg = String()
                msg.data = command
                self.publisher_.publish(msg)
                self.get_logger().info(f'Published command: {command}')


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControlNode()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()