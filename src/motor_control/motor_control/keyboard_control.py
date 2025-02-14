import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import termios
import tty

class KeyboardControlNode(Node):
    def __init__(self):
        super().__init__('keyboard_control_node')
        self.publisher = self.create_publisher(String, 'motor_command', 10)
        self.get_logger().info("Keyboard control node started. Use arrow keys to move, 's' to stop, and 'q' to quit.")

    def get_key(self):
        """Reads a single character from terminal input without pressing Enter."""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key

    def run(self):
        """Continuously read key presses and send commands."""
        while rclpy.ok():
            key = self.get_key()
            msg = String()

            if key == '\x1b':  # Detecting escape sequences for arrow keys
                key += self.get_key()
                key += self.get_key()

            if key == '\x1b[A':  # Up arrow key
                msg.data = "FORWARD 30 30"
            elif key == '\x1b[B':  # Down arrow key
                msg.data = "BACKWARD 30 30"
            elif key == '\x1b[D':  # Left arrow key
                msg.data = "LEFT 30 30"
            elif key == '\x1b[C':  # Right arrow key
                msg.data = "RIGHT 30 30"
            elif key == 's':  # 's' key for stop
                msg.data = "STOP 0 0"
            elif key == 'r':  # 'r' key for recover mode
                msg.data = "RECOVER 0 0"
            elif key == 'q':  # Quit key
                self.get_logger().info("Exiting keyboard control.")
                break
            else:
                continue  # Ignore other keys

            self.publisher.publish(msg)
            self.get_logger().info(f"Sent command: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    keyboard_control_node = KeyboardControlNode()
    try:
        keyboard_control_node.run()
    except KeyboardInterrupt:
        pass
    finally:
        keyboard_control_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
