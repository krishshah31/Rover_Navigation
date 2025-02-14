import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class MotorCommandPublisher(Node):
    def __init__(self):
        super().__init__('motor_command_publisher')
        # Create a publisher to the 'motor_command' topic
        self.publisher_ = self.create_publisher(String, 'motor_command', 10)
        self.get_logger().info("Motor command publisher node has been started")

    def publish_command(self, command, speed_m1, speed_m2):
        """Helper function to publish a motor command with specified speeds."""
        msg = String()
        msg.data = f"{command} {speed_m1} {speed_m2}"
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published command: {msg.data}")

    def gradual_acceleration(self, command, max_speed=38, step=2, delay=0.2, m1_offset=0, m2_offset=0):
        """Gradually increase speed to the max speed with synchronized startup."""
        speed_m1 = 20 + m1_offset
        speed_m2 = 20 + m2_offset
        
        while speed_m1 <= max_speed and speed_m2 <= max_speed:
            self.publish_command(command, speed_m1, speed_m2)
            time.sleep(delay)
            speed_m1 += step
            speed_m2 += step

    def gradual_deceleration(self, command, step=2, delay=0.2, min_speed=20):
        """Gradually decrease speed to stop. Adjusts for backward motion."""
        speed_m1 = 30 if command != "BACKWARD" else 25
        speed_m2 = 30 if command != "BACKWARD" else 25
        
        while speed_m1 > min_speed and speed_m2 > min_speed:
            self.publish_command(command, speed_m1, speed_m2)
            time.sleep(delay)
            speed_m1 -= step
            speed_m2 -= step
        
        self.publish_command('STOP', 0, 0)

    def run_demo_sequence(self):
        """Publish a series of commands to control the rover using time-based open-loop control with acceleration and deceleration."""
        
        # Move forward 1 meter with acceleration, hold, then decelerate
        self.gradual_acceleration('FORWARD', m1_offset=0, m2_offset=3)
        time.sleep(5)
        self.gradual_deceleration('FORWARD')
        time.sleep(1)

        # Stop before next manuver
        self.publish_command('STOP', 0, 0)
        self.get_logger().info("pause for next manuver")
        time.sleep(2)

        # Turn right 90 degree with acceleration, hold, then decelerate
        self.gradual_acceleration('RIGHT', m1_offset=0, m2_offset=-9)
        time.sleep(2)
        self.gradual_deceleration('RIGHT')
        time.sleep(1)

        # Stop before next manuver
        self.publish_command('STOP', 0, 0)
        self.get_logger().info("pause for next manuver")
        time.sleep(2)

        # Move forward 1 meter with acceleration, hold, then decelerate
        self.gradual_acceleration('FORWARD', m1_offset=0, m2_offset=3)
        time.sleep(5)
        self.gradual_deceleration('FORWARD')
        time.sleep(1)

        # Stop before next manuver
        self.publish_command('STOP', 0, 0)
        self.get_logger().info("pause for next manuver")
        time.sleep(2)

        # Turnaround right 360 degree with acceleration, hold, then decelerate
        self.gradual_acceleration('RIGHT', m1_offset=0, m2_offset=0)
        time.sleep(3.6)
        self.gradual_deceleration('RIGHT')
        time.sleep(1)

        # Stop before next manuver
        self.publish_command('STOP', 0, 0)
        self.get_logger().info("pause for next manuver")
        time.sleep(2)

        # Move forward 1 meter with acceleration, hold, then decelerate
        self.gradual_acceleration('FORWARD', m1_offset=0, m2_offset=3)
        time.sleep(5)
        self.gradual_deceleration('FORWARD')
        time.sleep(1)

        # Stop before next manuver
        self.publish_command('STOP', 0, 0)
        self.get_logger().info("pause for next manuver")
        time.sleep(2)

        # Turn left 90 degree with acceleration, hold, then decelerate
        self.gradual_acceleration('LEFT', m1_offset=-5, m2_offset=0)
        time.sleep(2)
        self.gradual_deceleration('LEFT')
        time.sleep(1)

        # Stop before next manuver
        self.publish_command('STOP', 0, 0)
        self.get_logger().info("pause for next manuver")
        time.sleep(2)

        # Move forward 1 meter with acceleration, hold, then decelerate
        self.gradual_acceleration('FORWARD', m1_offset=0, m2_offset=3)
        time.sleep(5)
        self.gradual_deceleration('FORWARD')
        time.sleep(1)

        # Stop before next manuver
        self.publish_command('STOP', 0, 0)
        self.get_logger().info("pause for next manuver")
        time.sleep(2)

        # Turnaround left 360 degree with acceleration, hold, then decelerate
        self.gradual_acceleration('LEFT', m1_offset=0, m2_offset=0)
        time.sleep(4)
        self.gradual_deceleration('LEFT')
        time.sleep(1)

        # Stop before next manuver
        self.publish_command('STOP', 0, 0)
        self.get_logger().info("pause for next manuver")
        time.sleep(2)

        # # Move backward with acceleration, hold, then decelerate (adjusted speed reduction)
        # self.gradual_acceleration('BACKWARD', m1_offset=1, m2_offset=0, max_speed=35.5)
        # time.sleep(5)
        # self.gradual_deceleration('BACKWARD', min_speed=15)
        # time.sleep(1)

        #Stop at the end
        self.publish_command('STOP', 0, 0)
        self.get_logger().info("Demo sequence complete")


def main(args=None):
    rclpy.init(args=args)
    publisher_node = MotorCommandPublisher()
    
    try:
        publisher_node.run_demo_sequence()
    finally:
        publisher_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

