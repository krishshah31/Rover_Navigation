import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from motor_control.roboclaw_3 import Roboclaw
from rclpy.qos import QoSProfile

# Modify this to match your RoboClaw configuration
ROBOCLAW_PORT = '/dev/ttyACM0'
BAUDRATE = 115200

class RoboClawCmdVelNode(Node):
    def __init__(self):
        super().__init__('roboclaw_cmd_vel_node')
        qos_profile = QoSProfile(depth=10)
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            qos_profile)
        self.roboclaw = Roboclaw(ROBOCLAW_PORT, BAUDRATE)
        self.roboclaw.open()
        self.track_width = 0.5  # Distance between the two tracks in meters, adjust as needed
        self.max_linear_velocity = 1.0  # Maximum linear velocity in m/s
        self.max_roboclaw_speed = 127  # Maximum speed for the RoboClaw motor controller

    def cmd_vel_callback(self, msg):
        # Extract linear and angular velocity
        linear_velocity = msg.linear.x
        angular_velocity = msg.angular.z

        # Calculate motor speeds based on differential drive kinematics
        left_motor_speed = linear_velocity - (angular_velocity * self.track_width / 2.0)
        right_motor_speed = linear_velocity + (angular_velocity * self.track_width / 2.0)

        # Scale motor speeds to match RoboClaw expected input (0 to 35)
        left_motor_command = int((abs(left_motor_speed) / self.max_linear_velocity) * self.max_roboclaw_speed)
        right_motor_command = int((abs(right_motor_speed) / self.max_linear_velocity) * self.max_roboclaw_speed)

        # Clamp motor commands to max speed range (0 to 35)
        left_motor_command = min(max(left_motor_command, 0), self.max_roboclaw_speed)
        right_motor_command = min(max(right_motor_command, 0), self.max_roboclaw_speed)

        # Determine direction and speed for left motor
        if left_motor_speed > 0:
            self.roboclaw.forward_m1(0x80, left_motor_command)
        elif left_motor_speed < 0:
            self.roboclaw.backward_m1(0x80, left_motor_command)
        else:
            self.roboclaw.forward_m1(0x80, 0)  # Stop motor

        # Determine direction and speed for right motor
        if right_motor_speed > 0:
            self.roboclaw.forward_m2(0x80, right_motor_command)
        elif right_motor_speed < 0:
            self.roboclaw.backward_m2(0x80, right_motor_command)
        else:
            self.roboclaw.forward_m2(0x80, 0)  # Stop motor

        self.get_logger().info(f'Sent motor commands: Left={left_motor_command}, Right={right_motor_command}')

def main(args=None):
    rclpy.init(args=args)
    node = RoboClawCmdVelNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
