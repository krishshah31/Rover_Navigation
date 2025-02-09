import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion, Twist
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from scipy.spatial.transform import Rotation as R
from motor_control.roboclaw_3 import roboclaw # Ensure this is available in your Python environment


class EncoderOdom(Node):
    def __init__(self):
        super().__init__('roboclaw_node')

        # Robot parameters
        self.declare_parameter('ticks_per_meter', 4342.2)
        self.declare_parameter('base_width', 0.315)
        self.declare_parameter('dev', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('address', 128)
        self.declare_parameter('max_speed', 2.0)

        self.TICKS_PER_METER = self.get_parameter('ticks_per_meter').value
        self.BASE_WIDTH = self.get_parameter('base_width').value
        self.dev = self.get_parameter('dev').value
        self.baud = self.get_parameter('baud').value
        self.address = self.get_parameter('address').value
        self.MAX_SPEED = self.get_parameter('max_speed').value

        # State variables
        self.cur_x = 0.0
        self.cur_y = 0.0
        self.cur_theta = 0.0
        self.last_enc_left = 0
        self.last_enc_right = 0
        self.last_time = self.get_clock().now()

        # Publishers and TF Broadcaster
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscriber for velocity commands
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # Initialize RoboClaw
        self.init_roboclaw()

        self.get_logger().info("RoboClaw Node Initialized.")

    def init_roboclaw(self):
        try:
            roboclaw.Open(self.dev, self.baud)
            version = roboclaw.ReadVersion(self.address)
            if version[0]:
                self.get_logger().info(f"RoboClaw Version: {version[1]}")
            else:
                self.get_logger().warn("Failed to read RoboClaw version.")
            roboclaw.ResetEncoders(self.address)
        except Exception as e:
            self.get_logger().fatal(f"Could not connect to RoboClaw: {e}")
            self.destroy_node()

    @staticmethod
    def normalize_angle(angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def update_odometry(self, enc_left, enc_right):
        left_ticks = enc_left - self.last_enc_left
        right_ticks = enc_right - self.last_enc_right
        self.last_enc_left = enc_left
        self.last_enc_right = enc_right

        dist_left = left_ticks / self.TICKS_PER_METER
        dist_right = right_ticks / self.TICKS_PER_METER
        dist = (dist_right + dist_left) / 2.0

        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds * 1e-9
        self.last_time = current_time

        if left_ticks == right_ticks:  # Straight-line motion
            d_theta = 0.0
            self.cur_x += dist * math.cos(self.cur_theta)
            self.cur_y += dist * math.sin(self.cur_theta)
        else:  # Rotational motion
            d_theta = (dist_right - dist_left) / self.BASE_WIDTH
            r = dist / d_theta
            self.cur_x += r * (math.sin(self.cur_theta + d_theta) - math.sin(self.cur_theta))
            self.cur_y -= r * (math.cos(self.cur_theta + d_theta) - math.cos(self.cur_theta))
            self.cur_theta = self.normalize_angle(self.cur_theta + d_theta)

        vel_x = dist / dt if dt > 0 else 0.0
        vel_theta = d_theta / dt if dt > 0 else 0.0

        self.publish_odom(self.cur_x, self.cur_y, self.cur_theta, vel_x, vel_theta)

    def publish_odom(self, cur_x, cur_y, cur_theta, vx, vth):
        # Create odometry message
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'

        # Set position
        odom.pose.pose.position.x = cur_x
        odom.pose.pose.position.y = cur_y
        odom.pose.pose.position.z = 0.0
        quat = R.from_euler('z', cur_theta).as_quat()
        odom.pose.pose.orientation.x = quat[0]
        odom.pose.pose.orientation.y = quat[1]
        odom.pose.pose.orientation.z = quat[2]
        odom.pose.pose.orientation.w = quat[3]

        # Set velocity
        odom.child_frame_id = 'base_link'
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = vth

        # Publish odometry message
        self.odom_pub.publish(odom)

        # Broadcast TF
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = cur_x
        t.transform.translation.y = cur_y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]
        self.tf_broadcaster.sendTransform(t)

    def cmd_vel_callback(self, twist_msg):
        """Handle velocity commands."""
        linear_x = twist_msg.linear.x
        if linear_x > self.MAX_SPEED:
            linear_x = self.MAX_SPEED
        elif linear_x < -self.MAX_SPEED:
            linear_x = -self.MAX_SPEED

        vr = linear_x + twist_msg.angular.z * self.BASE_WIDTH / 2.0
        vl = linear_x - twist_msg.angular.z * self.BASE_WIDTH / 2.0

        vr_ticks = int(vr * self.TICKS_PER_METER)
        vl_ticks = int(vl * self.TICKS_PER_METER)

        roboclaw.SpeedM1M2(self.address, vr_ticks, vl_ticks)


def main(args=None):
    rclpy.init(args=args)
    node = EncoderOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
