import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class odompub(Node):
    def __init__(self):
        super().__init__('odom_pub')

        # Subscribe to the original ZED odometry topic
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/zed/zed_node/odom',
            self.odom_callback,
            10
        )

        # Publish the corrected odometry topic
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)

    def odom_callback(self, msg):
        """Modify covariance matrix to remove negative values."""
        # Ensure all covariance values are non-negative
        fixed_covariance = [max(0.00000000001, x) for x in msg.pose.covariance]
        msg.pose.covariance = fixed_covariance
        msg.twist.covariance = fixed_covariance

        # Publish the corrected odometry
        self.odom_publisher.publish(msg)
        self.get_logger().info("Published corrected odometry.")

def main(args=None):
    rclpy.init(args=args)
    node = odompub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
