import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState

class MotorCommandPublisher(Node):
    def __init__(self):
        super().__init__('motor_command_publisher')
        # Create a publisher to the 'motor_command' topic
        self.publisher_ = self.create_publisher(String, 'motor_command', 10)
        self.get_logger().info("Motor command publisher node has been started")

        # Create a subscriber to the '/puck_joint_states' topic
        self.subscriber_ = self.create_subscription(
            JointState,
            '/puck_joint_states',
            self.joint_state_callback,
            10
        )
        self.get_logger().info("Subscribed to /puck_joint_states")

    def joint_state_callback(self, msg):
        """
        Callback function to process JointState messages, negate positions,
        multiply them by 2, and print the results.
        """
        if msg.position:
            processed_positions = [(-pos * 1.9) for pos in msg.position]  # Negate and multiply by 2
            self.get_logger().info(f"puck wheel distance: {processed_positions}")
        else:
            self.get_logger().info("No positions available in the message.")

def main(args=None):
    rclpy.init(args=args)
    publisher_node = MotorCommandPublisher()
    
    try:
        rclpy.spin(publisher_node)  # Keep the node running
    finally:
        publisher_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
