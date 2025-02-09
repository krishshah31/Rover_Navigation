from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    motor_control_subscriber_node = Node(
        package="motor_control",
        executable="motor_control_node"
        #name="<your node new name>"                                                           #to change the node name
        #remappings=[
        #    ("<old topic name> e.g motor_command","<new remapped topic name>")                #to remap the node to a new topic
        #]
    )

    motor_control_demo_node = Node(
        package="motor_control",
        executable="motor_command_publisher",
        #parameters=[                                                                          #to change parameters
        #    {"<parameter name>": <value>}
        #]
    )

    ld.add_action(motor_control_subscriber_node)
    ld.add_action(motor_control_demo_node)

    return ld
