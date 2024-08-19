from launch import LaunchDescription
from launch_ros.actions import Node

#Set parameter
TOPIC = "chatter"

def generate_launch_description():

    ld = LaunchDescription()

    talker_node = Node(
        package = "ros2_tutorials_py",
        executable = "talker_node",
        name = "talker_node",
        #Pass parameters to the node
        parameters=[{
            "topic" : TOPIC
        }]
    )

    listener_node = Node(
        package = "ros2_tutorials_py",
        executable = "listener_node",
        name = "listener_node",
        #Pass parameters to the node
        parameters=[{
            "topic" : TOPIC
        }]
    )

    ld.add_action(listener_node)
    ld.add_action(talker_node)

    return ld