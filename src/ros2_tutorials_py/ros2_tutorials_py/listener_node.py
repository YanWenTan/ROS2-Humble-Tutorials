import os

#rclpy is python version of ROS2 Client Library (RCL), needed for EVERYTHING related to ROS2
import rclpy

#import class Node for our self-created classes to inherit from
from rclpy.node import Node

#import message type publisher/subscriber uses, in this case string
from std_msgs.msg import String

from custom_interfaces.srv import AddTwoInts
from functools import partial

#Self-created Node, inherits from class in the brackets, in this case Node
class ListenerNode(Node):

    def __init__(self):
        #Initialise base class, give a name to register the node in brackets
        super().__init__("listener_Node")

        #Declare parameter, first argument must be the same name as the parameter in launch file
        #If no parameter is set, will use default value
        self.declare_parameter("topic", value= "listener_topic")
        #Get value of parameter and convert it to string value
        topic_name = self.get_parameter("topic").get_parameter_value().string_value

        #Create Listener, must include callback to run whenever message is received from topic
        #Parameter value can be used instead of hard-coded topic name
        self.counter = 0
        self.listener = self.create_subscription(String, topic_name, self.listener_callback, 10)

    def listener_callback(self, msg):
        self.get_logger().info(f"Received Message: {msg.data}")



def main():
    #Steps here are the same as that of talker_node except for running listener_node instead
    rclpy.init(args= None)
    listener_node = ListenerNode()
    rclpy.spin(listener_node)
    listener_node.destroy_node()
    rclpy.shutdown()

if (__name__ == "__main__"):
    main()
