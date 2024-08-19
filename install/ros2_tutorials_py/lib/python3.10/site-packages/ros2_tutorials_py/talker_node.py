import os

#rclpy is python version of ROS2 Client Library (RCL), needed for EVERYTHING related to ROS2
import rclpy

#import class Node for our self-created classes to inherit from
from rclpy.node import Node

#import message type publisher/subscriber uses, in this case string
from std_msgs.msg import String

#Self-created Node, inherits from class in the brackets, in this case Node
class TalkerNode(Node):

    def __init__(self):
    #Initialise base class, give a name to register the node in brackets
        super().__init__("talker_node")

        #Declare parameter, first argument must be the same name as the parameter in launch file
        #If no parameter is set, will use default value
        self.declare_parameter("topic", value= "talker_topic")
        #Get value of parameter and convert it to string value
        topic_name = self.get_parameter("topic").get_parameter_value().string_value

        #Create publisher, API inherited from Node class, last argument is queue size, in this case 10
        #Parameter value can be used instead of hard-coded topic name
        self.pub = self.create_publisher(String, topic_name, 10)
        #Create Timer to publish message, API inherited from Node class
        timer_period = 1.0
        self.timer = self.create_timer(timer_period, self.talker_callback)
        self.counter = 0
    
    #Timer callback
    def talker_callback(self):
        #Create message
        msg = String()
        msg.data = f"Hello World {self.counter}"
        self.counter += 1

        #Publish message
        self.pub.publish(msg)

        #Log data sent
        self.get_logger().info(f"Publishing {msg.data}")




def main():
    #Initialises ROS2, responsible for creating and destroying nodes
    rclpy.init(args = None)

    #Create Node based on self-created class
    talker_node = TalkerNode()

    #Start Node, rclpy.spin allows node to keep running and checks for events on subscribed topics and service/direct callbacks
    #Responsible for calling the publisher callback every 0.5s in this case
    rclpy.spin(talker_node)

    #Destroy Node
    talker_node.destroy_node()

    #Shutdown rclpy
    rclpy.shutdown()



if (__name__ == "__main__"):
    main()