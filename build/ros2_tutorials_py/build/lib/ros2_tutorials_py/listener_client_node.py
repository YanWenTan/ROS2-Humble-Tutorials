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
        #Create Listener, must include callback to run whenever message is received from topic
        self.counter = 0
        self.listener = self.create_subscription(String, "topic", self.listener_callback, 10)

    def listener_callback(self, msg):
        self.get_logger().info(f"Received Message: {msg.data}")
        #Calls service
        self.call_service()



    def call_service(self):
        #Create client, SERVICE NAME MUST BE SAME FOR BOTH CLIENT AND SERVICE NODES!
        self.client = self.create_client(AddTwoInts, "add_two_ints")
        #While service is not active, client will wait
        while not self.client.wait_for_service(timeout_sec= 1.0):
            self.get_logger().info("Service not available, waiting...")

        #Create the request and fill in data
        self.request = AddTwoInts.Request()
        self.request.a = self.counter
        self.request.b = self.counter + 1
        
        #Make async request to service
        #'future' is a handle of the state of the request (whether it is completed, results etc.)
        self.future = self.client.call_async(self.request)
        self.get_logger().info(f"Send Request: {self.request.a} + {self.request.b}")
        #Callback will run when service returns a response
        self.future.add_done_callback(partial(self.service_callback))
        self.counter += 1

    def service_callback(self, future):
        try:
            #response will take results from future as AddTwoInt data structure (int a, int b, int sum)
            response = self.future.result()
        except Exception as e:
            self.get_logger().info(f"Service call failed: {e}")
        else:
            self.get_logger().info(f"Response is {response.sum}")



def main():
    #Steps here are the same as that of talker_node except for running listener_node instead
    rclpy.init(args= None)
    listener_node = ListenerNode()
    rclpy.spin(listener_node)
    listener_node.destroy_node()
    rclpy.shutdown()

if (__name__ == "__main__"):
    main()
