import rclpy
from rclpy.node import Node
import time

#Import custom interface from create package
from custom_interfaces.srv import AddTwoInts

#Self-created Node, inherits from class in the brackets, in this case Node
class ServiceNode(Node):
    def __init__(self):
        #Initialise base class, give a name to register the node in brackets
        super().__init__("add_int_service")

        #Create Service, Data Structure of Service, Name of service and service callback
        #Service callback will be called when request is received
        self.service = self.create_service(AddTwoInts, "add_two_ints", self.add_two_ints_callback)

    #Two arguments, 'request' to hold information about the request being sent, 'response' is something this service will return
    #In this case, request holds the two integers, while response will have the sum of the two integers
    def add_two_ints_callback(self, request, response):
        self.get_logger().info(f"Incoming request:\na:{request.a} b:{request.b}")
        # time.sleep(1.0)
        response.sum = request.a + request.b
        self.get_logger().info(f"Returning response:\nsum:{response.sum}")
        return response




def main():
    rclpy.init(args= None)
    #Create Node
    service_node = ServiceNode()
    #Start Node, rclpy.spin allows node to keep running and checks for events on subscribed topics and service/direct callbacks
    rclpy.spin(service_node)
    #Destroy Node
    service_node.destroy_node()
    rclpy.shutdown()

if (__name__ == "__main__"):
    main()