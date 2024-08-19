#Import sys to get input numbers from terminal as arguments
import sys

import rclpy
from rclpy.node import Node

#Import custom interface from create package
from custom_interfaces.srv import AddTwoInts

#Service called from here is asynchronous
#eg. Node that calls this service does not have to wait until this service completes
#Deadlock will occur if you try to spin a node and wait for a synchronous service
class AsyncClientNode(Node):
    def __init__(self):
        super().__init__("async_addition_client")
        #Create client, SERVICE NAME MUST BE SAME FOR BOTH CLIENT AND SERVICE NODES!
        self.client = self.create_client(AddTwoInts, "add_two_ints")
        #While service is not active, client will wait
        while not self.client.wait_for_service(timeout_sec= 1.0):
            self.get_logger().info("Service not available, waiting...")
        #Once service is available, initialisation will finish

    def send_request(self):
        request = AddTwoInts.Request()
        request.a = int(sys.argv[1])
        request.b = int(sys.argv[2])
        #Make async request to service
        #'future' is a handle of the state of the request (whether it is completed, results etc.)
        self.future = self.client.call_async(request)



def main():
    rclpy.init(args= None)
    #Create Node
    addition_client = AsyncClientNode()
    addition_client.send_request()
    #Spin Node
    while rclpy.ok():
        rclpy.spin_once(addition_client)
        if addition_client.future.done():
            try:
                #response will take results from future as AddTwoInt data structure (int a, int b, int sum)
                response = addition_client.future.result()
            except Exception as e:
                addition_client.get_logger().info(f"Service call failed: {e}")
            else:
                addition_client.get_logger().info(f"Response is {response.sum}")
            break
    #Destroy Node
    addition_client.destroy_node()
    rclpy.shutdown()

if (__name__ == "__main__"):
    main()