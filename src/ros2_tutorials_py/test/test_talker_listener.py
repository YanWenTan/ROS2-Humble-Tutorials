#--- ROS2 Based Unit Testing ---

#Import OS for filepath stuff
import os
#Import sys for finding python executables
import sys
#Import time for trivial method (eg simulate timelapse)
import time
#Import unittest essential for framework for launch testing to work
import unittest
#Import uuid for trivial methods
import uuid

import launch
import launch_ros
import launch_ros.actions
import launch_testing.actions

import rclpy
import std_msgs.msg

#Next 2 lines needed to run colcon test from terminal
import pytest
@pytest.mark.rostest

#Launch feature node
def generate_test_description():
    # Get the file path for this file
    file_path = os.path.dirname(__file__)
    
    # Launch talker node in test mode
    talker_node = launch_ros.actions.Node(
        executable = sys.executable, #Python interpretator 
        arguments = [os.path.join(   #Path of source file with node
            file_path,
            "..", "ros2_tutorials_py", "talker_node.py")], #Go back one folder, then go to ros2_tutorial_py/talker_node.py
            additional_env = {"PYTHONUNBUFFERED": "1"}, #Ensures that python outputs (stdout, stderror) streams are sent straight to the terminal without being buffered
                                                        #Can see the output of application in real time
            #Set paramters
            parameters = [{
                "topic": "talker_chatter"
            }]
    )

    # Launch listener node in test mode
    listener_node = launch_ros.actions.Node(
        executable = sys.executable, #Python interpretator 
        arguments = [os.path.join(   #Path of source file with node
            file_path,
            "..", "ros2_tutorials_py", "listener_node.py")], #Go back one folder, then go to ros2_tutorial_py/talker_node.py
            additional_env = {"PYTHONUNBUFFERED": "1"}, #Ensures that python outputs (stdout, stderror) streams are sent straight to the terminal without being buffered
                                                        #Can see the output of application in real time
            #Set paramters
            parameters = [{
                "topic": "listener_chatter"
            }]
    )

    #Create Launch Description
    return(
        launch.LaunchDescription([
            #Launch talker and listener node
            talker_node,
            listener_node,
            #In this case, nothing needs to happen before these nodes are launched
            #Can start test right away without waiting for anything
            launch_testing.actions.ReadyToTest(),            
        ]),
        #Set the names of these 2 processes/nodes
        {
            "talker": talker_node,
            "listener": listener_node,
        }
    )

#Test nodes
class TestTalkerListenerLink(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        #Initialise ROS2
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        #Shutdown ROS2
        rclpy.shutdown()

    #Initialise a node to the counterpart of the tests
    #For publisher, we need a test node to subscribe to it
    #For subscriber, we need a test node to publish to it
    def setUp(self):
        #Create a ROS2 node for testing
        self.node = rclpy.create_node("test_talker_listener_link")
    
    #Destroy the node once test is complete
    def tearDown(self):
        self.node.destroy_node()
    
    #In this case for the talker node, we need a counterpart node to subscribe to it
    #MUST HAVE 'test_' IN FRONT! This allows it to be picked up by the test framework!
    #'talker' is given when setting the names of the nodes (line 65)

    #Testing framework automatically records all stdout and exit codes from launch processes
    #Made available via proc_info and proc_output, automatically added by test framework
    #We want to check if the talker node is outputting the correct stuff, so we query proc_output
    def test_talker_transmits(self, talker, proc_output):
        msg_rx = []
        #Using separated node instead of a Class
        sub = self.node.create_subscription(std_msgs.msg.String,
                                            "talker_chatter",
                                            #Lambda function because we don't need a separate callback
                                            #Will append whatever is published to msg_rx list
                                            lambda msg: msg_rx.append(msg),
                                            10)
        
        try:
            #Wait until talker transmits two messages over the ROS2 topic
            end_time = time.time() + 10
            while (time.time() < end_time):
                rclpy.spin_once(self.node, timeout_sec= 0.1)
                if len(msg_rx) > 2:
                    break
            #Assertion will be true if more than 2 messages were received
            #Will be false when above code times out and less than 2 messages were received
            self.assertGreater(len(msg_rx), 2)
            
            #Makes sure talker also outputs the same data via stdout
            #Checks if the message is the correct message from the correct process/node
            for msg in msg_rx:
                proc_output.assertWaitFor(
                    expected_output= msg.data, process= talker
                )
        finally:
            #Destroy subscription after test is complete
            self.node.destroy_subscription(sub)

    #Testing framework automatically records all stdout and exit codes from launch processes
    #Made available via proc_info and proc_output, automatically added by test framework
    #We want to check if the talker node is outputting the correct stuff, so we query proc_output
    def test_listener_receives(self, listener, proc_output):

        #Using separated node instead of a Class
        pub = self.node.create_publisher(std_msgs.msg.String, "listener_chatter", 10)
        time.sleep(2.0)

        #Publish message and check if listener node is putting the correct stuff on stdout
        try:
            msg = std_msgs.msg.String()
            msg.data = str(uuid.uuid4())
            #Test 10 times
            for _ in range(10):
                pub.publish(msg)
                #Checks whether listener node outputs same as test publisher
                success = proc_output.waitFor(
                    expected_output= msg.data,
                    process= listener,
                    timeout= 1.0,
                )
                #Break loop if success
                if success:
                    break
                #If fail, waits for time out
                assert success, "Waiting for output timeout"
        finally:
            self.node.destroy_publisher(pub)
        

