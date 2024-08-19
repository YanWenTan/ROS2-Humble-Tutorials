#Unit Test not based on ROS2
#colcon test will also pick up this test and put results in log file

import unittest

class TestDummy(unittest.TestCase):
    def test_dummy(self):
        assert False