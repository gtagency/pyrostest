import unittest

from std_msgs.msg import Int32

import pyrostest

class TestAddOne(pyrostest.RosTest):
    @pyrostest.launch_node('pyrostest', 'add_one.py')
    def test_add_one_node(self):
        with pyrostest.mock_pub('subber', Int32, queue_size=0) as sub:
            with pyrostest.check_topic('add_one', Int32) as out:
                sub.send(Int32(5))
                assert out.message.data == 6

