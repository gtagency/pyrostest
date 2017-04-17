import unittest

from std_msgs.msg import Int64

import pyrostest

class TestAddOne(pyrostest.RosTest):
    @pyrostest.launch_node('pyrostest', 'add_one.py')
    def test_add_one_node(self):
        with pyrostest.mock_pub('subber', Int64, queue_size=0) as sub:
            with pyrostest.check_topic('add_one', Int64) as out:
                sub.send(Int64(5))
                assert out.message.data == 6

