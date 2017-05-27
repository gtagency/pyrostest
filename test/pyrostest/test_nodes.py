from std_msgs.msg import Int32

import pyrostest

class TestAddOne(pyrostest.RosTest):
    @pyrostest.launch_node('pyrostest', 'add_one.py')
    def test_add_one_node_single(self):
        with self.mock_pub('/pub_val', Int32, queue_size=0) as sub:
            with self.check_topic('/pyrostest/add_one', Int32) as out:
                sub.send(Int32(5))
                assert out.message.data == 6

    @pyrostest.launch_node('pyrostest', 'add_one.py')
    def test_add_one_node_many(self):
        with self.mock_pub('/pub_val', Int32, queue_size=0) as sub:
            with self.check_topic('/pyrostest/add_one', Int32) as out:
                sub.send(Int32(5))
                assert out.message.data == 6
            with self.check_topic('/pyrostest/add_one', Int32) as out:
                sub.send(Int32(7))
                assert out.message.data == 8


class TestPubFoo(pyrostest.RosTest):
    @pyrostest.with_launch_file('pyrostest', 'launch.launch')
    @pyrostest.launch_node('pyrostest', 'pub_foo.py')
    def test_foo_node(self):
        with self.check_topic('/pub_val', Int32) as out:
            assert out.message.data == 7


class TestSequence(pyrostest.RosTest):
    @pyrostest.with_launch_file('pyrostest', 'launch.launch')
    @pyrostest.launch_node('pyrostest', 'pub_foo.py')
    @pyrostest.launch_node('pyrostest', 'add_one.py')
    def test_nodes_in_sequence(self):
        with self.check_topic('/pyrostest/add_one', Int32) as out:
            assert out.message.data == 8
