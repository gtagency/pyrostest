:fire: pyrostest :fire:
=========

The most :fire: lit :fire: ros testing framework.

Pyrostest is a (more) pythonic framework for writing ros tests.

## Why pyrostest?

Writing rostests and rostunit tests is annoying. You have to write launch files
for all of your tests and then do strange things in your unittests and then
somehow the rostests and unittests will communicate. To be honest, I never could
get rostest to work. So I built something better. `pyrostest` allows you to test
your ros nodes in pure python, no need for xml files.

## Usage

Here's a minimal example of a pyrostest test. Assume you have a ros node,
`add_one` in the ros package `pkg`. The node subscribes to `input`, a Float64
message, and published to `output`, another Float64, it adds one to the input
and publishes it. This would test that functionality

```python

from pyrostest import RosTest, launch_node, mock_node
from std_msgs.msg import Float64

class TestAddOne(RosTest):
    @launch_node('pkg', 'add_one')
    def test_add_one(self):
        with self.mock_pub('/input', Flaot64, queue_size=0) as p:
            with self.check_topic('/output', Float64) as r:
                p.send(Float64(7))
                assert r.message.data == 8  # or self.assertEqual

```

This code will spin up an isolated instance of Roscore, as well as all of the
nodes needed for testing. It will then publish the float 7, and wait until the
data has been recieved, and assert about its value.

