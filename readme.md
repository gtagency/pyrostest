# ROS Test Utils

This library includes a number of tools for testing ros nodes in isolation
without having to deal with the normal rostesting library.

## API

The major components to `rostest_utils` are:

 - `RosTest`
 - `with_launch_file`
 - `launch_node`
 - `mock_pub`
 - `check_topic`

I'll go over each these in detail.

#### `RosTest`

This is a subclass of `unittest.Testcase`. It should be used for all
integration-style tests that involve spinning up nodes and having them
communicate via ROS. This class will automatically spin up an instance of Ros
for each test you run, and make sure that it is isolated and cleaned up
correctly.

#### `with_launch_file`

ROS requires launch files. Normally you would run something like `roslaunch
xyz.launch`, but you can't do that when you aren't using the roslaunch command
line tool, so this will do that for you. 

#### `launch_node`
Most RosTests are generally structured in this manner:

 1. Spin up a node under test
 2. Simulate some input
 3. Assert something about the output from that node

`launch_node` handles this first step, it will spin up a rosnode for the
entirety of the test.

A quick note on these two decorators: you can, at this time, only use a single
launch file, and you must run the launch file before any nodes. This is for icky
architectural reasons, but it means that

```python
@launch_node(package, name)
def test(...):
    pass

@with_launch_file(package, name)
@launch_node(package, name)
def test(...):
    pass

@with_launch_file(package, name)
@launch_node(package, name)
@launch_node(package, name)
def test(...):
    pass
```

Are all valid, but these are not (and will throw errors):

```python
@launch_node(package, name)
@with_launch_file(package, name)
def test(...):
    pass

@with_launch_file(package, name)
@with_launch_file(package, name)
@launch_node(package, name)
def test(...):
    pass
```

#### `mock_pub`

This is a tool that solves #2 from above. It allows you to mock a node
publishing some data. It is used as a context manager, so you create a context
and then use the node inside it like so:

```python
with mock_pub(topic, ros_message_type, queue_size) as pub:
    pub.send(ROS_MESSAGE_OBJECT)
```


#### `check_topic`

Similar to `mock_pub`, `check_topic` is a context manager that allows you to
recieve data on a topic.

```python
with check_topic(topic, ros_message_type) as ct:
    print(ct.message)
```

Because of how `check_topic` is implemented, ct.message may take an arbitrary
amount of time, but will only provide a value once a value is published onto the
monitored topic.

#### Usage

Here's some usage examples, annotated with some explanations.

```python

<imports>

class TestExample(RosTest):
    @with_launch_file('pkg', 'test_params.launch')
    # test_params.launch might contain a set of global config settings
    @launch_node('pkg', 'add_one.py')
    # this simple node subscribes to `number` and publishes to `result`, both
    # are of type Float64
    def test_simple_node(self):
        with mock_pub('/number', Flaot64, queue_size=0) as p:
            with check_topic('/result', Float64) as r:
                p.send(Float64(7))
                assert ct.message.data == 8


    @with_launch_file('pkg', 'test_params.launch')
    @launch_node('pkg', 'step_1.py')
    @launch_node('pkg', 'step_2.py')
    # in this case, step_1 subscribes to 'input', publishes to 'middle', and 
    # step_2 subscribes to 'middle' and publishes to 'result'. Each is otherwise
    # identical to add_one above.
    def test_chain_of_events(self):
        with mock_pub('/input', Flaot64, queue_size=0) as p:
            with check_topic('/result', Float64) as r:
                p.send(Float64(7))
                assert ct.message.data == 9

    # a real example: the bearting node only publishes after recieving its
    # second piece of data, so you need to send 2 to get a result.
    @with_launch_file('buzzmobile', 'test_params.launch')
    @launch_node('buzzmobile', 'bearing.py')
    def test_bearing_node(self):
        with mock_pub('/fix', NavSatFix, queue_size=0) as fix_node:
            with check_topic('/buzzmobile/bearing', Float64) as ct:
                # send mock data
                fix_node.send(NavSatFix(None, None, 33.636700, -84.427863, None, None, None))
                fix_node.send(NavSatFix(None, None, 39.029128, -111.838257, None, None, None))

                assert np.isclose(ct.message.data, 1.19212)
```




## How It Works

If you're just a user, no need for this section. If, however, you have the
misfortune of having to modify `rostest_utils`, I offer you my sympathy and my
knowledge.

This is structured similarly to the above, but as a deep dive into how each
component works:


#### `RosTest`

#### `with_launch_file`

#### `launch_node`

#### `mock_pub`

#### `check_topic`

