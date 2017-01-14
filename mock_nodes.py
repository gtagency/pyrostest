"""Utilities that deal with creating fake rosnodes for testing.

Contains tools to send and recieve data from fake nodes and topics.
"""

import contextlib
import functools
import time
import rospy

class TimeoutError(Exception):
    """Py3 shim, represents no response from a syscall.
    """
    pass



class NoMessage(Exception):
    """Exception for a lack of message in a Node.
    """
    pass

class MockNode(object)
    """Mock of a node object for testing.
    """
    def __init__(self, topic, msg_type, node):
        self.topic = topic
        self.msg_type = msg_type
        self.node = node

    def send(self, value):
        """Sends data to be publoshed to the mocked topic.
        """
        self.node.publish(value)
        # Just long enough to prevent out of order
        time.sleep(.1)

@contextlib.contextmanager
def mock_pub(topic, rosmsg_type, queue_size=1):
    """Mocks a node and cleans it up when done.
    """
    pub = rospy.Publisher(topic, rosmsg_type, queue_size=queue_size)
    yield MockNode(topic, rosmsg_type, pub)
    pub.unregister()


class TestNode(object):
    """Wrapper around a node used for testing.
    """

    def __init__(self, topic, msg_type):
        self.topic = topic
        self.msg_type = msg_type
        self.received = False
        self._message = None

    @property
    def message(self):
        """Getter for the message property.

        Makes sure you've actually recieved a message before providing one.
        """
        if not self.received:
            raise NoMessage(('No message has been '
                'published to {} yet').format(self.topic))
        return self._message

    def wait_for_message(self, timeout=10):
        """Awaits a message on the node topic.

        Suspends until the result is recieved.
        """
        elapsed = 0
        while not self.received and elapsed < timeout:
            time.sleep(.1)
            elapsed += .1
        if elapsed >= timeout:
            raise TimeoutError(('Timed out waiting '
                'for message on {}').format(self.topic))
        yield

@contextlib.contextmanager
def check_topic(topic, rosmsg_type, callback=None):
    """Context manager that monitors a rostopic and gets a message sent to it.
    """
    if callback is None:
        def default_callback(message):
            """A no-op callback for default use.
            """
            return message
        callback = default_callback

    rospy.init_node('test_'+topic.split('/')[-1], anonymous=True)
    test_node = TestNode(topic, rosmsg_type)
    new_callback = functools.partial(callback, test_node)

    def cb_wrapper(message):
        """Wrapper around the user-provided callback.

        Sets a flag to be used by other methods.
        """
        test_node.received = True
        test_node._message = message  # pylint: disable=protected-access

        return new_callback(message)

    rospy.Subscriber(topic, rosmsg_type, cb_wrapper)
    yield test_node
    rospy.signal_shutdown('test complete')

    # Ros really doesn't want you to reinitialize a node once it's been
    # shutdown because there can be bad side effects, but we are good
    # at cleaning up after ourselves.
    rospy.client._init_node_args = None  # pylint: disable=protected-access
    rospy.core._shutdown_flag = False  # pylint: disable=protected-access
    rospy.core._in_shutdown = False  # pylint: disable=protected-access

