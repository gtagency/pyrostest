"""Utilities that deal with creating fake rosnodes for testing.

Contains tools to send and recieve data from fake nodes and topics. This is
needed because ROS doesn't allow you to run multiple nodes from the same
process, so instead we have to set up a background process and communicate with
it over the system.
"""

import contextlib
import cPickle as pickle
import os
import pkg_resources
import subprocess
import time
import threading
from StringIO import StringIO
import unittest

import pyrostest.rostest_utils

FileNotFoundError = IOError


class TimeoutError(Exception):
    """Py3 shim, represents no response from a syscall.
    """
    pass


class NoMessage(Exception):
    """Exception for a lack of message from a Node.
    """
    pass


class MockPublisher(object):
    """Mock of a node object for testing.
    """
    def __init__(self, topic, msg_type, queue_size):
        self.topic = topic
        self.msg_type = msg_type
        pub_data = pickle.dumps((topic, msg_type, queue_size))
        # dynamically looks up the location of the publisher.py file in
        # relation to this file (they should be in the same dir)
        location = pkg_resources.resource_filename(__name__, "publisher.py")
        if not os.path.isfile(location):
            raise FileNotFoundError('{} cannot be located'.format(location))

        self.proc = subprocess.Popen(['python', location, pub_data],
                stdin=subprocess.PIPE)

    def send(self, value):
        """Sends data to be publoshed to the mocked topic.
        """
        time.sleep(.5)
        value.serialize(self.proc.stdin)

    def kill(self):
        """Kills the publisher ros node process.
        """
        self.proc.kill()
        self.proc.wait()


class MockSubscriber(object):
    """Wrapper around a node used for testing.
    """

    def __init__(self, topic, msg_type, timeout):
        self.timeout = timeout
        self.topic = topic
        self.msg_type = msg_type
        self.killed = False

        location = pkg_resources.resource_filename(__name__, "subscriber.py")
        if not os.path.isfile(location):
            raise FileNotFoundError('{} cannot be located'.format(location))
        self.proc = subprocess.Popen(['python', location,
            pickle.dumps((topic, msg_type))], stdout=subprocess.PIPE)
        self._message = None

    def kill(self):
        """Kills the subscriber ros node process.
        """
        self.proc.kill()
        self.proc.wait()

    @property
    def message(self):
        """Getter for the message property.

        Makes sure you've actually recieved a message before providing one.
        """
        if not self._message:
            msg = self.msg_type()
            sio = StringIO()
            msg.serialize(sio)
            # This weird business with StringIO is to make sure we read the
            # correct number of bytes. Essentially, a serialized ros message
            # is a fixed length string sent over the channel, so we serialize
            # the default message of a type and then read that number of bytes
            # so that we don't encounter any issues with reading too much or
            # too little data.

            def kill_proc():
                """Kills the proccess and sets a flag to raise an error.
                """
                self.proc.kill()
                self.killed = True

            timer = threading.Timer(self.timeout, kill_proc)
            try:
                timer.start()
                data = self.proc.stdout.read(sio.len)
            finally:
                timer.cancel()

            if self.killed:
                raise TimeoutError('No message published to {} within {} '
                        'seconds. Needed {} bytes, got {}.'.format(self.topic,
                            self.timeout, sio.len, len(data)))
            msg.deserialize(data)
            self._message = msg
        return self._message


def _await_node(topic, prefix, rosmaster_uri):
    """Helper to block until a node is availible in ROS.
    """
    no_ns = topic.split('/')[-1]
    while not any(nn.split('/')[-1].startswith(''.join([prefix, '_', no_ns]))
            for nn in
            pyrostest.rostest_utils.my_get_node_names(uri=rosmaster_uri)):
        time.sleep(.1)


class RosTest(unittest.TestCase):
    """A subclass of TestCase that exposes some additional ros-related attrs.

    self.port is the port this instance will run on.
    self.rosmaster_uri is equivalent to the ROS_MASTER_URI environmental var
    """
    __metaclass__ = pyrostest.rostest_utils.RosTestMeta

    def __init__(self, *args, **kwargs):
        super(RosTest, self).__init__(*args, **kwargs)
        self.LAUNCHER = dict()

    @contextlib.contextmanager
    def check_topic(self, topic, rosmsg_type, timeout=10):
        """Context manager that monitors a rostopic and gets a message sent to
        it.
        """
        test_node = MockSubscriber(topic, rosmsg_type, timeout=timeout)
        _await_node(topic, 'mock_subscribe', self.rosmaster_uri)

        try:
            yield test_node
        finally:
            test_node.kill()

    @contextlib.contextmanager
    def mock_pub(self, topic, rosmsg_type, queue_size=1):
        """Mocks a node and cleans it up when done.
        """
        pub = MockPublisher(topic, rosmsg_type, queue_size)
        _await_node(topic, 'mock_publish', self.rosmaster_uri)
        try:
            yield pub
        finally:
            pub.kill()

