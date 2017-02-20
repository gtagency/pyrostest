"""Utilities that deal with creating fake rosnodes for testing.

Contains tools to send and recieve data from fake nodes and topics.
"""

import contextlib
import cPickle as pickle
import os
import subprocess
import time
import threading
from StringIO import StringIO

import rosnode


class TimeoutError(Exception):
    """Py3 shim, represents no response from a syscall.
    """
    pass


class NoMessage(Exception):
    """Exception for a lack of message in a Node.
    """
    pass

class MockPublisher(object):
    """Mock of a node object for testing.
    """
    def __init__(self, topic, msg_type, queue_size):
        self.topic = topic
        self.msg_type = msg_type
        pub_data = pickle.dumps((topic, msg_type, queue_size))
        this_dir = os.path.dirname(os.path.abspath(__file__))
        location = os.path.join(this_dir, 'publisher.py')
        self.proc = subprocess.Popen([location, pub_data],
                stdin=subprocess.PIPE)

    def send(self, value):
        """Sends data to be publoshed to the mocked topic.
        """
        value.serialize(self.proc.stdin)
        # Just long enough to prevent out of order
        time.sleep(.2)

    def kill(self):
        """Kills the publisher ros node process.
        """
        self.proc.kill()


@contextlib.contextmanager
def mock_pub(topic, rosmsg_type, queue_size=1):
    """Mocks a node and cleans it up when done.
    """
    pub = MockPublisher(topic, rosmsg_type, queue_size)
    no_ns = topic.split('/')[-1]
    while not any(nn.split('/')[-1].startswith(
        ''.join(['mock_publish_', no_ns])) for nn in rosnode.get_node_names()):
        time.sleep(.1)
    try:
        yield pub
    finally:
        pub.kill()


class MockListener(object):
    """Wrapper around a node used for testing.
    """

    def __init__(self, topic, msg_type, timeout=10):
        self.timeout = timeout
        self.topic = topic
        self.msg_type = msg_type
        self.killed = False

        this_dir = os.path.dirname(os.path.abspath(__file__))
        location = os.path.join(this_dir, 'listener.py')
        self.proc = subprocess.Popen([location,
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
        if  not self._message:
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
                raise TimeoutError('No message published on {} within {} '
                        'seconds'.format(self.topic, self.timeout))
            msg.deserialize(data)
            self._message = msg
        return self._message


@contextlib.contextmanager
def check_topic(topic, rosmsg_type, timeout=10):
    """Context manager that monitors a rostopic and gets a message sent to it.
    """
    test_node = MockListener(topic, rosmsg_type, timeout=timeout)
    no_ns = topic.split('/')[-1]
    while not any(nn.split('/')[-1].startswith(
        ''.join(['mock_listen_', no_ns])) for nn in rosnode.get_node_names()):
        time.sleep(.1)


    try:
        yield test_node
    finally:
        test_node.kill()
