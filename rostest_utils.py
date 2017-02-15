"""A collection of utilities to make testing with ros less painful.
"""

import os
import signal
import random
import subprocess
import unittest
import socket
import time
import logging
import rosgraph
import rosnode
import psutil

try: 
    from xmlrpc.client import ServerProxy 
except ImportError: 
    from xmlrpclib import ServerProxy

def rand_port():
    """Picks a random port number.

    This is potentially unsafe, but shouldn't generally be a problem.
    """
    return random.randint(10311, 12311)


class RosTestMeta(type):
    """Metaclass for RosTest that adds the setup/teardown we want.
    """
    def __new__(mcs, name, bases, dct):

        # It will break unless we throw in fake setup and teardown methods if
        # the real ones don't exist yet.

        def noop(_):
            """Do nothing function.

            This is injected if there is no user-defined setUp or tearDown
            method on an instance of RosTest.
            """
            pass

        try:
            old_setup = dct['setUp']
        except KeyError:
            old_setup = noop
        try:
            old_teardown = dct['tearDown']
        except KeyError:
            old_teardown = noop

        def new_setup(self):
            """Wrapper around the user-defined setUp method that runs roscore.
            """
            self.port = rand_port()
            self.rosmaster_uri = 'http://{}:{}'.format(socket.gethostname(),
                    self.port)
            env = {k:v for k, v in os.environ.iteritems()}
            env.update({'ROS_MASTER_URI': self.rosmaster_uri})
            roscore_initialized = False
            logging.basicConfig()
            log = logging.getLogger(__name__)
            log.warning(name)
            while not roscore_initialized:
                self.roscore = subprocess.Popen(
                        ['roscore', '-p', str(self.port)], env=env)
                # an error will return a nonzero errorcode, and None indicates
                # that the process is still running, so falsy results are good
                time.sleep(3)
                if not self.roscore.poll():
                    roscore_initialized = True
                else:
                    self.roscore.kill()
                    self.roscore = None
            old_setup(self)

        def new_teardown(self):
            """Wrapper around the user-defined tearDown method to end roscore.
            """
            master = rosgraph.masterapi.Master('roslib', 
                    master_uri=self.rosmaster_uri)
            rosmaster_pid = master.getPid()
            rosout_node = ServerProxy(rosnode.get_api_uri(master, 'rosout'))
            rosout_pid = rosout_node.getPid('roslog')[-1]

            old_teardown(self)
            self.roscore.kill()
            self.roscore.wait()
            self.roscore = None
            os.kill(rosmaster_pid, signal.SIGTERM)
            os.kill(rosout_pid, signal.SIGTERM)

        dct['setUp'] = new_setup
        dct['tearDown'] = new_teardown
        dct['setUp'].__name__ = 'setUp'
        dct['tearDown'].__name__ = 'tearDown'
        return super(RosTestMeta, mcs).__new__(mcs, name, bases, dct)


class RosTest(unittest.TestCase):
    """A subclass of TestCase that exposes some additional ros-related attrs.

    self.port is the port this instance will run on.
    self.rosmaster_uri is equivalent to the ROS_MASTER_URI environmental var
    """
    __metaclass__ = RosTestMeta


def await(gen):
    """Shim to add await syntax to python2, kinda.

    On a high level, this function simply takes the next item from a generator
    and passes it along, but it blocks until that item is gotten. When used in
    the context

        await(TestNode.wait_for_message())

    it will wait for the `wait_for_message` call to finish running.

    In other words, if await(Node) doesn't raise an error, you should have
    successfully recieved a message, and can therefore test against it,
    otherwise the message access can raise a NoMessage error.
    """
    res = None
    for item in gen:
        res = item
    return res
