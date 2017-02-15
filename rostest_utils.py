"""A collection of utilities to make testing with ros less painful.
"""

import os
import random
import subprocess
import unittest
import socket


def rand_port():
    """Picks a random port number.

    This is potentially unsafe, but shouldn't generally be a problem.
    """
    return random.randint(10311, 12311)


class RosTestMeta(type):
    def __new__(cls, name, bases, dct):

        # It will break unless we throw in fake setup and teardown methods if
        # the real ones don't exist yet.

        def noop(_):
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
            try:
                #TODO: make this retry on fail, or something similar.
                self.roscore = subprocess.Popen(
                        ['roscore', '-p', str(self.port)], env=env)
            except:
                raise Exception('port not availible')
            old_setup(self)

        def new_teardown(self):
            """Wrapper around the user-defined tearDown method that ends roscore.
            """
            old_teardown(self)
            self.roscore.kill()
            self.roscore.wait()

        dct['setUp'] = new_setup
        dct['tearDown'] = new_teardown
        dct['setUp'].__name__ = 'setUp'
        dct['tearDown'].__name__ = 'tearDown'
        return super(RosTestMeta, cls).__new__(cls, name, bases, dct)


class RosTest(unittest.TestCase):
    """A subclass of TestCase that exposes some additional ros-related attrs.

    self.port is the port this instance will run on.
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
