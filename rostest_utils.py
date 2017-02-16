"""A collection of utilities to make testing with ros less painful.
"""

import os
import random
import subprocess
import unittest
import socket
import time
import psutil


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
            p = psutil.Process(self.roscore.pid)
            children = p.children(recursive=True)

            old_teardown(self)
            self.roscore.kill()
            self.roscore.wait()
            self.roscore = None
            for child in children:
                child.terminate()
                child.wait()


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
