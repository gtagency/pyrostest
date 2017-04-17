"""Utilities for launching ros nodes for tests.
"""
import functools
import os
import sys
import time

import rosgraph
import roslaunch
import rosnode


class RosLaunchException(Exception):
    """An Exception thrown when the with_launch_file decorator is misused.
    """
    pass


def my_get_node_names(namespace=None, uri='http://localhost:11311'):
    """Monkeypatches get_node_names with a non-default ROS_MASTER_URI.
    """
    old_master = rosgraph.Master
    rosgraph.Master = functools.partial(rosgraph.Master, master_uri=uri)
    nodenames = rosnode.get_node_names(namespace=namespace)
    rosgraph.Master = old_master
    return nodenames


class ROSLauncher(roslaunch.scriptapi.ROSLaunch):
    """
    ROSLaunch but allows the use of launch files.

    This was found by peering into the roslaunch util source code from
    `which roslaunch`.
    """
    def __init__(self, files, port=11311):
        super(ROSLauncher, self).__init__()
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, True)
        self.parent = roslaunch.parent.ROSLaunchParent(uuid,
                files, is_core=False, port=port)


_LAUNCHER = dict()

# The following two methods are intended for use together. You need to be
# aware of the following limitations:
# `with_launch_file` should always be called first. That is:
#
# ```
# @with_launch_file(*args)
# @launch_node(*node_args)
# def test_function(self):
#     ...
# ```

# is valid, as is
#
# ```
# @launch_node(*node_args)
# @launch_node(*other_node_args)
# def test_function(self):
#     ...
# ```
# however, neither of these would be valid:
#
# ```
# @launch_node(*node_args)
# @with_launch_file(*args)
# def test_function(self):
#     ...
# ```
#
# ```
# @with_launch_file(*args)
# @launch_node(*node_args)
# @with_launch_file(*more_launch_args)
# def test_function(self):
#     ...
# ```
#
# this is handled by having `with_launch_file` check for a running launcher for
# its rosmaster port, and if one exists already, raising an error. Otherwise it
# just works.
# Similarly, launch_node will use a launcher if one exists, otherwise the
# outermost launch_node will create a master launcher and clean up after itself
# and all child nodes launched for the test.

def with_launch_file(package, launch, **kwargs):
    """Decorator to source a launch file for running nodes.

    This should always be run first.

    This and launch nodes work together gracefully, as long as you follow the
    guidelines outlined in `buzzmobile/tests/test_utils/launch_tools.py`.
    """
    full_name = roslaunch.rlutil.resolve_launch_arguments([package, launch])
    def launcher(func):
        """Decorator function created by the decorator-gen.
        """
        @functools.wraps(func)
        def new_test(self):
            """Wrapper around the user provided test that runs a launch file.
            """
            # set env variables and add argvs to sys.argv
            os.environ['ROS_MASTER_URI'] = self.rosmaster_uri
            new_argvs = ['{}:={}'.format(k, v) for k, v in kwargs.iteritems()]
            sys.argv.extend(new_argvs)

            launch = ROSLauncher(full_name, port=self.port)
            launch.start()
            if self.port in _LAUNCHER:
                raise RosLaunchException('Rosmaster port {} already in use. '
                'You must call use @with_launch_file only once for any single '
                'test, and before any @launch_node calls.'.format(self.port))

            _LAUNCHER[self.port] = launch

            try:
                temp = func(self)
            except Exception as exc:
                raise exc
            finally:
                _LAUNCHER[self.port].stop()
                # clean argvs from sys.argv
                sys.argv = sys.argv[:len(new_argvs)]
                del _LAUNCHER[self.port]
            return temp
        return new_test
    return launcher

def launch_node(package, name, namespace=None):
    """Decorator to manage running a node and shutting it down gracefully.

    Note that this will wrap itself up cleanly and launch all nodes with a
    single launcher, instead of multiples.
    """
    if not namespace:
        namespace = '/'+package
    def launcher(func):
        """Actual decorator generated by the above.
        """
        @functools.wraps(func)
        def new_test(self):
            """Wrapper around the user-provided test that runs a ros node.
            """
            env = {'ROS_MASTER_URI': self.rosmaster_uri}
            node = roslaunch.core.Node(package, name, namespace=namespace,
                    env_args=env.iteritems())
            is_master = False
            if self.port not in _LAUNCHER:
                # set env variables and add argvs to sys.argv
                os.environ['ROS_MASTER_URI'] = self.rosmaster_uri
                launch = ROSLauncher([], port=self.port)
                launch.start()
                _LAUNCHER[self.port] = launch
                is_master = True
            else:
                launch = _LAUNCHER[self.port]

            process = launch.launch(node)
            # Beware this is a bit of a hack, and will currently not work if we
            # want to run more than 1 node with the same name.
            while not any(nn.split('/')[-1].startswith(name.replace('.', '_'))
                    for nn in my_get_node_names(uri=self.rosmaster_uri)):
                time.sleep(.1)
            try:
                temp = func(self)
            except:
                raise
            finally:
                process.stop()
            if is_master:
                _LAUNCHER[self.port].stop()
                del _LAUNCHER[self.port]
            return temp

        return new_test
    return launcher
