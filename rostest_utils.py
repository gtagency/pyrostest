"""A collection of utilities to make testing with ros less painful.
"""

import subprocess

def with_roscore(obj):
    """Decorator to run all tests in a testcase with their own roscore.

    This wraps the setUp and tearDown methods to start by first spinning up a
    roscore process, and tears it down at the very end. This adds a small time
    penalty, but its worth it.

    Its worth deciding whether to make this work on a per-method, per object,
    or both basis.
    """
    old_setup = obj.setUp
    old_teardown = obj.tearDown

    def new_setup(self):
        """Wrapper around the user-defined setUp method that runs roscore.
        """
        self.roscore = subprocess.Popen(['roscore'])
        old_setup(self)

    def new_teardown(self):
        """Wrapper around the user-defined tearDown method that ends roscore.
        """
        old_teardown(self)
        self.roscore.kill()
        subprocess.call(['killall', '-9', 'rosmaster'])
        self.roscore.wait()

    obj.setUp = new_setup
    obj.tearDown = new_teardown
    return obj

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
