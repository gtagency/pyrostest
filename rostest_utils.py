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
