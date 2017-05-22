import pytest

import pyrostest

class TestSpinUp(pyrostest.RosTest):
    def noop(self):
        pass

class FailureCases(pyrostest.RosTest):
    @pytest.mark.xfail(strict=True)
    @pyrostest.launch_node('this_isnt_a_project', 'add_one.py')
    def no_rospackage(self):
        pass


    @pytest.mark.xfail(strict=True)
    @pyrostest.launch_node('pyrostest', 'this_isnt_a_rosnode.py')
    def no_node(self):
        pass

    
    @pytest.mark.xfail(strict=True)
    @pyrostest.with_launch_file('pyrostest', 'does_not_exist')
    def no_launch_file(self):
        pass

    @pytest.mark.xfail(strict=True)
    @pyrostest.with_launch_file('not_a_package', 'exists')
    def no_launch_package(self):
        pass
