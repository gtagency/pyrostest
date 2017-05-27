import pytest

import pyrostest

class TestSpinUp(pyrostest.RosTest):
    def test_noop(self):
        pass

    @pyrostest.launch_node('pyrostest', 'add_one.py')
    def test_launches_node(self):
        pass

    @pyrostest.with_launch_file('pyrostest', 'launch.launch')
    def test_just_launch(self):
        pass


    @pyrostest.with_launch_file('pyrostest', 'launch.launch')
    @pyrostest.launch_node('pyrostest', 'add_one.py')
    def test_launch_and_unused_node(self):
        pass

    @pyrostest.with_launch_file('pyrostest', 'launch.launch')
    @pyrostest.launch_node('pyrostest', 'pub_foo.py')
    def test_launch_and_used_node(self):
        pass


class TestFailureCases(pyrostest.RosTest):
    @pytest.mark.xfail(strict=True)
    @pyrostest.launch_node('this_isnt_a_project', 'add_one.py')
    def test_no_rospackage(self):
        pass


    @pytest.mark.xfail(strict=True)
    @pyrostest.launch_node('pyrostest', 'this_isnt_a_rosnode.py')
    def test_no_node(self):
        pass

    
    @pytest.mark.xfail(strict=True)
    @pyrostest.with_launch_file('pyrostest', 'does_not_exist')
    def test_no_launch_file(self):
        pass

    @pytest.mark.xfail(strict=True)
    @pyrostest.with_launch_file('not_a_package', 'exists')
    def test_no_launch_package(self):
        pass

