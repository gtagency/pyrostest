#!/bin/bash

set -e

cd ~

# Enter a virtualenv we control!
python -m virtualenv .
source bin/activate
pip install --upgrade pip

# install ros and some dependencies
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://pgp.mit.edu:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt-get update -qq
sudo apt-get install -y python-catkin-pkg python-rosdep ros-indigo-catkin ros-indigo-ros ros-indigo-roslaunch build-essential ros-indigo-rosnode

# Install our project
pip install -e ~/$CIRCLE_PROJECT_REPONAME

# And copy it into the catkin_ws directory
cp -r ~/pyrostest/test/pyrostest ~/catkin_ws/src

# And install the testing tool
pip install pytest

# Install and update rosdep
sudo rosdep init
rosdep update

pip install rospkg
pip install catkin-pkg

# Build the project and install rosdeps
cd ~/catkin_ws/src
source /opt/ros/indigo/setup.bash
rosdep install -y --from-paths ./pyrostest --ignore-src --rosdistro=indigo
cd ~/catkin_ws
catkin_make
source devel/setup.bash
pytest src/pyrostest
