#!/bin/bash

set -e

cd ~

# Escape virtualenv and upgrade pip
deactivate
python -m pip install --upgrade pip

# install ros
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net --recv-key 0xB01FA116
sudo apt-get update -qq
sudo apt-get install -y python-catkin-pkg python-rosdep ros-indigo-catkin ros-indigo-ros ros-indigo-roslaunch build-essential

# Install our project
pip install -e ~/$CIRCLE_PROJECT_REPONAME

# And copy it into the catkin_ws directory
cp -r ~/pyrostest/test/pyrostest ~/catkin_ws/src

# And install the testing tool
python -m pip install pytest

# Install and update rosdep
sudo rosdep init
rosdep update

# Build the project and install rosdeps
cd ~/catkin_ws/src
source /opt/ros/indigo/setup.bash
catkin_init_workspace
rosdep install -y --from-paths ./pyrostest --ignore-src --rosdistro=indigo
cd ~/catkin_ws
catkin_make
source devel/setup.bash
python -m pytest src/pyrostest
