#!/bin/bash

set -e

source /opt/ros/indigo/setup.sh
cd ../catkin_ws
catkin_init_workspace
sudo rosdep init
rosdep update
rosdep install -y --from-paths ./pyrostest --ignore-src --rosdistro=indigo
catkin_make
source devel/setup.sh
pytest src/pyrostester/test
