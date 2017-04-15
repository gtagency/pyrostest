#!/bin/bash

set -e

source /opt/ros/indigo/setup.sh
cd ../catkin_ws/src
catkin_init_workspace
sudo rosdep init
rosdep update
rosdep install -y --from-paths ./pyrostester --ignore-src --rosdistro=indigo
cd ..
catkin_make
source devel/setup.sh
pytest src/pyrostester/test
