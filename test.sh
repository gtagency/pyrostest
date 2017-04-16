#!/bin/bash

set -e

source /opt/ros/indigo/setup.sh
cd ../catkin_ws/src
pip install catkin-pkg
catkin_init_workspace
sudo rosdep init
rosdep update
rosdep install -y --from-paths ./pyrostest --ignore-src --rosdistro=indigo
cd ..
catkin_make
source devel/setup.sh
pytest src/pyrostester/test
