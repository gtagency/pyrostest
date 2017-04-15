#!/bin/bash

set -e

source /opt/ros/indigo/setup.sh
cd ../catkin_ws
catkin_make
source devel/setup.sh
pytest src/pyrostester/test
