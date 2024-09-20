#!/bin/bash

source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash

export LINOROBOT2_BASE=mecanum

ros2 launch linorobot2_navigation slam.launch.py rviz:=true sim:=true

bash