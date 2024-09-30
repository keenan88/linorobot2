#!/bin/bash
set -e


source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash

ros2 launch antdrone_depth_to_laserscan depth_to_laserscan.launch.py

exec "$@"