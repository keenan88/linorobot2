#!/bin/bash
set -e


source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash

ros2 launch linorobot2_bringup pointcloud.launch.py

exec "$@"