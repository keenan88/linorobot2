#!/bin/bash

source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash

export LINOROBOT2_BASE=mecanum

if [ "$SLAM_OR_NAV" = "NAV" ]; then
    echo "Running navigation..."
    ros2 launch linorobot2_navigation navigation.launch.py rviz:=true
elif [ "$SLAM_OR_NAV" = "SLAM" ]; then
    echo "Running SLAM..."
    ros2 launch linorobot2_navigation slam.launch.py rviz:=true sim:=true
else
    echo "Unknown SLAM_OR_NAV mode: $SLAM_OR_NAV. Please set SLAM_OR_NAV to 'NAV' or 'SLAM'."
fi



bash