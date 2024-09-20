#!/bin/bash

source /opt/ros/humble/setup.bash

colcon build --packages-select linorobot2_joystick linorobot2_bringup
source install/setup.bash

ros2 launch linorobot2_bringup joystick.launch.py

bash