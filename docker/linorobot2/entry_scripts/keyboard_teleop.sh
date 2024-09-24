#!/usr/bin/env bash

source /opt/ros/humble/setup.bash

gnome-terminal -e 'ros2 run teleop_twist_keyboard teleop_twist_keyboard'

bash # Prevents container from closing