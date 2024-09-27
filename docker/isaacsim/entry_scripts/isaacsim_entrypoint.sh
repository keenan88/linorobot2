#!/bin/bash

source /opt/ros/humble/setup.bash
colcon build --packages-select linorobot2_description
source install/setup.bash

. /isaac-sim/runheadless.native.sh --reset-data


exec bash