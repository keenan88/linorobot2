FROM osrf/ros:humble-desktop

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && \
    apt-get install -y \
    nano \
    ros-humble-realsense2-camera \ 
    ros-humble-realsense2-description \
    usbutils \
    python3-pip -y \
    ros-humble-pointcloud-to-laserscan \
    ros-humble-depthimage-to-laserscan \
    ros-humble-joint-state-publisher \
    ros-humble-controller-manager \
    ros-humble-joint-state-publisher-gui \
    && rm -rf /var/lib/apt/lists/*

RUN pip install setuptools==58.2.0

RUN mkdir -p /home/humble_ws/src
WORKDIR /home/humble_ws

# Build in-image to avoid having to re-build from scratch in entrypoint script
# COPY ./antworker_bringup /home/humble_ws/src/antworker_bringup
# COPY ./antworker_realsense /home/humble_ws/src/antworker_realsense
# RUN source /opt/ros/humble/setup.bash && colcon build --symlink-install

# COPY ./docker/antworker/entry_scripts/realsense_entrypoint.sh /realsense_entrypoint.sh
# RUN chmod +x /realsense_entrypoint.sh

RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
RUN echo "source /home/humble_ws/install/setup.bash" >> ~/.bashrc
