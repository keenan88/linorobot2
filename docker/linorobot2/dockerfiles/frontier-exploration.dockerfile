FROM osrf/ros:humble-desktop

RUN apt-get update && apt-get install -y \
    gnome-terminal \
    dbus-x11 \
    ros-humble-nav2-costmap-2d \
    ros-humble-nav2-msgs \
    && rm -rf /var/lib/apt/lists/*

# Set up environment
SHELL ["/bin/bash", "-c"]
WORKDIR /root

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
