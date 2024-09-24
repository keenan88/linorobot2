# Use the ROS 2 Humble base image
FROM ros:humble

# Install teleop_twist_keyboard package
RUN apt-get update && apt-get install -y \
    ros-humble-teleop-twist-keyboard \
    gnome-terminal \
    dbus-x11 \
    && rm -rf /var/lib/apt/lists/*

# Set up environment
SHELL ["/bin/bash", "-c"]
WORKDIR /root

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
