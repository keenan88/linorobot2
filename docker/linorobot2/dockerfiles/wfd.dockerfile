FROM osrf/ros:humble-desktop

RUN apt-get update && apt-get install -y \
    gnome-terminal \
    dbus-x11 \
    ros-humble-cartographer-ros \
    ros-humble-navigation2 \    
    ros-humble-nav2-bringup \
    ros-humble-turtlebot3-msgs \
    ros-humble-dynamixel-sdk \
    ros-humble-hls-lfcd-lds-driver \
    ros-humble-turtlebot3 \
    ros-humble-turtlebot3-gazebo \
    ros-humble-turtlebot3-simulations \
    ros-humble-gazebo-ros-pkgs \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

RUN pip install setuptools==58.2.0

# Set up environment
SHELL ["/bin/bash", "-c"]
WORKDIR /root

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc