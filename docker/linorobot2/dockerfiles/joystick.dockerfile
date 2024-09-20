FROM osrf/ros:humble-desktop

RUN apt-get update && apt-get install -y \
    nano \
    ros-humble-teleop-twist-joy \
    ros-humble-domain-bridge \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /home/humble_ws/

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

