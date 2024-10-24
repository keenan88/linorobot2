FROM osrf/ros:humble-desktop

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && \
    apt-get install -y \
    python3-pip \
    curl \
    python3-colcon-mixin \
    ros-dev-tools \
    ros-humble-rmf-dev \
    ros-humble-gazebo-plugins \
    python3-paho-mqtt \
    python3-flask-socketio \
    python3-flask \
    python3-flask-cors \
    python3-websockets \
    ros-humble-ros-ign-bridge

RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
RUN wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -

RUN python3 -m pip install flask-socketio fastapi uvicorn setuptools==58.2.0

WORKDIR /home/humble_ws/src

RUN git clone https://github.com/open-rmf/rmf_demos.git -b 2.0.3

WORKDIR /home/humble_ws

RUN source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install && \
    source install/setup.bash

RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
RUN echo "source /home/humble_ws/install/setup.bash" >> ~/.bashrc
