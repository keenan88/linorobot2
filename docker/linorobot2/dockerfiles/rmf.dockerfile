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
    ros-humble-nav2-bringup \
    ros-humble-ros-ign-bridge

RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
RUN wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -

RUN python3 -m pip install flask-socketio fastapi uvicorn 

RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
RUN echo "source /home/humble_ws/install/setup.bash" >> ~/.bashrc
