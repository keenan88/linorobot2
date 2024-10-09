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

RUN python3 -m pip install flask-socketio fastapi uvicorn setuptools==58.2.0

RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
