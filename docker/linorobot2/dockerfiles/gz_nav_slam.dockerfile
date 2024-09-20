FROM osrf/ros:humble-desktop

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    git \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    python3-vcstool \
    python3-pip \
    ros-humble-nav2-bringup \
    ros-humble-joint-state-publisher \
    ros-humble-xacro \
    ros-humble-robot-localization \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-image-proc \
    ros-humble-depth-image-proc \
    python3-networkx \
    ros-humble-imu-tools \
    ros-humble-joy-linux \
    ros-humble-laser-filters \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install -U trimesh

COPY linorobot2 /home/humble_ws/src/linorobot2
COPY linorobot2_base /home/humble_ws/src/linorobot2_base
COPY linorobot2_bringup /home/humble_ws/src/linorobot2_bringup
COPY linorobot2_description /home/humble_ws/src/linorobot2_description
COPY linorobot2_gazebo /home/humble_ws/src/linorobot2_gazebo
COPY linorobot2_navigation /home/humble_ws/src/linorobot2_navigation

ENV NVIDIA_VISIBLE_DEVICES ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

RUN echo "export LINOROBOT2_BASE=mecanum" >> ~/.bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc