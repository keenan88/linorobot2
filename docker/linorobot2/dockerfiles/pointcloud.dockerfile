FROM ros:humble-ros-base

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive

# Do installs before git clone to avoid needing to do a full image rebuild each time image is built
RUN apt-get update && apt-get install --no-install-recommends -y \
    nano \
    ros-humble-image-pipeline \
    ros-humble-domain-bridge \
    python3-pip \
    libpcl-dev \
    libpcl-common1.12 \
    libpcl-io1.12 \
    libpcl-features1.12 \
    libpcl-filters1.12 \
    libpcl-segmentation1.12 \
    libpcl-surface1.12 \
    ros-humble-pcl-msgs \
    nano \  
    && rm -rf /var/lib/apt/lists/*

RUN pip install setuptools==58.2.0

WORKDIR /home/humble_ws

COPY ./perception_pcl /home/humble_ws/src/perception_pcl
RUN source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install --packages-select pcl_conversions pcl_ros perception_pcl && \
    source install/setup.bash

RUN echo "source /home/humble_ws/install/setup.bash" >> ~/.bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc