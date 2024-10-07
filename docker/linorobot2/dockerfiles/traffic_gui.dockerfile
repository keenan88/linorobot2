FROM osrf/ros:humble-desktop

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && \
    apt-get install -y \
    python3-shapely \
    python3-yaml \
    python3-requests \
    ros-humble-rmf-building-map-msgs \
    libignition-fuel-tools7-dev \
    python3-fiona \
    python3-pyproj \
    ros-humble-rmf-site-map-msgs \
    sqlite3 \
    ros-humble-rmf-utils \
    libceres-dev \
    libceres-dev \
    python3-pip \
    python3-rtree

RUN pip install setuptools==58.2.0

RUN mkdir -p /home/humble_ws/src
WORKDIR /home/humble_ws

# Build in-image to avoid having to re-build from scratch in entrypoint script
COPY ./rmf_traffic_editor /home/humble_ws/src/rmf_traffic_editor
# RUN source /opt/ros/humble/setup.bash && colcon build --symlink-install

RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
RUN echo "source /home/humble_ws/install/setup.bash" >> ~/.bashrc
