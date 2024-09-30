FROM nvcr.io/nvidia/isaac-sim:4.0.0 

# ROS2 Install steps taken from: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html
RUN apt update &&  apt install locales -y
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
RUN export LANG=en_US.UTF-8

RUN apt install software-properties-common -y
RUN add-apt-repository universe

RUN apt update &&  apt install curl -y
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" |  tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt update -y

RUN apt upgrade -y

# Set the timezone, ROS2 install will hang while waiting for user input if timezone is not set
ENV ROS_VERSION=2
ENV ROS_DISTRO=humble
ENV ROS_PYTHON_VERSION=3
ENV TZ=America/New_York
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

RUN apt install ros-humble-desktop -y

RUN apt install ros-dev-tools -y

# ros-humble-vision-msgs necessary for IsaacSim ROS2 bridge as per: https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_ros.html
RUN apt install ros-humble-vision-msgs

RUN apt install nano -y

ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

ENV isaac_sim_package_path=$HOME/.local/share/ov/pkg/isaac-sim-4.1.0
ENV LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$isaac_sim_package_path/exts/omni.isaac.ros2_bridge/humble/lib

ENV FASTRTPS_DEFAULT_PROFILES_FILE=/root/.ros/fastdds.xml 

WORKDIR /isaac-sim/humble_ws

# COPY ./docker/isaacsim/entry_scripts/isaacsim_entrypoint.sh /isaacsim_entrypoint.sh:rwx

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc