FROM hihi313/ros-kalibr-ceres

ENV TZ=Etc/UTC
ARG DEBIAN_FRONTEND=noninteractive
ARG DEPTHAI_SETUP_SCRIPT=https://raw.githubusercontent.com/luxonis/depthai-ros/main/install_dependencies.sh
ARG DEPTHAI_REPO=https://github.com/luxonis/depthai-ros.git

WORKDIR /catkin_ws

USER root

# Install dependencies
# Kalibr's dockerfile has install wget
RUN sudo wget -qO- ${DEPTHAI_SETUP_SCRIPT} | sudo bash
# Setup depthai-ros
RUN mkdir -p src \
    && cd src \
    # Kalibr's dockerfile has installed git
    && git clone ${DEPTHAI_REPO} \
    && cd .. \
    && rosdep install --from-paths src --ignore-src -r -y \
    # Kalibr's dockerfile has set ROS's environment variable
    && catkin_make -j$(nproc) \
    && source devel/setup.bash

# Clean up
RUN apt-get clean \
    && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*
    
CMD bash 