FROM hihi313/ros-kalibr-ceres

ENV TZ=Etc/UTC
ARG DEBIAN_FRONTEND=noninteractive
ARG DEPTHAI_SETUP_SCRIPT=https://raw.githubusercontent.com/luxonis/depthai-ros/main/install_dependencies.sh
ARG DEPTHAI_REPO=https://raw.githubusercontent.com/luxonis/depthai-ros/main/underlay.repos

WORKDIR /catkin_ws

USER root

# Install dependencies
# Kalibr's dockerfile has install wget
RUN wget -qO- ${DEPTHAI_SETUP_SCRIPT} | sudo bash
# Setup depthai-ros
RUN mkdir src \
    && wget ${DEPTHAI_REPO} \
    && vcs import src < underlay.repos \
    && rosdep install --from-paths src --ignore-src -r -y \
    # Kalibr's dockerfile has set ROS's environment variable
    && catkin_make -j$(nproc) \
    && source devel/setup.bash

# Clean up
RUN apt-get clean \
    && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*
    
CMD bash 