FROM hihi313/ros-kalibr-ceres

ENV TZ=Etc/UTC
ARG DEBIAN_FRONTEND=noninteractive
ARG DEPTHAI_URL=https://docs.luxonis.com
ARG DEPTHAI_SETUP_SCIRPT=install_dependencies.sh

WORKDIR /catkin_ws

USER root

# Install dependencies
ADD $DEPTHAI_URL/$DEPTHAI_SETUP_SCIRPT ./
RUN apt update \
    && $DEPTHAI_SETUP_SCIRPT | bash
# Install DepthAI python SDK
RUN python3 -m pip install depthai

# Clean up
RUN apt-get clean \
    && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*
    
CMD bash 