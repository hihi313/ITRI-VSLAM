###############################################################################
# depthai-ros has too many error when install in docker 
# need to solve these error later
###############################################################################
FROM hihi313/ros-kalibr-ceres

ENV TZ=Etc/UTC
ARG DEBIAN_FRONTEND=noninteractive

WORKDIR /catkin_ws
SHELL ["/bin/bash", "-c"]

USER root
# Add USB rules
RUN echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | tee /etc/udev/rules.d/80-movidius.rules 
RUN udevadm control --reload-rules && udevadm trigger
# Install
RUN apt update \
    && apt install ros-noetic-depthai-ros

# Clean up
RUN apt-get clean \
    && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

# Set environment
ENTRYPOINT echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc \
    && source ~/.bashrc
    
CMD bash 