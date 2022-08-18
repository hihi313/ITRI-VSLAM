FROM hihi313/ros-kalibr-ceres

ENV TZ=Etc/UTC
ARG DEBIAN_FRONTEND=noninteractive

WORKDIR /catkin_ws
SHELL ["/bin/bash", "-c"]

USER root
# Add USB rules
RUN echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | tee /etc/udev/rules.d/80-movidius.rules 
# TODO: add below to entrypoint
RUN /lib/systemd/systemd-udevd --daemon \
    && udevadm control --reload-rules && udevadm trigger
# Install
RUN apt update \
    && apt install  -y --no-install-recommends \
    ros-noetic-depthai-ros

# Clean up
RUN apt-get clean \
    && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

# Setup environment variable
COPY ./ros_entrypoint.sh /
ENTRYPOINT ["/ros_entrypoint.sh"]
    
CMD bash 
