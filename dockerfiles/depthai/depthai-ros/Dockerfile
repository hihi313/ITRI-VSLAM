FROM osrf/ros:noetic-desktop-full

ENV TZ=Etc/UTC
ARG DEBIAN_FRONTEND=noninteractive

WORKDIR /app

USER root
# Install
# install depthai dependencies
RUN apt update \
    && apt install  -y --no-install-recommends \
    python3 python3-pip udev cmake git python3-numpy libusb-1.0-0-dev build-essential libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev python-dev libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libdc1394-22-dev ffmpeg libsm6 libxext6 libgl1-mesa-glx \
    libopencv-dev
RUN python3 -m pip install --upgrade pip
# Allow all users to read and write to Myriad X devices
RUN echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | tee /etc/udev/rules.d/80-movidius.rules
# Install depthai core
# cd /tmp
WORKDIR /tmp
RUN git clone --recursive https://github.com/luxonis/depthai-core.git --branch main
RUN cmake -j$(nproc) -Hdepthai-core -Bdepthai-core/build -DBUILD_SHARED_LIBS=ON -DCMAKE_INSTALL_PREFIX=/usr/local \
    && cmake --parallel $(nproc) --build depthai-core/build --target install
RUN rm -r depthai-core 
# Install depthai ros
WORKDIR /app/dai_ws/src
RUN git clone https://github.com/luxonis/depthai-ros.git
WORKDIR /app
RUN rosdep install --from-paths src --ignore-src -r -y
RUN source /opt/ros/$ROS_DISTRO/setup.bash \
    && catkin_make -j$(nproc)

# Clean up
RUN apt-get clean \
    && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*
RUN rm -r ~/.hunter

# Setup environment variable
COPY ./entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]
    
CMD bash 
