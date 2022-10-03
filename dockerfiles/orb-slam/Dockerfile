FROM ubuntu:20.04

ENV TZ=Etc/UTC
ARG DEBIAN_FRONTEND=noninteractive
ARG ROOT_PWD=root

WORKDIR /app

USER root
# Set root password
RUN echo 'root:${ROOT_PWD}' | chpasswd

RUN apt update \
    && apt -y upgrade \
    && apt install -y  --no-install-suggests --no-install-recommends \
    # General
    git nano cmake g++ ninja-build make build-essential \
    # Pangolin dependencies
    libgl1-mesa-dev libwayland-dev libxkbcommon-dev wayland-protocols libegl1-mesa-dev \
    libc++-dev libglew-dev \
    # OpenCV dependencies
    libavcodec-dev libavformat-dev libswscale-dev libgstreamer-plugins-base1.0-dev libgstreamer1.0-dev \
    # Eigen3
    libeigen3-dev


WORKDIR /dpds

# Install Pangolin
RUN git clone --recursive --branch v0.8 https://github.com/stevenlovegrove/Pangolin.git \
    && cd Pangolin \
    && cmake -D CMAKE_BUILD_TYPE=RELEASE \
    && cmake -B build -GNinja \
    && cmake --build build 

# Install OpenCV
# RUN cd /dpds/ \
#     && git clone https://github.com/opencv/opencv.git \
#     && git -C opencv checkout 4.x \
#     && mkdir -p build \
#     && cd build \
#     && cmake -GNinja ../opencv \
#     ninja


# Clean up
RUN apt-get clean \
    && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

CMD bash 
