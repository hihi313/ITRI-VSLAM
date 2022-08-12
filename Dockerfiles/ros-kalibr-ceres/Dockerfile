FROM hihi313/ros-kalibr

ENV TZ=Etc/UTC
ARG DEBIAN_FRONTEND=noninteractive
ARG ROOT_PWD=root

WORKDIR /catkin_ws

USER root
# Set root password
RUN echo 'root:${ROOT_PWD}' | chpasswd

# Install dependencies
RUN apt update \
    && apt install -y --no-install-recommends \
    git \
    build-essential cmake libgoogle-glog-dev libgflags-dev libatlas-base-dev libeigen3-dev libsuitesparse-dev \
    # Some package we need to use
    # libdw-dev for https://github.com/gaowenliang/imu_utils.git
    libdw-dev nano\
    # Patch for kalibr official Dockerfile
    python3-pyx

# Build Ceres from source
RUN cd \
    && mkdir -p ceres-bin \
    && cd ceres-bin \
    && git clone https://github.com/ceres-solver/ceres-solver \
    && cd ceres-solver \
    && mkdir build \
    && cd build \
    && echo $PWD \
    && cmake .. \
    && make -j$(nproc)\
    && make install

# Clean up
RUN apt-get clean \
    && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

CMD bash 
