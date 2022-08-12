FROM hihi313/ros-kalibr

ENV TZ=Etc/UTC
ARG DEBIAN_FRONTEND=noninteractive
ARG ROOT_PWD=root

WORKDIR /catkin_ws

USER root
# Set root password
RUN echo 'root:${ROOT_PWD}' | chpasswd

RUN apt update \
    && apt install -y --no-install-recommends \
    git nano \
    build-essential cmake libgoogle-glog-dev libgflags-dev libatlas-base-dev libeigen3-dev libsuitesparse-dev \
    libdw-dev \
    # Patch for kalibr official Dockerfile
    python3-pyx

# Install Ceres
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

CMD bash 
