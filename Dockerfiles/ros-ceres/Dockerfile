FROM osrf/ros:noetic-desktop-full

ENV TZ=Etc/UTC
ARG DEBIAN_FRONTEND=noninteractive
ARG ROOT_PWD=root

WORKDIR /app

USER root
# Set root password
RUN echo 'root:${ROOT_PWD}' | chpasswd

RUN apt update \
    && apt install -y --no-install-recommends \
    git nano \
    build-essential cmake libgoogle-glog-dev libgflags-dev libatlas-base-dev libeigen3-dev libsuitesparse-dev \
    libdw-dev

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

# Set env
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

COPY ../../code_utils /app/catkin_ws/src/
COPY ../../imu_utils /app/catkin_ws/src/

CMD bash 
