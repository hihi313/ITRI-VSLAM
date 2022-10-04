FROM nvidia/cuda:11.7.1-cudnn8-runtime-ubuntu20.04

ENV TZ=Etc/UTC
ARG DEBIAN_FRONTEND=noninteractive
ARG ROOT_PWD=root

WORKDIR /app

USER root
# Set root password
RUN echo 'root:${ROOT_PWD}' | chpasswd

RUN apt update \
    && apt install -y --no-install-recommends \
    nano g++ cmake make ninja-build git gcc ca-certificates\
    # OpenCV dependencies
    libavcodec-dev libavformat-dev libswscale-dev libgstreamer-plugins-base1.0-dev \
    libgstreamer1.0-dev libgtk-3-dev libpng-dev libjpeg-dev libopenexr-dev libtiff-dev \
    libwebp-dev \
    # Eigen
    libeigen3-dev \
    # g2o dependencies
    libsuitesparse-dev qtdeclarative5-dev qt5-qmake libqglviewer-dev-qt5


WORKDIR /tmp

# Install OpenCV
RUN git clone https://github.com/opencv/opencv.git \
    && cd opencv \
    && mkdir build \
    && cd build \
    && cmake -D CMAKE_BUILD_TYPE=RELEASE \
    -D BUILD_TIFF=ON \
    -D WITH_CUDA=ON \
    -D ENABLE_AVX=OFF \
    -D WITH_OPENGL=OFF \
    -D WITH_OPENCL=OFF \
    -D WITH_IPP=OFF \
    -D WITH_TBB=ON \
    -D BUILD_TBB=ON \
    -D WITH_EIGEN=ON \
    -D WITH_V4L=OFF \
    -D WITH_VTK=OFF \
    -D BUILD_TESTS=OFF \
    -D BUILD_PERF_TESTS=OFF \
    -D OPENCV_GENERATE_PKGCONFIG=ON \
    -GNinja \
    ../ \
    && ninja \
    && ninja install

# Install g2o
RUN git clone https://github.com/RainerKuemmerle/g2o.git \
    && cd g2o \
    && mkdir build \
    && cd build \
    && cmake -D CMAKE_BUILD_TYPE=Release -GNinja ../ \
    && ninja \
    && ninja install

# Install Sophus
RUN git clone https://github.com/strasdat/Sophus.git\
    && cd Sophus \
    && mkdir build \
    && cd build \
    && cmake -D CMAKE_BUILD_TYPE=Release -GNinja ../ \
    && ninja \
    && ninja install


WORKDIR /app

# Clean up
RUN apt-get clean \
    && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

CMD bash 
