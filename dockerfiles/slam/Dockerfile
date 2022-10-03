FROM ubuntu:20.04

ENV TZ=Etc/UTC
ARG DEBIAN_FRONTEND=noninteractive
ARG ROOT_PWD=root

WORKDIR /app

USER root
# Set root password
RUN echo 'root:${ROOT_PWD}' | chpasswd

RUN apt update \
    && apt install -y --no-install-recommends \
    nano g++ cmake make ninja-build git gcc \
    # OpenCV dependencies
    libavcodec-dev libavformat-dev libswscale-dev libavcodec-dev libavformat-dev \
    libswscale-dev libgtk-3-dev libpng-dev libjpeg-dev libopenexr-dev libtiff-dev \
    libwebp-dev

# Install OpenCV

# Clean up
# RUN apt-get clean \
#     && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

CMD bash 
