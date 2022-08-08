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

CMD bash 
