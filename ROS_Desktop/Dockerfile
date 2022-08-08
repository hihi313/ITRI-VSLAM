FROM ubuntu:20.04

# Defualt user of anibali/pytorch
ARG ROOT_PWD=root

USER root
# Set root password
RUN echo 'root:${ROOT_PWD}' | chpasswd
# Replace to latest ORB_SLAM3 code by mount host volume
RUN rm -rf /dpds/ORB_SLAM3

# Install 
# glxgears: to test X display
RUN apt-get install mesa-utils gdb

CMD bash 
