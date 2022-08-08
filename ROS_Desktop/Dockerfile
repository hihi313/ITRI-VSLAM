FROM lmwafer/orb-slam-3-ready:1.1-ubuntu18.04

# Defualt user of anibali/pytorch
ARG ROOT_PWD=root

USER root
# Share apt-get's list 
RUN mkdir -p /var/lib/apt/lists/ \
    && chmod -R 777 /var/lib/apt/lists/
# Prevent reinstall vs code extension after each image build
RUN mkdir -p /root/.vscode-server/extensions \
    && chmod -R 777 /root/.vscode-server
# Set root password
RUN echo 'root:${ROOT_PWD}' | chpasswd
# Replace to latest ORB_SLAM3 code by mount host volume
RUN rm -rf /dpds/ORB_SLAM3

# Install 
# glxgears: to test X display
RUN apt-get install mesa-utils gdb

CMD bash 