FROM hihi313/ros-kalibr-ceres

ARG DEBIAN_FRONTEND=noninteractive
ARG DEPTHAI_URL=https://docs.luxonis.com
ARG DEPTHAI_SETUP_SCIRPT=install_dependencies.sh

WORKDIR /app

USER root

# Install dependencies
ADD $DEPTHAI_URL/$DEPTHAI_SETUP_SCIRPT ./
RUN apt update \
    && $DEPTHAI_SETUP_SCIRPT | bash
# Install DepthAI python SDK
RUN python3 -m pip install depthai

# Clean up
RUN apt-get clean \
    && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*
RUN rm -f ./$DEPTHAI_SETUP_SCIRPT

# Setup environment variable
COPY ./entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]
    
CMD bash 