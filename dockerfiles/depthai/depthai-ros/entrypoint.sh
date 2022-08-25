#!/bin/bash
set -e

# setup ros environment
# $ROS_DISTRO defined in ROS's official image
echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
source ~/.bashrc
# run the CMD
exec "$@"