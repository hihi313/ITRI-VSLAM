#!/bin/bash
set -e

# setup ros environment
# $ROS_DISTRO defined in ROS's official image
source "/opt/ros/$ROS_DISTRO/setup.bash" --
# run the CMD
exec "$@"