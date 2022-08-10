FROM hihi313/ros-ceres:latest AS rc

# app is default working dir
ARG CATKIN_WS=/app/catkin_ws
ARG N_THREAD=$(nproc)
ARG ROS_SRC=/opt/ros/noetic/setup.bash

USER root

COPY code_utils /app/catkin_ws/src/
RUN cd $CATKIN_WS \
    && source $ROS_SRC \
    && catkin_ws -j$N_THREAD
COPY imu_utils /app/catkin_ws/src/
RUN cd $CATKIN_WS \
    && source $ROS_SRC \
    && catkin_ws -j$N_THREAD

CMD bash