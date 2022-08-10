FROM hihi313/ros-ceres:latest

# app is default working dir
ARG CATKIN_WS=/app/catkin_ws
ARG N_THREAD=$(nproc)

COPY code_utils /app/catkin_ws/src/
RUN cd $CATKIN_WS \
    && catkin_ws -j$N_THREAD
COPY imu_utils /app/catkin_ws/src/
RUN cd $CATKIN_WS \
    && catkin_ws -j$N_THREAD

CMD bash