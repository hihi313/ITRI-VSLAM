Workspace set up for ITRI VSLAM project

# Camera calibration

* Can use [pre-built docker image][1], which has dependency *OpenCV(python)*
* `cd` to the directory of `get_K_and_D.py` to execut it

# IMU calibration

## Dependencies

1. ROS 1
2. Ceres
	* Above requirement can pull & use the [pre-built docker image][1] directly
3. code_utils
	* Clone the any one of the repo below
	* [hihi313/code_utils][2]
		* Fork from below repo & fix some compile error
	* [gaowenliang/code_utils][3]
		* Need to fix compile error
			* See issue for more info
4. imu_utils
	* [gaowenliang/imu_utils][4]

## Instructions

1. `mkdir -p catkin_ws/src`
2. clone code_utils & build via `catkin_make -j$(nproc)` in directory `catkin_ws`
3. clone imu_utils & build via `catkin_make -j$(nproc)` in directory `catkin_ws`

> Or use the [image][5] that has built these 2 repo

# Camera & IMU calibration

> Can use [kalibr][6] repo to build docker image
> > Or use the pre-built [image][7]

# Dockerfiles

## ros-kalibr

I build the image using official [GitHub][6] Dockerfile & it seems it need to **patch with `apt install python3-pyx`**

[1]: https://hub.docker.com/r/hihi313/ros-ceres (ros-ceres:latest)
[2]: https://github.com/hihi313/code_utils.git
[3]: https://github.com/gaowenliang/code_utils
[4]: https://github.com/gaowenliang/imu_utils.git
[5]: https://hub.docker.com/layers/ros-ceres/hihi313/ros-ceres/built/images/sha256-c27d7ae53e86c685837ce1feb675dc92d28907bdbaa8c742e1abcefa34ae55e6?context=explore (ros-ceres:built)
[6]: https://github.com/ethz-asl/kalibr.git
[7]: https://hub.docker.com/r/hihi313/kalibr (kalibr:latest)