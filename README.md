Dockerfiles for ITRI VSLAM project

# Camera calibration

* Can use [pre-built docker image][1], which has dependency *OpenCV(python)*
* `cd` to the directory of `get_K_and_D.py` to execut it

# IMU calibration

## Dependencies

1. ROS 1
2. Ceres
	* Above requirement can pull & use the [pre-built docker image][1] directly
3. code_utils
	* Clone the any one of the below repo & build using `catkin_make`
	* [hihi313/code_utils](https://github.com/hihi313/code_utils.git)
		* Fork from below repo & fix some compile error
	* [gaowenliang/code_utils](https://github.com/gaowenliang/code_utils)
		* Need to fix compile error
			* See issue for more info
4. imu_utils
	* Clone & build (via `catkin_make`)
	* [gaowenliang/imu_utils](https://github.com/gaowenliang/imu_utils.git)

# Camera & IMU calibration

> Can use [kalibr](https://github.com/ethz-asl/kalibr.git) repo to build docker image

> Pre-built [image](https://hub.docker.com/r/hihi313/kalibr)

[1]: https://hub.docker.com/r/hihi313/ros-ceres (ros & ceres image)
