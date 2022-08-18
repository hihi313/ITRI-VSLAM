
#include "ros/ros.h"

#include <iostream>
#include <cstdio>
// #include "utility.hpp"
#include "sensor_msgs/Image.h"
#include <camera_info_manager/camera_info_manager.h>

// Inludes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"
#include <depthai_bridge/BridgePublisher.hpp>
#include <depthai_bridge/ImageConverter.hpp>

dai::Pipeline createPipeline(){
    dai::Pipeline pipeline;
    auto colorCam = pipeline.create<dai::node::ColorCamera>();
    auto xlinkOut = pipeline.create<dai::node::XLinkOut>();
    xlinkOut->setStreamName("video");
    
    colorCam->setPreviewSize(300, 300);
    colorCam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    colorCam->setInterleaved(false);

    // Link plugins CAM -> XLINK
    colorCam->video.link(xlinkOut->input);
    return pipeline;
}

nt main(int argc, char** argv){

    ros::init(argc, argv, "rgb_node");
    ros::NodeHandle h;

    return 0;
}