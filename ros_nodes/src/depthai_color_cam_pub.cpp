#include <ros/ros.h>
#include <depthai/depthai.hpp>
#include <string>
#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h>

dai::Pipeline get_pipeline(int fps = 30)
{
    // Create pipeline
    dai::Pipeline pipeline;

    // Define sources and outputs
    auto colorCam = pipeline.create<dai::node::ColorCamera>();
    // auto videoEncoder = pipeline.create<dai::node::VideoEncoder>();
    // auto stillEncoder = pipeline.create<dai::node::VideoEncoder>();

    // auto controlIn = pipeline.create<dai::node::XLinkIn>();
    // auto configIn = pipeline.create<dai::node::XLinkIn>();
    // auto videoMjpegOut = pipeline.create<dai::node::XLinkOut>();
    // auto stillMjpegOut = pipeline.create<dai::node::XLinkOut>();
    // auto previewOut = pipeline.create<dai::node::XLinkOut>();
    auto ispOut = pipeline.create<dai::node::XLinkOut>();

    // controlIn->setStreamName("control");
    // configIn->setStreamName("config");
    // videoMjpegOut->setStreamName("video");
    // stillMjpegOut->setStreamName("still");
    // previewOut->setStreamName("preview");
    ispOut->setStreamName("isp");

    // Properties
    colorCam->setResolution(dai::ColorCameraProperties::SensorResolution.THE_12_MP);
    colorCam->setFps(fps);
    colorCam->setIspScale(4, 13);
    // colorCam->setVideoSize(640, 360);
    // colorCam->setPreviewSize(300, 300);
    // videoEncoder->setDefaultProfilePreset(colorCam->getFps(), dai::VideoEncoderProperties::Profile::MJPEG);
    // stillEncoder->setDefaultProfilePreset(1, dai::VideoEncoderProperties::Profile::MJPEG);

    // Linking
    colorCam->isp.link(ispOut->input);
    // colorCam->video.link(videoEncoder->input);
    // colorCam->still.link(stillEncoder->input);
    // colorCam->preview.link(previewOut->input);
    // controlIn->out.link(colorCam->inputControl);
    // configIn->out.link(colorCam->inputConfig);
    // videoEncoder->bitstream.link(videoMjpegOut->input);
    // stillEncoder->bitstream.link(stillMjpegOut->input);

    return pipeline;
}