#include <ros/ros.h>
#include <depthai/depthai.hpp>
#include "vslam.cpp"

using namespace std;

dai::Pipeline get_stereo_rgb_imu_pipeline(string rgbStreamName, string monoLStreamName, string monoRStreamName, string imuStreamName,
                                          dai::ColorCameraProperties::SensorResolution rgbResolution, int rgbFps, const int rgbScale[],
                                          dai::MonoCameraProperties::SensorResolution monoResolution, int monoFps, const int monoSize[],
                                          const dai::IMUSensor sensors[], const int hzs[], bool sync = false, int reportTh = 20, int maxReport = 20)
{
    // Create pipeline
    dai::Pipeline pipeline;

    // Define sources and outputs
    // RGB camera
    shared_ptr<dai::node::ColorCamera> colorCam = pipeline.create<dai::node::ColorCamera>();
    shared_ptr<dai::node::XLinkOut> ispOut = pipeline.create<dai::node::XLinkOut>();
    // Mono left
    shared_ptr<dai::node::MonoCamera> monoL = pipeline.create<dai::node::MonoCamera>();
    shared_ptr<dai::node::ImageManip> manipL = pipeline.create<dai::node::ImageManip>();
    shared_ptr<dai::node::XLinkOut> manipOutL = pipeline.create<dai::node::XLinkOut>();
    // Mono right
    shared_ptr<dai::node::MonoCamera> monoR = pipeline.create<dai::node::MonoCamera>();
    shared_ptr<dai::node::ImageManip> manipR = pipeline.create<dai::node::ImageManip>();
    shared_ptr<dai::node::XLinkOut> manipOutR = pipeline.create<dai::node::XLinkOut>();
    // IMU
    shared_ptr<dai::node::IMU> imu = pipeline.create<dai::node::IMU>();
    shared_ptr<dai::node::XLinkOut> imuOut = pipeline.create<dai::node::XLinkOut>();

    // Set stream name
    ispOut->setStreamName(rgbStreamName);
    manipOutL->setStreamName(monoLStreamName);
    manipOutR->setStreamName(monoRStreamName);
    imuOut->setStreamName(imuStreamName);

    // Properties
    // RGB camera
    colorCam->setResolution(rgbResolution);
    colorCam->setFps(rgbFps);
    colorCam->setIspScale(rgbScale[0], rgbScale[1]);
    // Mono left
    monoL->setBoardSocket(dai::CameraBoardSocket::LEFT);
    monoL->setResolution(monoResolution);
    monoL->setFps(monoFps);
    manipL->initialConfig.setResize(monoSize[0], monoSize[1]);
    // Mono right
    monoL->setBoardSocket(dai::CameraBoardSocket::RIGHT);
    monoL->setResolution(monoResolution);
    monoL->setFps(monoFps);
    manipL->initialConfig.setResize(monoSize[0], monoSize[1]);
    // IMU
    if (sync)
    {
        vector<dai::IMUSensor> s = vector<dai::IMUSensor>(sensors, sensors + vslam::size(sensors));
        imu->enableIMUSensor(s, *min_element(hzs, hzs + vslam::size(hzs)));
    }
    else
    {
        for (int i = 0; i < vslam::size(sensors); ++i)
            imu->enableIMUSensor(sensors[i], hzs[i]);
    }
    // it's recommended to set both setBatchReportThreshold and setMaxBatchReports to 20 when integrating in a pipeline with a lot of input/output connections
    // above this threshold packets will be sent in batch of X, if the host is not blocked and USB bandwidth is available
    imu->setBatchReportThreshold(reportTh);
    // maximum number of IMU packets in a batch, if it's reached device will block sending until host can receive it
    // if lower or equal to batchReportThreshold then the sending is always blocking on device
    // useful to reduce device's CPU load  and number of lost packets, if CPU load is high on device side due to multiple nodes
    imu->setMaxBatchReports(maxReport);

    //  Linking
    colorCam->isp.link(ispOut->input);
    monoL->out.link(manipL->inputImage);
    manipL->out.link(manipOutL->input);
    monoR->out.link(manipR->inputImage);
    manipR->out.link(manipOutR->input);
    imu->out.link(xlinkOut->input);

    return pipeline;
}

int main(int argc, char **argv)
{
    // node
    ros::init(argc, argv, "depthai_stereo_rgb_imu");
    return 0;
}