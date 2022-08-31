#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <depthai/depthai.hpp>
#include "vslam.cpp"

using namespace std;
using namespace vslam;
namespace enc = sensor_msgs::image_encodings;

int main(int argc, char **argv)
{
    // node
    ros::init(argc, argv, "mono_R_publisher");
    ros::NodeHandle handle;
    int hz = 30;
    ros::Rate loop_rate(hz);

    // Publisher
    image_transport::ImageTransport it(handle);
    ImagePublisher imgPub = ImagePublisher(it, "/mono/right", 5, enc::MONO8, "camera2");

    // Pipeline
    string stream_name = "monoR";
    int size[] = {1152, 720};
    dai::Pipeline pipeline = get_pipeline(dai::CameraBoardSocket::RIGHT, stream_name, size);

    // Connect to device and start pipeline
    dai::Device device(pipeline);

    // Output queues
    shared_ptr<dai::DataOutputQueue> qRight = device.getOutputQueue(stream_name, 4, false);
    int i = 0, max = (hz / 10.0); // print in 10 hz
    while (ros::ok())
    {
        shared_ptr<dai::ImgFrame> imgL = qRight->get<dai::ImgFrame>();
        cv::Mat frame = imgL->getCvFrame();
        imgPub.publish(frame);
        // Print
        if (i < max)
        {
            ++i;
        }
        else
        {
            i = 0;
        }
        loop_rate.sleep();
    }
    return 0;
}