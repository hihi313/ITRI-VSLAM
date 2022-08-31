#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <depthai/depthai.hpp>

using namespace std;
namespace enc = sensor_msgs::image_encodings;

namespace vslam
{
    dai::Pipeline get_pipeline(dai::CameraBoardSocket socket, string stream_name, int size[])
    {
        // Create pipeline
        dai::Pipeline pipeline;

        // Define sources and outputs
        shared_ptr<dai::node::MonoCamera> mono = pipeline.create<dai::node::MonoCamera>();
        shared_ptr<dai::node::ImageManip> manip = pipeline.create<dai::node::ImageManip>();
        shared_ptr<dai::node::XLinkOut> manipOut = pipeline.create<dai::node::XLinkOut>();

        manipOut->setStreamName(stream_name);

        // Properties
        mono->setBoardSocket(socket);
        mono->setResolution(dai::MonoCameraProperties::SensorResolution::THE_800_P);
        manip->initialConfig.setResize(size[0], size[1]);

        //  Linking
        mono->out.link(manip->inputImage);
        manip->out.link(manipOut->input);

        return pipeline;
    }

    class Publisher
    {
    protected:
        string topic, frame_id;
    };

    class ImagePublisher : protected Publisher
    {
    private:
        string encoding;
        image_transport::Publisher pub;

    public:
        ImagePublisher(image_transport::ImageTransport &it, string topic, uint32_t queue_size,
                       string encoding, string frame_id = "")
        {
            this->topic = topic;
            this->frame_id = frame_id;
            this->encoding = encoding;
            this->pub = it.advertise(this->topic, queue_size);
        }
        void publish(cv::Mat &image)
        {
            cv_bridge::CvImage cvImg = cv_bridge::CvImage(std_msgs::Header(), this->encoding, image);
            cvImg.header.frame_id = this->frame_id;
            cvImg.header.stamp = ros::Time::now();
            this->pub.publish(cvImg.toImageMsg());
        }
    };
}