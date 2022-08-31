#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/image_encodings.h>
#include <depthai/depthai.hpp>

using namespace std;
using namespace std::chrono;

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
    dai::Pipeline get_pipeline(const dai::IMUSensor sensors[], const int hzs[],
                               bool sync = false, int reportTh = 20, int maxReport = 20)
    {
        // Create pipeline
        dai::Pipeline pipeline;

        // Define sources and outputs
        shared_ptr<dai::node::IMU> imu = pipeline.create<dai::node::IMU>();
        shared_ptr<dai::node::XLinkOut> xlinkOut = pipeline.create<dai::node::XLinkOut>();

        xlinkOut->setStreamName("imu");

        if (sync)
        {
            vector<dai::IMUSensor> s = vector<dai::IMUSensor>(sensors, sensors + size(sensors));
            imu->enableIMUSensor(s, *min_element(hzs, hzs + size(hzs)));
        }
        else
        {
            for (int i = 0; i < size(sensors); ++i)
            {
                imu->enableIMUSensor(sensors[i], hzs[i]);
            }
        }
        // it's recommended to set both setBatchReportThreshold and setMaxBatchReports to 20 when integrating in a pipeline with a lot of input/output connections
        // above this threshold packets will be sent in batch of X, if the host is not blocked and USB bandwidth is available
        imu->setBatchReportThreshold(reportTh);
        // maximum number of IMU packets in a batch, if it's reached device will block sending until host can receive it
        // if lower or equal to batchReportThreshold then the sending is always blocking on device
        // useful to reduce device's CPU load  and number of lost packets, if CPU load is high on device side due to multiple nodes
        imu->setMaxBatchReports(maxReport);

        // Link plugins IMU -> XLINK
        imu->out.link(xlinkOut->input);

        return pipeline;
    }
    // size of array
    template <typename T>
    int size(const T arr[])
    {
        return sizeof(arr) / sizeof(T);
    }

    long toMs(steady_clock::duration &duration)
    {
        return static_cast<long>(duration_cast<milliseconds>(duration).count());
    }

    class Publisher
    {
    protected:
        string topic, frame_id;
        uint32_t queue_size;
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

    class IMUPublisher : protected Publisher
    {
    private:
        ros::Publisher pub;

    public:
        bool firstTs;
        time_point<steady_clock, steady_clock::duration> baseTs;
        steady_clock::duration gMaxDelay, aMaxDelay;

        IMUPublisher(ros::NodeHandle &handle,
                     string topic, string frame_id = "")
        {
            this->topic = topic;
            this->frame_id = frame_id;

            this->pub = handle.advertise<sensor_msgs::Imu>(this->topic, 1000);

            this->firstTs = false;
            this->baseTs = time_point<steady_clock, steady_clock::duration>();
            this->gMaxDelay = steady_clock::duration::zero();
            this->aMaxDelay = steady_clock::duration::zero();
        }
        void publish(dai::IMUPacket packet, bool stat = false)
        {
            dai::IMUReportAccelerometer &a = packet.acceleroMeter;
            dai::IMUReportGyroscope &g = packet.gyroscope;
            dai::IMUReportRotationVectorWAcc &q = packet.rotationVector;

            time_point<steady_clock, steady_clock::duration> aT = a.timestamp.get();
            time_point<steady_clock, steady_clock::duration> gT = g.timestamp.get();
            time_point<steady_clock, steady_clock::duration> qT = q.timestamp.get();

            sensor_msgs::Imu msg;
            msg.orientation.w = q.real;
            msg.orientation.x = q.i;
            msg.orientation.y = q.j;
            msg.orientation.z = q.k;
            msg.orientation_covariance[0] = -1;
            msg.angular_velocity.x = g.x;
            msg.angular_velocity.y = g.y;
            msg.angular_velocity.z = g.z;
            msg.angular_velocity_covariance[0] = -1;
            msg.linear_acceleration.x = a.x;
            msg.linear_acceleration.y = a.y;
            msg.linear_acceleration.z = a.z;
            msg.linear_acceleration_covariance[0] = -1;
            msg.header.frame_id = this->frameId;
            msg.header.stamp = ros::Time::now();
            this->pub.publish(msg);
            // ROS_INFO("%s published", this->topic)

            // Compute max delay
            if (stat)
            {
                steady_clock::duration diff = gT - aT;
                if (diff > this->gMaxDelay)
                    this->gMaxDelay = diff;
                else if (-diff > this->aMaxDelay)
                    this->aMaxDelay = -diff;
            }
            // base time
            if (!this->firstTs)
            {
                this->baseTs = min(aT, gT);
                this->firstTs = true;
            }
        }
    };

    class PublisherNode : protected Publisher
    {
    protected:
        ros::NodeHandle handle;
    };

    class ImagePublisherNode : protected PublisherNode
    {
    private:
        string encoding;
        image_transport::ImageTransport &it;
        image_transport::Publisher pub;

    public:
        ImagePublisherNode(ros::NodeHandle &handle, string topic, uint32_t queue_size,
                           string encoding, string frame_id = "")
        {
            this->topic = topic;
            this->queue_size = queue_size;
            this->encoding = encoding;
            this->frame_id = frame_id;
            this->it = image_transport::ImageTransport(handle);
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