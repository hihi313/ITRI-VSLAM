#include <ros/ros.h>
#include <depthai/depthai.hpp>
#include <string>
#include "sensor_msgs/Imu.h"

using namespace std;
using namespace std::chrono;
using namespace dai;

template <typename T>
int size(const T arr[])
{
    return sizeof(arr) / sizeof(T);
}

Pipeline get_pipeline(const IMUSensor sensors[], const int hzs[],
                      bool sync = false, int reportTh = 20, int maxReport = 20)
{
    // Create pipeline
    Pipeline pipeline;

    // Define sources and outputs
    shared_ptr<node::IMU> imu = pipeline.create<node::IMU>();
    shared_ptr<node::XLinkOut> xlinkOut = pipeline.create<node::XLinkOut>();

    xlinkOut->setStreamName("imu");

    if (sync)
    {
        vector<IMUSensor> s = vector<IMUSensor>(sensors, sensors + size(sensors));
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

long toMs(steady_clock::duration &duration)
{
    return static_cast<long>(duration_cast<milliseconds>(duration).count());
}

class IMUPublisher
{
private:
    ros::Publisher pub;
    bool firstTs;
    time_point<steady_clock, steady_clock::duration> baseTs;
    steady_clock::duration gMaxDelay, aMaxDelay;

public:
    string topic, frameId;
    IMUPublisher(ros::NodeHandle &handle,
                 string topic, string frameId = "")
    {
        this->topic = topic;
        this->frameId = frameId;

        this->pub = handle.advertise<sensor_msgs::Imu>(this->topic, 1000);
        this->firstTs = false;
        this->baseTs = time_point<steady_clock, steady_clock::duration>();
        this->gMaxDelay = steady_clock::duration::zero();
        this->aMaxDelay = steady_clock::duration::zero();
    }
    void publish(IMUPacket packet, bool print = false, bool stat = false)
    {
        IMUReportAccelerometer &a = packet.acceleroMeter;
        IMUReportGyroscope &g = packet.gyroscope;

        time_point<steady_clock, steady_clock::duration> aT = a.timestamp.get();
        time_point<steady_clock, steady_clock::duration> gT = g.timestamp.get();

        sensor_msgs::Imu msg;
        msg.orientation_covariance[0] = -1;
        msg.angular_velocity.x = g.x;
        msg.angular_velocity.y = g.y;
        msg.angular_velocity.z = g.z;
        msg.angular_velocity_covariance[0] = -1;
        msg.linear_acceleration.x = a.x;
        msg.linear_acceleration.y = a.y;
        msg.linear_acceleration.z = a.z;
        msg.linear_acceleration_covariance[0] = -1;
        msg.header.stamp = ros::Time::now();
        this->pub.publish(msg);
    }
};

int main(int argc, char **argv)
{
    // node
    ros::init(argc, argv, "imu_publisher");
    ros::NodeHandle handle;
    int hz = 400 ;
    ros::Rate loop_rate(hz);

    // Pipeline
    IMUSensor sensors[] = {IMUSensor::ACCELEROMETER_RAW, IMUSensor::GYROSCOPE_RAW, IMUSensor::ROTATION_VECTOR};
    int hzs[] = {500};
    Pipeline pipeline = get_pipeline(sensors, hzs, true);

    // Publisher
    IMUPublisher imuPub = IMUPublisher(handle, "imu", "imu");

    // Pipeline is defined, now we can connect to the device
    Device d(pipeline);

    bool firstTs = false;

    shared_ptr<DataOutputQueue> imuQueue = d.getOutputQueue("imu", 50, false);
    time_point<steady_clock, steady_clock::duration> baseTs = time_point<steady_clock, steady_clock::duration>();
    while (ros::ok())
    {
        shared_ptr<IMUData> imuData = imuQueue->get<IMUData>();

        vector<IMUPacket> imuPackets = imuData->packets;
        for (IMUPacket &imuPacket : imuPackets)
        {
            // IMUReportAccelerometer &acceleroValues = imuPacket.acceleroMeter;
            // IMUReportGyroscope &gyroValues = imuPacket.gyroscope;

            // time_point<steady_clock, steady_clock::duration> acceleroTs1 = acceleroValues.timestamp.get();
            // time_point<steady_clock, steady_clock::duration> gyroTs1 = gyroValues.timestamp.get();
            // if (!firstTs)
            // {
            //     baseTs = min(acceleroTs1, gyroTs1);
            //     firstTs = true;
            // }

            // steady_clock::duration acceleroTs = acceleroTs1 - baseTs;
            // steady_clock::duration gyroTs = gyroTs1 - baseTs;

            // printf("Accelerometer timestamp: %ld ms\n", toMs(acceleroTs));
            // printf("Accelerometer [m/s^2]: x: %.3f y: %.3f z: %.3f \n", acceleroValues.x, acceleroValues.y, acceleroValues.z);
            // printf("Gyroscope timestamp: %ld ms\n", static_cast<long>(duration_cast<milliseconds>(gyroTs).count()));
            // printf("Gyroscope [rad/s]: x: %.3f y: %.3f z: %.3f \n", gyroValues.x, gyroValues.y, gyroValues.z);

            // Publish
            imuPub.publish(imuPacket);
            loop_rate.sleep();
        }
    }

    return 0;
}