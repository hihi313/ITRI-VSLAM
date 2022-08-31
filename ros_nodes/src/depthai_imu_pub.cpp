#include <ros/ros.h>
#include <depthai/depthai.hpp>
#include "vslam.cpp"

using namespace std;
using namespace std::chrono;
using namespace dai;
using namespace vslam;

int main(int argc, char **argv)
{
    // node
    ros::init(argc, argv, "imu_publisher");
    ros::NodeHandle handle;
    int hz = 400;
    ros::Rate loop_rate(hz);

    // Publisher
    IMUPublisher imuPub = IMUPublisher(handle, "imu", "imu");

    // Pipeline
    IMUSensor sensors[] = {IMUSensor::ACCELEROMETER_RAW, IMUSensor::GYROSCOPE_RAW, IMUSensor::ROTATION_VECTOR};
    int hzs[] = {500, 500, 500};
    Pipeline pipeline = get_pipeline(sensors, hzs, false);
    Device d(pipeline);
    shared_ptr<DataOutputQueue> imuQueue = d.getOutputQueue("imu", 50, false);

    int i = 0, max = (hz / 10.0) - 1; // print in 10 hz
    while (ros::ok())
    {
        shared_ptr<IMUData> imuData = imuQueue->get<IMUData>();
        vector<IMUPacket> imuPackets = imuData->packets;
        for (IMUPacket &imuPacket : imuPackets)
        {
            // Publish
            imuPub.publish(imuPacket);

            // Print
            if (i >= max)
            {
                IMUReportAccelerometer &a = imuPacket.acceleroMeter;
                IMUReportGyroscope &g = imuPacket.gyroscope;
                steady_clock::duration aT = a.timestamp.get() - imuPub.baseTs;
                steady_clock::duration gT = g.timestamp.get() - imuPub.baseTs;
                printf("Acc[%ld ms]:\tx: %.3f\ty: %.3f\tz: %.3f[m/s^2]\n", toMs(aT), a.x, a.y, a.z);
                printf("Gyro[%ld ms]:\tx: %.3f\ty: %.3f\tz: %.3f[rad/s]\n", toMs(gT), g.x, g.y, g.z);
                i = 0;
            }
            ++i;
            loop_rate.sleep();
        }
    }

    return 0;
}