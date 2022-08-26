#include <ros/ros.h>
#include <depthai/depthai.hpp>
#include <string>
//#include <bits/stdc++.h>

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

long toMs(time_point<steady_clock, steady_clock::duration> &duration)
{
    auto tmp = duration_cast<milliseconds>(duration);
    //duration_cast<milliseconds>(time_point<steady_clock, duration<long int, miliseconds> >&)
    return static_cast<long>(duration_cast<milliseconds>(duration).count());
}

class IMUPublisher
{
private:
public:
    string topic, frameId;
};

int main()
{

    IMUSensor sensors[] = {IMUSensor::ACCELEROMETER_RAW, IMUSensor::GYROSCOPE_RAW, IMUSensor::ROTATION_VECTOR};
    int hzs[] = {500};
    Pipeline pipeline = get_pipeline(sensors, hzs, true);

    // Pipeline is defined, now we can connect to the device
    Device d(pipeline);

    bool firstTs = false;

    shared_ptr<DataOutputQueue> imuQueue = d.getOutputQueue("imu", 50, false);
    time_point<steady_clock, steady_clock::duration> baseTs = time_point<steady_clock, steady_clock::duration>();

    while (true)
    {
        shared_ptr<IMUData> imuData = imuQueue->get<IMUData>();

        vector<IMUPacket> imuPackets = imuData->packets;
        for (IMUPacket &imuPacket : imuPackets)
        {
            IMUReportAccelerometer &acceleroValues = imuPacket.acceleroMeter;
            IMUReportGyroscope &gyroValues = imuPacket.gyroscope;

            time_point<steady_clock, steady_clock::duration> acceleroTs1 = acceleroValues.timestamp.get();
            time_point<steady_clock, steady_clock::duration> gyroTs1 = gyroValues.timestamp.get();
            if (!firstTs)
            {
                baseTs = std::min(acceleroTs1, gyroTs1);
                firstTs = true;
            }

            time_point<steady_clock, steady_clock::duration> acceleroTs = acceleroTs1 - baseTs;
            time_point<steady_clock, steady_clock::duration> gyroTs = gyroTs1 - baseTs;

            printf("Accelerometer timestamp: %ld ms\n", toMs(acceleroTs));
            printf("Accelerometer [m/s^2]: x: %.3f y: %.3f z: %.3f \n", acceleroValues.x, acceleroValues.y, acceleroValues.z);
            printf("Gyroscope timestamp: %ld ms\n", static_cast<long>(duration_cast<milliseconds>(gyroTs).count()));
            printf("Gyroscope [rad/s]: x: %.3f y: %.3f z: %.3f \n", gyroValues.x, gyroValues.y, gyroValues.z);
        }
    }

    return 0;
}