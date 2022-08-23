#!/usr/bin/env python3

from datetime import timedelta

import cv2
import depthai as dai
import rospy


def get_pipeline(accHz: int = 500, gyroHz: int = 400, reportTh: int = 5, maxReport: int = 20) -> dai.Pipeline:
    # Create pipeline
    pipeline = dai.Pipeline()

    # Define sources and outputs
    imu = pipeline.create(dai.node.IMU)
    xlinkOut = pipeline.create(dai.node.XLinkOut)

    xlinkOut.setStreamName("imu")

    # enable ACCELEROMETER_RAW at 500 hz rate
    imu.enableIMUSensor(dai.IMUSensor.ACCELEROMETER_RAW, accHz)
    # enable GYROSCOPE_RAW at 400 hz rate
    imu.enableIMUSensor(dai.IMUSensor.GYROSCOPE_RAW, gyroHz)
    # it's recommended to set both setBatchReportThreshold and setMaxBatchReports to 20 when integrating in a pipeline with a lot of input/output connections
    # above this threshold packets will be sent in batch of X, if the host is not blocked and USB bandwidth is available
    imu.setBatchReportThreshold(reportTh)
    # maximum number of IMU packets in a batch, if it's reached device will block sending until host can receive it
    # if lower or equal to batchReportThreshold then the sending is always blocking on device
    # useful to reduce device's CPU load  and number of lost packets, if CPU load is high on device side due to multiple nodes
    imu.setMaxBatchReports(maxReport)

    # Link plugins IMU -> XLINK
    imu.out.link(xlinkOut.input)

    return pipeline


def timeDeltaToMilliS(delta) -> float:
    return delta.total_seconds()*1000


if __name__ == '__main__':
    rospy.init_node("image_publisher", anonymous=True)
    rate = rospy.Rate(300)  # ? Hz

    pipeline = get_pipeline()

    # Pipeline is defined, now we can connect to the device
    with dai.Device(pipeline) as device:

        # Output queue for imu bulk packets
        imuQueue = device.getOutputQueue(
            name="imu", maxSize=30, blocking=False)
        baseTs = None
        maxDiff = timedelta()
        while True:
            imuData = imuQueue.get()  # blocking call, will wait until a new data has arrived

            imuPackets = imuData.packets
            for imuPacket in imuPackets:
                acceleroValues = imuPacket.acceleroMeter
                gyroValues = imuPacket.gyroscope

                acceleroTs = acceleroValues.timestamp.get()
                gyroTs = gyroValues.timestamp.get()
                if abs(gyroTs - acceleroTs) > maxDiff:
                    maxDiff = abs(gyroTs - acceleroTs)
                print(f"maxDiff: {timeDeltaToMilliS(maxDiff)}")
                if baseTs is None:
                    baseTs = acceleroTs if acceleroTs < gyroTs else gyroTs
                acceleroTs = timeDeltaToMilliS(acceleroTs - baseTs)
                gyroTs = timeDeltaToMilliS(gyroTs - baseTs)

                imuF = "{:.06f}"
                tsF = "{:.03f}"

                print(f"Accelerometer timestamp: {tsF.format(acceleroTs)} ms")
                print(
                    f"Accelerometer [m/s^2]: x: {imuF.format(acceleroValues.x)} y: {imuF.format(acceleroValues.y)} z: {imuF.format(acceleroValues.z)}")
                print(f"Gyroscope timestamp: {tsF.format(gyroTs)} ms")
                print(
                    f"Gyroscope [rad/s]: x: {imuF.format(gyroValues.x)} y: {imuF.format(gyroValues.y)} z: {imuF.format(gyroValues.z)} ")
            if cv2.waitKey(1) == ord('q'):
                break
