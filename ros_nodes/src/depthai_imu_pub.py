#!/usr/bin/env python3

from datetime import timedelta
from struct import pack
from typing import List

import cv2
import depthai as dai
import rospy
from sensor_msgs.msg import Imu


def get_pipeline(sensors: List[dai.IMUSensor], hzs: List[int], sync: bool = False,
                 reportTh: int = 5, maxReport: int = 20) -> dai.Pipeline:
    # Create pipeline
    pipeline = dai.Pipeline()

    # Define sources and outputs
    imu = pipeline.create(dai.node.IMU)
    xlinkOut = pipeline.create(dai.node.XLinkOut)

    xlinkOut.setStreamName("imu")

    if (sync):
        imu.enableIMUSensor(sensors, min(hzs))
    else:
        for (sensor, hz) in zip(sensors, hzs):
            imu.enableIMUSensor(sensor, hz)
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


class IMUPublisher(object):
    def __init__(self, topic: str,  frameId: str = "") -> None:
        self.topic = topic
        self.frameId = frameId

        self.pubImu = rospy.Publisher(topic, Imu)
        self.baseT = None
        self.gMaxDelay = timedelta()
        self.aMaxDelay = timedelta()

        self.imuF = "{: .06f}"
        self.tsF = "{: .03f}"

    def publish(self, packet: dai.IMUPacket, printImu: bool = False, stat: bool = False) -> None:
        a = packet.acceleroMeter
        g = packet.gyroscope
        q = packet.rotationVector
        aT = a.timestamp.get()
        gT = g.timestamp.get()
        qT = q.timestamp.get()

        msg = Imu()
        msg.orientation.w = q.real
        msg.orientation.x = q.i
        msg.orientation.y = q.j
        msg.orientation.z = q.k
        msg.orientation_covariance[0] = -1
        msg.angular_velocity.x = g.x
        msg.angular_velocity.y = g.y
        msg.angular_velocity.z = g.z
        msg.angular_velocity_covariance[0] = -1
        msg.linear_acceleration.x = a.x
        msg.linear_acceleration.y = a.y
        msg.linear_acceleration.z = a.z
        msg.linear_acceleration_covariance[0] = -1
        msg.header.stamp = rospy.Time.now()
        self.pubImu.publish(msg)

        if stat:
            # Compute max delay
            diff = gT - aT
            if diff > self.gMaxDelay:
                self.gMaxDelay = diff
            elif -diff > self.aMaxDelay:
                self.aMaxDelay = -diff
        # base time
        if printImu:
            if self.baseT is None:
                self.baseT = aT
            # rospy.loginfo(f"{self.topic} published")
            print(aT, gT)

    def print(self, aT: timedelta, gT: timedelta) -> None:
        # Compute base time
        # if self.baseT is None:
        #     self.baseT = aT if aT < gT else gT

        print(
            f"Acc[{self.tsF.format(toMs(aT - self.baseT))} ms]: "
            f"\tx: {self.imuF.format(a.x)}\ty: {self.imuF.format(a.y)}\tz: {self.imuF.format(a.z)}[m/s^2]"
            f"\tGyro[{self.tsF.format(toMs(gT - self.baseT))} ms]: "
            f"\tx: {self.imuF.format(g.x)}\ty: {self.imuF.format(g.y)}\tz: {self.imuF.format(g.z)}[rad/s]",
            end="\r")


def toMs(delta: timedelta) -> float:
    return delta.total_seconds()*1000


if __name__ == '__main__':
    rospy.init_node("image_publisher", anonymous=True)
    hz = 400
    rate = rospy.Rate(hz)  # ? Hz

    pipeline = get_pipeline(sensors=[dai.IMUSensor.ROTATION_VECTOR,
                            dai.IMUSensor.ACCELEROMETER_RAW, dai.IMUSensor.GYROSCOPE_RAW], hzs=[500], sync=True)
    imuPub = IMUPublisher("imu", "imu")

    # Pipeline is defined, now we can connect to the device
    with dai.Device(pipeline) as device:

        # Output queue for imu bulk packets
        imuQueue = device.getOutputQueue(
            name="imu", maxSize=30, blocking=False)

        while not rospy.is_shutdown():
            imuData = imuQueue.get()  # blocking call, will wait until a new data has arrived

            for packet in imuData.packets:
                imuPub.publish(packet)
                rate.sleep()

        print(f"\nGyro max delay: {toMs(imuPub.gMaxDelay): .03f} ms")
        print(f"Acc max delay: {toMs(imuPub.aMaxDelay): .03f} ms")
