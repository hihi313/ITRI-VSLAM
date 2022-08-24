#!/usr/bin/evn python
from typing import Tuple

import cv2
import depthai as dai
import rospy
from cv_bridge import CvBridge, CvBridgeError

from depthai_color_cam_pub import ImagePublisher


def get_pipeline(size: Tuple[int, int]):
    # Create pipeline
    pipeline = dai.Pipeline()

    # Define sources and outputs
    monoRight = pipeline.create(dai.node.MonoCamera)
    monoLeft = pipeline.create(dai.node.MonoCamera)
    manipRight = pipeline.create(dai.node.ImageManip)
    manipLeft = pipeline.create(dai.node.ImageManip)

    # controlIn = pipeline.create(dai.node.XLinkIn)
    configIn = pipeline.create(dai.node.XLinkIn)
    manipOutRight = pipeline.create(dai.node.XLinkOut)
    manipOutLeft = pipeline.create(dai.node.XLinkOut)

    # controlIn.setStreamName('control')
    configIn.setStreamName('config')
    manipOutRight.setStreamName("right")
    manipOutLeft.setStreamName("left")

    # Properties
    monoRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)
    monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
    monoRight.setResolution(
        dai.MonoCameraProperties.SensorResolution.THE_800_P)
    monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_800_P)
    # manipRight.initialConfig.setKeepAspectRatio(True)
    manipRight.initialConfig.setResize(size[0], size[1])
    manipLeft.initialConfig.setResize(size[0], size[1])
    manipRight.setMaxOutputFrameSize(
        monoRight.getResolutionHeight()*monoRight.getResolutionWidth()*3)

    # Linking
    monoRight.out.link(manipRight.inputImage)
    monoLeft.out.link(manipLeft.inputImage)
    # controlIn.out.link(monoRight.inputControl)
    # controlIn.out.link(monoLeft.inputControl)
    configIn.out.link(manipRight.inputConfig)
    configIn.out.link(manipLeft.inputConfig)
    manipRight.out.link(manipOutRight.input)
    manipLeft.out.link(manipOutLeft.input)

    return pipeline


if __name__ == '__main__':
    rospy.init_node("image_publisher", anonymous=True)
    rate = rospy.Rate(10)  # ? Hz

    pipeline = get_pipeline((1152, 720))
    cvbridge = CvBridge()
    lPub = ImagePublisher(cvbridge, "mono/left", "camera1")
    rPub = ImagePublisher(cvbridge, "mono/right", "camera2")

    # Connect to device and start pipeline
    with dai.Device(pipeline) as device:

        # Output queues will be used to get the grayscale frames from the outputs defined above
        qLeft = device.getOutputQueue(name="left", maxSize=4, blocking=False)
        qRight = device.getOutputQueue(name="right", maxSize=4, blocking=False)

        while not rospy.is_shutdown():
            # Instead of get (blocking), we use tryGet (non-blocking) which will return the available data or None otherwise
            lFrames = qLeft.tryGetAll()
            for lFrame in lFrames:
                frame = lFrame.getCvFrame()
                # cv2.imshow("left", frame)
                lPub.publish(frame)

            rFrames = qRight.tryGetAll()
            for rFrame in rFrames:
                frame = rFrame.getCvFrame()
                # cv2.imshow("right", frame)
                rPub.publish(frame)

            if cv2.waitKey(1) == ord('q'):
                break

            rate.sleep()
