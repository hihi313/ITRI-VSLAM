#!/usr/bin/evn python
# TODO: add scale 

import cv2
import depthai as dai


def get_pipeline():
    # Create pipeline
    pipeline = dai.Pipeline()

    # Define sources and outputs
    monoLeft = pipeline.create(dai.node.MonoCamera)
    monoRight = pipeline.create(dai.node.MonoCamera)
    xoutLeft = pipeline.create(dai.node.XLinkOut)
    xoutRight = pipeline.create(dai.node.XLinkOut)

    xoutLeft.setStreamName('left')
    xoutRight.setStreamName('right')

    # Properties
    monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
    monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_800_P)
    monoRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)
    monoRight.setResolution(
        dai.MonoCameraProperties.SensorResolution.THE_800_P)

    # Linking
    monoRight.out.link(xoutRight.input)
    monoLeft.out.link(xoutLeft.input)

    return pipeline


if __name__ == '__main__':
    pipeline = get_pipeline()
    # Connect to device and start pipeline
    with dai.Device(pipeline) as device:

        # Output queues will be used to get the grayscale frames from the outputs defined above
        qLeft = device.getOutputQueue(name="left", maxSize=4, blocking=False)
        qRight = device.getOutputQueue(name="right", maxSize=4, blocking=False)

        while True:
            # Instead of get (blocking), we use tryGet (non-blocking) which will return the available data or None otherwise
            lFrames = qLeft.tryGetAll()
            for lFrame in lFrames:
                cv2.imshow("left", lFrame.getCvFrame())

            rFrames = qRight.tryGetAll()
            for rFrame in rFrames:
                cv2.imshow("right", rFrame.getCvFrame())

            if cv2.waitKey(1) == ord('q'):
                break
