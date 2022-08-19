#!/usr/bin/evn python
import cv2
import depthai as dai
import rospy
import std_msgs.msg
from cv_bridge import CvBridge, CvBridgeError
from numpy import ndarray
from sensor_msgs.msg import Image


def get_pipeline():
    # Create pipeline
    pipeline = dai.Pipeline()
    camRgb = pipeline.create(dai.node.ColorCamera)
    # Set the camera used?
    camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_12_MP)
    camRgb.setFps(30)
    # 4056*3040-> 1248*936
    # This scale has most things supported
    #   see the spreed sheet in
    #   https://docs.luxonis.com/projects/api/en/latest/components/nodes/color_camera/#limitations
    camRgb.setIspScale(4, 13)
    stillEncoder = pipeline.create(dai.node.VideoEncoder)

    controlIn = pipeline.create(dai.node.XLinkIn)
    ispOut = pipeline.create(dai.node.XLinkOut)
    stillMjpegOut = pipeline.create(dai.node.XLinkOut)

    controlIn.setStreamName('control')
    ispOut.setStreamName('isp')
    stillMjpegOut.setStreamName('still')

    # Properties
    stillEncoder.setDefaultProfilePreset(
        1, dai.VideoEncoderProperties.Profile.MJPEG)

    # Linking
    camRgb.isp.link(ispOut.input)
    camRgb.still.link(stillEncoder.input)
    controlIn.out.link(camRgb.inputControl)
    stillEncoder.bitstream.link(stillMjpegOut.input)

    return pipeline


class ImagePublisher(object):
    def __init__(self, topic: str,
                 frameId: str = "",
                 encoding: str = "passthrough") -> None:
        self.topic = topic
        self.frameId = frameId
        self.encoding = encoding

        self.cvbridge = CvBridge()
        self.pubImg = rospy.Publisher(topic, Image)

    def publish(self, image: ndarray) -> None:
        if not rospy.is_shutdown():
            try:
                img_msg = self.cvbridge.cv2_to_imgmsg(image, self.encoding)
            except CvBridgeError as e:
                print(e)
            h = std_msgs.msg.Header()
            h.stamp = rospy.Time.now()
            h.frame_id = self.frameId
            img_msg.header = h
            self.pubImg.publish(img_msg)
            rospy.loginfo("image published")


if __name__ == "__main__":
    rospy.init_node("image_publisher", anonymous=True)
    rate = rospy.Rate(2)  # 30 Hz

    pipeline = get_pipeline()
    imgPub = ImagePublisher("rgb_image")

    # Connect to device and start pipeline
    with dai.Device(pipeline) as device:
        # Get data queues
        controlQueue = device.getInputQueue('control')
        ispQueue = device.getOutputQueue('isp')
        stillQueue = device.getOutputQueue('still')

        while True:
            ispFrames = ispQueue.tryGetAll()
            for ispFrame in ispFrames:
                imgPub.publish(ispFrame.getCvFrame())
                # cv2.imshow('isp', ispFrame.getCvFrame())
            rate.sleep()
