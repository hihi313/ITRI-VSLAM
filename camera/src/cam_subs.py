import os

import cv2
import rospy
import std_msgs.msg
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


class ImageSubscriber(object):
    def __init__(self, cvbridge: CvBridge,
                 topic: str,
                 filename: str = "{0}.jpg",
                 path: str = "images",
                 encoding: str = "passthrough") -> None:
        self.cvbridge = cvbridge
        self.topic = topic
        self.filename = filename
        self.path = path
        self.encoding = encoding

        self.subsImg = rospy.Subscriber(topic, Image, self.callback)

    def callback(self, msg: Image) -> None:
        try:
            img = self.cvbridge.imgmsg_to_cv2(msg, self.encoding)
        except CvBridgeError as e:
            print(e)
        cv2.imshow(f"Subscriber :{self.topic}", img)

        # Save image
        key = cv2.waitKey(1)
        # Press a to save image
        if key & 0x00FF == ord('a'):
            time = msg.header.stamp
            fname = self.filename.format(
                str(time.secs) + "_" + str(time.nsecs))
            cv2.imwrite(os.path.join(self.path, fname), img)
            rospy.loginfo("image saved")


if __name__ == '__main__':
    rospy.init_node("image_subscriber", anonymous=True)
    path = os.path.dirname(os.path.abspath(__file__))
    cvbridge = CvBridge()
    ispSubs = ImageSubscriber(cvbridge,
                              "color/isp", filename="isp_{0}.jpg", path=os.path.join(path, os.pardir, "images"))
    videoSubs = ImageSubscriber(cvbridge,
                                "color/video", filename="video_{0}.jpg", path=os.path.join(path, os.pardir, "images"))
    rospy.spin()
    # cv2.destroyAllWindows()
