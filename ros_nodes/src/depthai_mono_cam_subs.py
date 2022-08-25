import os

import cv2
import rospy
import std_msgs.msg
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from depthai_color_cam_subs import ImageSubscriber


if __name__ == '__main__':
    rospy.init_node("image_subscriber", anonymous=True)
    path = os.path.dirname(os.path.abspath(__file__))
    cvbridge = CvBridge()
    lSubs = ImageSubscriber(cvbridge,
                              "mono/left", filename="monoL_{0}.jpg", path=os.path.join(path, os.pardir, "images"))
    rSubs = ImageSubscriber(cvbridge,
                                "mono/right", filename="monoR_{0}.jpg", path=os.path.join(path, os.pardir, "images"))
    rospy.spin()
    # cv2.destroyAllWindows()
