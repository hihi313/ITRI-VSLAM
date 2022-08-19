# 按下a 幫你拍照
import cv2
import rospy
import std_msgs.msg
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class Nodo(object):
    def __init__(self):
        self.isstop = False
        self.t = std_msgs.msg.Header()
        # cam
        #self.cam = cv2.VideoCapture(0)
        # Params
        self.frame = None
        self.br = CvBridge()
        #threading.Thread(target=self.showimage, daemon=True, args=()).start()

        # Subscribers
        rospy.Subscriber('image', Image, self.callback, queue_size=1)
        rospy.spin()

    def callback(self, msg):
        #rospy.loginfo('Image received...')
        header = msg.header
        #timestamp = header.stamp.to_sec()
        t = rospy.Time.now()
        ms = ((float(t.nsecs)*1e-9) + (t.secs-header.stamp.secs))*1000
        dt = ms-(float(header.stamp.nsecs)*1e-6)
        print("{:<5d}   {:>2.6f}".format(header.seq, dt))

        self.frame = self.br.imgmsg_to_cv2(msg, "mono8")
        cv2.imshow('Capture', self.frame)

        key = cv2.waitKey(1)
        if key & 0x00FF == ord('a'):
            cv2.imwrite("./fisheye_calibration/images/" +
                        str(self.i)+'.jpg', self.frame)
            self.i = self.i+1


class ImageSubscriber(object):
    def __init__(self, topic: str) -> None:
        self.topic = topic
        self.cvbridge = CvBridge()
        self.subsImg = rospy.Subscriber(topic, Image, self.callback)
        rospy.spin()

    def callback(msg) -> None:


if __name__ == '__main__':
    rospy.init_node("image_subscriber", anonymous=True)
    my_node = Nodo()
    my_node.isstop = True
    cv2.destroyAllWindows()
