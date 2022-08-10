#按下a 幫你拍照
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import std_msgs.msg

class Nodo(object):
    def __init__(self):
        self.isstop = False
        self.t=std_msgs.msg.Header()
	#cam
        #self.cam = cv2.VideoCapture(0)
	# Params
        self.frame = None
        self.br = CvBridge()
        #threading.Thread(target=self.showimage, daemon=True, args=()).start()
        
        # Subscribers
        rospy.Subscriber('image',Image,self.callback,queue_size=1)
        rospy.spin()
        
    def callback(self, msg):
        #rospy.loginfo('Image received...')
        header = msg.header
        #timestamp = header.stamp.to_sec()                                                  
        t=rospy.Time.now()    
        ms=((float(t.nsecs)*1e-9)+ (t.secs-header.stamp.secs))*1000
        dt = ms-(float(header.stamp.nsecs)*1e-6)
        print("{:<5d}   {:>2.6f}".format(header.seq,dt))
             
        self.frame = self.br.imgmsg_to_cv2(msg,"mono8") 
        cv2.imshow('Capture',self.frame)

        key = cv2.waitKey(1)
        if key & 0x00FF  == ord('a'):
            cv2.imwrite("./魚眼相機標定/圖片/"+str(self.i)+'.jpg',self.frame)
            self.i=self.i+1
                
if __name__ == '__main__':
    rospy.init_node("roscam_sub", anonymous=True)
    my_node = Nodo()
    my_node.isstop = True
    cv2.destroyAllWindows()




