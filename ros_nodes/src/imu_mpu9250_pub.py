import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Pose, Quaternion,PoseWithCovarianceStamped
import numpy as np
import math
from imusensor.MPU9250 import MPU9250
import smbus

class Nodo(object):
     def __init__(self):
        
        self.address = 0x68
        self.bus = smbus.SMBus(1)
        self.imu = MPU9250.MPU9250(self.bus, self.address)
        self.imu.begin()
        #self.imu.loadCalibDataFromFile("/home/sampi/Desktop/mup9250/calib.json")
        self.loop_rate = rospy.Rate(100)

        # Publishers
        self.pub = rospy.Publisher('imu_9250', Imu,queue_size=1)

     def start(self):
        rospy.loginfo("start")
        while not rospy.is_shutdown():
            self.imu.readSensor()
            self.imu.computeOrientation()
            msg=Imu()
            msg.header.stamp=rospy.Time.now()
            msg.angular_velocity.x=math.radians(self.imu.GyroVals[0])
            msg.angular_velocity.y=math.radians(self.imu.GyroVals[1])
            msg.angular_velocity.z=math.radians(self.imu.GyroVals[2])
            msg.linear_acceleration.x=self.imu.AccelVals[0]
            msg.linear_acceleration.y=self.imu.AccelVals[1]
            msg.linear_acceleration.z=self.imu.AccelVals[2]
            self.pub.publish(msg)
            print(msg.header.stamp)

            self.loop_rate.sleep()




if __name__ == '__main__':
        rospy.init_node("imu", anonymous=True)
        my_node = Nodo()
        my_node.start()
