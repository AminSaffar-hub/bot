#!/usr/bin/env python
import time
import serial 
import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64


class imu_serial :
    def __init__(self):
        self.Ard = serial.Serial('/dev/ttyUSB0',115200)

        self.output_node = Float64()
        self.dataPacket = None
        self.data_f = 0.0
        self.pub_imu = rospy.Publisher('/BNO055', Float64, queue_size=10)
        rospy.init_node('serial_imu', anonymous=True)
        time.sleep(2)

    def heading(self):
        self.dataPacket= self.Ard.readline()
        self.dataPacket = str(self.dataPacket)
        self.data_f = float(self.dataPacket)
        return self.data_f

    def start_node(self):
        while not rospy.is_shutdown():
            self.dataPacket= self.Ard.readline()
            self.data_f = self.heading()
            self.output_node.data = self.data_f
            rospy.loginfo(self.data_f)
            self.pub_imu.publish(self.output_node)
            

if __name__ =='__main__' :
    A = imu_serial()
    A.start_node()


