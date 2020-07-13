#!/usr/bin/env python
import time
import serial 
import rospy
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField
import tf
from std_msgs.msg import Float64


class imu_serial :
    def __init__(self):
        self.Ard = serial.Serial('/dev/ttyUSB0',115200)
        self.imu = Imu()
        self.imu.header.frame_id = "imu_link"
        self.Mag = MagneticField()
        self.output_node = Float64()
        self.dataPacket = None
        self.data_f = 0.0
        self.pub_imu = rospy.Publisher('/BNO055', Imu, queue_size=10)
        self.pub_tf = tf.TransformBroadcaster()
        rospy.init_node('serial_imu', anonymous=True)
        time.sleep(2)

    def heading(self):
        self.dataPacket= self.Ard.readline()
        self.dataPacket = str(self.dataPacket)
        self.data_f = float(self.dataPacket)
        return self.data_f

    def build_covariance(self,flag):
        matrix = []
        if flag == 2 :
            matrix.append(0.0001)
            matrix.append(0.0)
            matrix.append(0.0)
            matrix.append(0.0)
            matrix.append(0.0001)
            matrix.append(0.0)
            matrix.append(0.0)
            matrix.append(0.0)
            matrix.append(0.0001)
            return (matrix)
        elif flag == 1:
            matrix.append(0.00017)
            matrix.append(0.0)
            matrix.append(0.0)
            matrix.append(0.0)
            matrix.append(0.00017)
            matrix.append(0.0)
            matrix.append(0.0)
            matrix.append(0.0)
            matrix.append(0.00017)
            return (matrix)
        elif flag == 3 :
            matrix.append(0.00065)
            matrix.append(0.0)
            matrix.append(0.0)
            matrix.append(0.0)
            matrix.append(0.00065)
            matrix.append(0.0)
            matrix.append(0.0)
            matrix.append(0.0)
            matrix.append(0.00065)
            return (matrix)



        
    def build_messages_topics(self):
        self.dataPacket= self.Ard.readline()
        self.dataPacket = str(self.dataPacket)
        self.message_info = self.dataPacket.split(",")
        print (self.message_info)
        
        self.imu.orientation.x = float(self.message_info[10])
        self.imu.orientation.y = float(self.message_info[11])
        self.imu.orientation.z = float(self.message_info[12][0:-5])
        self.imu.orientation.w = float(self.message_info[9])
        self.imu.orientation_covariance = self.build_covariance(1)

        self.imu.angular_velocity.x = float(self.message_info[3])
        self.imu.angular_velocity.y = float(self.message_info[4])
        self.imu.angular_velocity.z = float(self.message_info[5])
        self.imu.angular_velocity_covariance = self.build_covariance(2)

        self.imu.linear_acceleration.x = float(self.message_info[0])
        self.imu.linear_acceleration.y = float(self.message_info[1])
        self.imu.linear_acceleration.z = float(self.message_info[2])
        self.imu.linear_acceleration_covariance = self.build_covariance(3)
        self.pub_tf.sendTransform((0,0,0),self.imu.orientation,rospy.Time.now,"imu_link","odom")
        self.pub_imu.publish(self.imu)

    
    



    def start_node(self):
        while not rospy.is_shutdown():
            self.dataPacket= self.Ard.readline()
    
            self.build_messages_topics()
            

if __name__ =='__main__' :
    A = imu_serial()
    A.start_node()


