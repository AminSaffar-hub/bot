#!/usr/bin/env python
import rospy 
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
class Control : 
    def __init__(self):
        self.Wheel_radius = 0.065
        self.Wheel_separation_width = 0.267
        self.Wheel_separation_length = 0.167
        self.wheel_fr = 0.0
        self.wheel_fl = 0.0
        self.wheel_br = 0.0
        self.wheel_bl  = 0.0
        rospy.Subscriber("/odometry/filtered",Odometry,self.odom_calback,queue_size=10)
        self.pub = rospy.Publisher("/vel_pose",Float32MultiArray,queue_size=10)
        self.Vel_table = Float32MultiArray()
        self.Vel_table.data.append(0.0)
        self.Vel_table.data.append(0.0)
        self.Vel_table.data.append(0.0)
        self.Vel_table.data.append(0.0)
        

    def odom_calback(self,data):
        vx = data.twist.twist.linear.x
        vy = data.twist.twist.linear.y
        az = data.twist.twist.angular.z
        self.converting(vx,vy,az)
    
    def converting(self,linearx,lineary,angularz):
      	"""self.wheel_fl = (linearx - lineary - (self.Wheel_separation_width + self.Wheel_separation_length)* angularz) / self.Wheel_radius
        self.wheel_fr = (linearx + lineary + (self.Wheel_separation_width + self.Wheel_separation_length)* angularz) / self.Wheel_radius
        self.wheel_bl = (linearx + lineary - (self.Wheel_separation_width + self.Wheel_separation_length)* angularz) / self.Wheel_radius
        self.wheel_br = (linearx - lineary + (self.Wheel_separation_width + self.Wheel_separation_length)* angularz) / self.Wheel_radius
        """
        self.wheel_fl = (linearx - (self.Wheel_separation_width/2)* angularz) / self.Wheel_radius
        self.wheel_fr = (linearx + (self.Wheel_separation_width/2)* angularz) / self.Wheel_radius
        self.wheel_bl = self.wheel_fl
        self.wheel_br = self.wheel_fr
        self.Vel_table.data[0] = self.wheel_fl
        self.Vel_table.data[1] = self.wheel_bl
        self.Vel_table.data[2] = self.wheel_fr
        self.Vel_table.data[3] = self.wheel_br

        self.pub.publish(self.Vel_table)

        """#convert m/s to m/min
        linear_vel_x_mins_ = linearx * 60
        linear_vel_y_mins_ = lineary * 60
        #convert rad/s to rad/min
        angular_vel_z_mins_ = angularz * 60
        #Vt = w * radius
        tangential_vel_ = angular_vel_z_mins_ * self.Wheel_separation_width
        x_rpm_ = linear_vel_x_mins_ / (self.Wheel_radius*2*3.14)
        y_rpm_ = linear_vel_y_mins_ / (self.Wheel_radius*2*3.14)
        tan_rpm_ = tangential_vel_ / (self.Wheel_radius*2*3.14)

        #calculate for the target motor RPM and direction
        #front-left motor
        self.wheel_fl = (x_rpm_ - y_rpm_ - tan_rpm_)*(2*3.14/60)
        #rear-left motor
        self.wheel_bl = (x_rpm_ + y_rpm_ - tan_rpm_)*(2*3.14/60)
        #front-right motor
        self.wheel_fr = (x_rpm_ + y_rpm_ + tan_rpm_)*(2*3.14/60)
        #rear-right motor
        self.wheel_br = (x_rpm_ - y_rpm_ + tan_rpm_)*(2*3.14/60)"""
        

        #rospy.loginfo("wheelFL = %f , wheelBL = %f ,wheelFR = %f , wheelBR = %f "%(self.wheel_fl,self.wheel_bl,self.wheel_fr,self.wheel_br))

    
    def run(self):
        
        rospy.spin()






if __name__ == "__main__":
    rospy.init_node("Control_velocity",anonymous=True)
    A = Control()
    A.run()
    



