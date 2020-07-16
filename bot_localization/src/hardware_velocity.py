#!/usr/bin/env python
import rospy 
from nav_msgs.msg import Odometry
class Control : 
    def __init__(self):
        self.Wheel_radius = 0.065
        self.Wheel_separation_width = 0.267/2
        self.Wheel_separation_length = 0.167/2
        self.wheel_fr = 0.0
        self.wheel_fl = 0.0
        self.wheel_br = 0.0
        self.wheel_bl  = 0.0

    def odom_calback(self,data):
        vx = data.twist.twist.linear.x
        vy = data.twist.twist.linear.y
        az = data.twist.twist.angular.z
        self.converting(vx,vy,az)
    
    def converting(self,linearx,lineary,angularz):
        self.wheel_fl =(linearx - lineary - (self.Wheel_separation_width + self.Wheel_separation_length)*angularz)
        self.wheel_fr =(linearx + lineary + (self.Wheel_separation_width + self.Wheel_separation_length)*angularz)
        self.wheel_bl =(linearx + lineary - (self.Wheel_separation_width + self.Wheel_separation_length)*angularz)
        self.wheel_br =(linearx - lineary + (self.Wheel_separation_width + self.Wheel_separation_length)*angularz)
        rospy.loginfo("wheelFR = %f , wheelFL = %f ,wheelBR = %f , wheelBL = %f "%(self.wheel_fr,self.wheel_fl,self.wheel_br,self.wheel_br))
    
    def run(self):
        rospy.Subscriber("/odometry/filtered",Odometry,self.odom_calback,queue_size=10)
        rospy.spin()






if __name__ == "__main__":
    rospy.init_node("Control_velocity",anonymous=True)
    A = Control()
    A.run()
    



