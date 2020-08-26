#!/usr/bin/env python
import rospy 
from std_msgs.msg import Float32MultiArray

class PID_Control:

    def appending_variables(self,obj) :
        for i in range(0,4) :
            obj.append(0.0)
    
    def __init__(self):
        self._KP = 0.02
        self._KD = 0.0
        self._KI = 0.0005

        self._Vc = []
        self._Vpos = []

        self._prev_error = []
        self._error = []
        self._sum_error = []
        self._diff_error = []

        self._cmd = []
        self._pwm_base = []
        self._pwm = []

        self.appending_variables(self._Vc)
        self.appending_variables(self._Vpos)
        self.appending_variables(self._prev_error)
        self.appending_variables(self._error)
        self.appending_variables(self._sum_error)
        self.appending_variables(self._diff_error)
        self.appending_variables(self._cmd)
        #self.appending_variables(self._pwm_base)
        self._pwm_base.append(80.0)
        self._pwm_base.append(80.0)
        self._pwm_base.append(80.0)
        self._pwm_base.append(80.0)
        self.appending_variables(self._pwm)

        rospy.init_node('pwm_controller', anonymous=True)
        self.pub = rospy.Publisher('/pwm_cmd',Float32MultiArray,queue_size=10)
        self._pwm_pub = Float32MultiArray()
        self.appending_variables(self._pwm_pub.data)
        rospy.Subscriber('/vel_cmd',Float32MultiArray,self.command_vel,queue_size=10)
        rospy.Subscriber('/vel_pose',Float32MultiArray,self.pos_vel,queue_size=10)


    def command_vel(self,tab):
        self._Vc[0] = tab.data[0]
        self._Vc[1] = tab.data[1]
        self._Vc[2] = tab.data[2]
        self._Vc[3] = tab.data[3]
        

    def pos_vel(self,tab_pos) :
        self._Vpos[0] = tab_pos.data[0]
        self._Vpos[1] = tab_pos.data[1]
        self._Vpos[2] = tab_pos.data[2]
        self._Vpos[3] = tab_pos.data[3]
        

    
    
    
    def set_in_interval(self):
        self._cmd[2] *= 2.5 
        self._cmd[3] *= 2.5
        for i in range(0,4) :
           # if (i == 2) or (i == 3): self._cmd += 30 
            self._pwm_base[i] =  self._pwm_base[i] + self._cmd[i]
            
            if self._pwm_base[i] > 255.0 :
                self._pwm_base[i] = 255.0 
            elif self._pwm_base[i] < -255.0 :
                self._pwm_base[i] = -255.0 


            
    


    def velocity_control(self):

        for i in range(0,4) :
            self._error[i] = self._Vc[i] - self._Vpos[i]
            self._sum_error[i] = self._sum_error[i] + self._error[i]
            self._diff_error[i] = self._error[i] - self._prev_error[i]
            self._prev_error[i] = self._error[i]
            
            self._cmd[i] = self._KP *self._error[i] + self._KI *self._error[i] + self._KD *self._error[i]
            rospy.loginfo(self._error)
            
            # self._pwm_pub.data[1] = 0.0
            # self._pwm_pub.data[2] = 0.0
            # self._pwm_pub.data[3] = 0.0
        if self._Vc == [0.0,0.0,0.0,0.0] :  
            self._pwm_pub.data[0] = 0.0
            self._pwm_pub.data[1] = 0.0
            self._pwm_pub.data[2] = 0.0
            self._pwm_pub.data[3] = 0.0
            self._pwm_base[0] = 80.0
            self._pwm_base[1] = 80.0
            self._pwm_base[2] = 80.0
            self._pwm_base[3] = 80.0
        else :
            self.set_in_interval()
            self._pwm_pub.data[0] = self._pwm_base[0] # left 
            self._pwm_pub.data[1] = self._pwm_base[2] # right 
            self._pwm_pub.data[2] = self._pwm_base[3] # right B
            self._pwm_pub.data[3] = self._pwm_base[1] # left B

        self.pub.publish(self._pwm_pub)
    
    def run_node(self):
        r = rospy.Rate(50)
        while not rospy.is_shutdown() :
            self.velocity_control()
            r.sleep()


if __name__ == "__main__":
    A = PID_Control()
    A.run_node()