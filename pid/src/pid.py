#!/usr/bin/env python
import rospy 
from std_msgs.msg import Float32MultiArray

from dynamic_reconfigure.server import Server
from dynamic_tutorials.cfg import TutorialsConfig



class PID_Control:

    def appending_variables(self,obj) :
        for i in range(0,4) :
            obj.append(0.0)
    
    def __init__(self):
        rospy.init_node('pwm_controller', anonymous=True)
        self.srv = Server(TutorialsConfig, self.callback)
        self._KP = 1.0
        self._KD = 0.0
        self._KI = 0.0

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
        self.appending_variables(self._pwm_base)
        self.appending_variables(self._pwm)

        
        self.pub = rospy.Publisher('/pwm_cmd',Float32MultiArray,queue_size=10)
        self._pwm_pub = Float32MultiArray()
        self.appending_variables(self._pwm_pub.data)
        rospy.Subscriber('/vel_cmd',Float32MultiArray,self.command_vel,queue_size=10)
        rospy.Subscriber('/vel_pos',Float32MultiArray,self.pos_vel,queue_size=10)


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

    def callback(self,config, level):
        rospy.loginfo("""Reconfigure Request: {k_p}, {k_i}, 
            {k_d}""".format(**config))
        self._KP = config["k_p"]
        self._KI = config["k_i"]
        self._KD = config["k_d"]
        print(self._KP)
        print(self._KI)
        print(self._KD)
        return config
    
    
    
    def set_in_interval(self):

        for i in range(0,4) :
            self._pwm_base[i] = self._pwm_base[i] + self._cmd[i]
            
            if self._pwm_base[i] > 255.0 :
                self._pwm_base[i] = 255.0 
            elif self._pwm_base[i] < -255.0 :
                self._pwm_base[i] = -255.0 
        rospy.loginfo(self._pwm_base)    
    


    def velocity_control(self):

        for i in range(0,4) :
            self._error[i] = self._Vc[i] - self._Vpos[i]
            self._sum_error[i] = self._sum_error[i] + self._error[i]
            self._diff_error[i] = self._error[i] - self._prev_error[i]
            self._prev_error[i] = self._error[i]
            
            self._cmd[i] = self._KP *self._error[i] + self._KI *self._error[i] + self._KD *self._error[i]
        self.set_in_interval()

        if self._Vc == [0.0,0.0,0.0,0.0]:
            self._pwm_pub.data[0] = 0.0
            self._pwm_pub.data[1] = 0.0
            self._pwm_pub.data[2] = 0.0
            self._pwm_pub.data[3] = 0.0
            self._pwm_base[0] = 0.0
            self._pwm_base[1] = 0.0
            self._pwm_base[2] = 0.0
            self._pwm_base[3] = 0.0
        else:
            self._pwm_pub.data[0] = self._pwm_base[0]
            self._pwm_pub.data[1] = self._pwm_base[3]
            self._pwm_pub.data[2] = self._pwm_base[2]
            self._pwm_pub.data[3] = self._pwm_base[1]

        self.pub.publish(self._pwm_pub)
    
    def run_node(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown() :
            self.velocity_control()
            r.sleep()


if __name__ == "__main__":
    A = PID_Control()
    A.run_node()