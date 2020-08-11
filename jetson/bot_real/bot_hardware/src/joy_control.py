#!/usr/bin/env python
import rospy 
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Joy 

cmd = Float32MultiArray()
cmd.data.append(0)
cmd.data.append(0)
cmd.data.append(0)
cmd.data.append(0)

def joy_callback(data):
    global pub,cmd
    cmd_x_Axes = data.axes[1] * 255.0
    cmd_y_Axes = data.axes[0] * 255.0
    fr = cmd_x_Axes - cmd_y_Axes
    fl = cmd_x_Axes + cmd_y_Axes

    if fr > 255.0 :
        fr = 255.0
    elif fr < -255.0 :
        fr = -255.0

    if fl > 255.0 :
        fl = 255.0
    elif fl < -255.0 :
        fl = -255.0

    #print("fr = %f, fl = %f"%(fr,fl))
    cmd.data[0] = fl
    cmd.data[1] = fr
    cmd.data[2] = fr
    cmd.data[3] = fl
    pub.publish(cmd)

rospy.init_node('joy_control',anonymous=True)
rospy.Subscriber('/joy',Joy,joy_callback,queue_size=10)
pub = rospy.Publisher('/vel_cmd',Float32MultiArray,queue_size=10)
rospy.spin()