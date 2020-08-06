#!/usr/bin/env python
import rospy 

def mycallback(event):
    print('hey')

rospy.init_node('timer',anonymous=True)
rospy.Timer(rospy.Duration(0.5),mycallback)
rospy.spin()