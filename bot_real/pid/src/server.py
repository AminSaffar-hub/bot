#!/usr/bin/env python

import rospy

from dynamic_reconfigure.server import Server
from dynamic_tutorials.cfg import TutorialsConfig

def callback(config, level):
    rospy.loginfo("""Reconfigure Request: {k_p}, {k_i}, 
          {k_d}""".format(**config))
    return config

if __name__ == "__main__":
    rospy.init_node("pid", anonymous = False)

    srv = Server(TutorialsConfig, callback)
    rospy.spin()
