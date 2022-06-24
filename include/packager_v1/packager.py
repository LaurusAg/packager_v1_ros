#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from .stage_3 import *

def start():
    rospy.init_node('packager_node', log_level=rospy.DEBUG)
    rate = rospy.Rate(1)
    pub = rospy.Publisher('packager/test', String, queue_size=10)
    stage_3 = Stage_3()
    while not rospy.is_shutdown():
        pub.publish(stage_3.PrintState())
        rate.sleep()
