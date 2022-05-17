#!/usr/bin/env python
# -*- coding:UTF-8 -*-
import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Int32
 
rospy.init_node('topic_publisher')
pub = rospy.Publisher('counter', Float64)
rospy.loginfo("Servopos---->%f\n",servopos)
pub.publish(count)
 
