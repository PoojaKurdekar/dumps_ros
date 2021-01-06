#! /usr/bin/env python
import rospy
from std_msgs.msg import Float32
from random import random

rospy.init_node("learning")
pub = rospy.Publisher("reward", Float32, queue_size=1)
rate = rospy.Rate(1)

while not rospy.is_shutdown():
	pub.publish( random() * 50 )
	rate.sleep()
