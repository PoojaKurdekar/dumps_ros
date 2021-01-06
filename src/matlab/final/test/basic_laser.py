#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import sensor_msgs.msg
import random
import numpy as np
from geometry_msgs.msg import Twist
from itertools import *
from operator import itemgetter

LINX = 0.0 #Always forward linear velocity.
THRESHOLD = 1.5 #THRESHOLD value for laser scan.
PI = 3.14
Kp = 0.05
angz = 0

def LaserScanProcess(data):
	range_angels = min(data.ranges)
	print(range_angels)

rospy.init_node('listener', anonymous=True)

pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
rospy.Subscriber("/scan", sensor_msgs.msg.LaserScan , LaserScanProcess)
rate = rospy.Rate(10) # 10hz

while not rospy.is_shutdown():
        command = Twist()
        command.linear.x = LINX
        command.angular.z = angz
        pub.publish(command)
        rate.sleep()
