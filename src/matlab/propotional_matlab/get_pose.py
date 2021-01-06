#!/usr/bin/env python

import rospy, tf
import geometry_msgs.msg, nav_msgs.msg
from math import *
from time import sleep
import numpy as np

def diffOdomCallback(message):
	pos = message.pose.pose
        quat = pos.orientation
   
	angles = tf.transformations.euler_from_quaternion((quat.x,quat.y,quat.z,quat.w))
    	theta = angles[2]
    
    	pose = [pos.position.x, pos.position.y, theta]  # X, Y, Theta 

	print('diff_robot: x=%4.1f,y=%4.1f yaw=%4.2f'%(pose[0],pose[1],pose[2]))


##################################################################
rospy.init_node('ControlTurtleBot',anonymous=True)
    


# Setup subscription - which implemets our controller.
rospy.Subscriber('/odom',nav_msgs.msg.Odometry,diffOdomCallback)


rospy.spin()

