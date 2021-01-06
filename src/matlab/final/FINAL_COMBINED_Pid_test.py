#!/usr/bin/env python

import rospy, tf
import geometry_msgs.msg, nav_msgs.msg
from math import *
from time import sleep
import numpy as np
import scipy.integrate as integrate
import scipy
import rospy
from geometry_msgs.msg import Twist, Point, Quaternion
import tf
from math import radians, copysign, sqrt, pow, pi, atan2
from tf.transformations import euler_from_quaternion
import numpy as np
import subprocess
import os
import time
import math 
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from tf.transformations import quaternion_from_euler

kp_distance = 1
ki_distance = 0.01
kd_distance = 0.5

kp_angle = 1
ki_angle = 0.03
kd_angle = 0.05


def diffOdomCallback(message,cpms):
    
    pub,msg,goal = cpms
  
    wgain = 0.8 # Gain for the angular velocity 
    vconst = 0.1# Gain for Linear velocity 
    distThresh = 0.1 # Distance treshold [m]

    pos = message.pose.pose
    quat = pos.orientation
  
    # From quaternion to Euler
    angles = tf.transformations.euler_from_quaternion((quat.x,quat.y,quat.z,quat.w))
    theta = angles[2]
    
    pose = [pos.position.x, pos.position.y, theta]  # X, Y, Theta 
    

	# Proportional Controller

    last_rotation = 0
    linear_speed = 1    #kp_distance
    angular_speed = 1  #kp_angular

    goal_distance = sqrt((pose[0]-goal[0])**2+(pose[1]-goal[1])**2)
    #distance is the error for length, x,y
    distance = goal_distance
    previous_distance = 0
    total_distance = 0 
    last_rotation = 0
    previous_angle = 0
    total_angle = 0
       
    while (distance > distThresh):
        path_angle = atan2(goal[1]-pose[1],goal[0]-pose[0])
	u = path_angle-pose[2]
        bound = atan2(sin(u),cos(u))      	
	w = min(0.5 , max(-0.5, wgain*bound))

	diff_angle = path_angle - previous_angle
        diff_distance = distance - previous_distance

	control_signal_distance = kp_distance*distance + ki_distance*total_distance + kd_distance*diff_distance

        control_signal_angle = kp_angle*path_angle + ki_angle*total_angle + kd_distance*diff_angle

    	
	last_rotation = pose[2]
	r.sleep()
        previous_distance = distance
        total_distance = total_distance + distance
        print("Current positin and rotation are: ", (pose))
   
    msg.angular.z = (control_signal_angle) - pose[2])
    msg.linear.x = min(control_signal_distance, 0.5)
    pub.publish(msg)
  

#################################################################

rospy.init_node('ControlTurtleBot',anonymous=True)
    
goal = input ('enter required goal')
print ('goal = %d , %d '%(goal[0],goal[1]))


cmdmsg = geometry_msgs.msg.Twist()
cmdpub = rospy.Publisher('/cmd_vel',geometry_msgs.msg.Twist, queue_size=10)

rospy.Subscriber('/odom',nav_msgs.msg.Odometry,diffOdomCallback,(cmdpub,cmdmsg,goal))


r = rospy.Rate(10)

while not rospy.is_shutdown():
    r.sleep()


