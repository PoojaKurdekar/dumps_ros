#!/usr/bin/env python

import rospy, tf
import geometry_msgs.msg, nav_msgs.msg
from math import *
from time import sleep
import numpy as np
import scipy.integrate as integrate
import scipy
from simple_pid import PID


def diffOdomCallback(message,cpms):
    
    pub,msg,goal = cpms
  
    wgain = 0.8 # Gain for the angular velocity 
    vconst = 0.1# Gain for Linear velocity 
    distThresh = 0.1 # Distance treshold [m]
    K_I = 0.01
    K_P = 0.1
    e1_sum_error = 0
    pos = message.pose.pose
    quat = pos.orientation
  
    # From quaternion to Euler
    angles = tf.transformations.euler_from_quaternion((quat.x,quat.y,quat.z,quat.w))
    theta = angles[2]
    
    pose = [pos.position.x, pos.position.y, theta]  # X, Y, Theta 
    

	# Proportional Controller
    v = 0 # default linear velocity
    w = 0 # default angluar velocity
    distance = sqrt((pose[0]-goal[0])**2+(pose[1]-goal[1])**2)
    
    if (distance > distThresh):
		     	
		v = (K_P * distance) + (e1_sum_error * K_I)
		v = max(min(1,v), 0)
        	desireYaw = atan2(goal[1]-pose[1],goal[0]-pose[0])
		u = desireYaw-theta
        	bound = atan2(sin(u),cos(u))      	
		w = min(0.5 , max(-0.5, wgain*bound))
		print('diff_robot: x=%4.1f,y=%4.1f dist=%4.2f, cmd.v=%4.2f, cmd.w=%4.2f'%(pose[0],pose[1],distance,v,w))
    sleep(0.9)
    e1_sum_error += distance
    print e1_sum_error


    msg.linear.x = v
    msg.angular.z = w
    pub.publish(msg)
    if (w == 0):
 	print ('goal reached= %d , %d '%(goal[0],goal[1]))
    


#################################################################

rospy.init_node('ControlTurtleBot',anonymous=True)
    
goal = input ('enter required goal')
print ('goal = %d , %d '%(goal[0],goal[1]))


cmdmsg = geometry_msgs.msg.Twist()
cmdpub = rospy.Publisher('/cmd_vel',geometry_msgs.msg.Twist, queue_size=10)

rospy.Subscriber('/odom',nav_msgs.msg.Odometry,diffOdomCallback,(cmdpub,cmdmsg,goal))

rospy.spin()

