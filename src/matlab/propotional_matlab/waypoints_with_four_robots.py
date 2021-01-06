#!/usr/bin/env python

import serial
import rospy, tf
import geometry_msgs.msg, nav_msgs.msg
from math import *
from time import sleep
import numpy as np

def diffOdomCallback(message,cpms):
   
    pub,msg,goal = cpms
  
    wgain = 0.8 # kp_angle
    kp_distance = 0.4# Gain for Linear velocity 
    distThresh = 0.0 # Distance treshold [m]
  
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
		v = min((kp_distance*distance),0.5)     	
        	desireYaw = atan2(goal[1]-pose[1],goal[0]-pose[0])
		u = desireYaw-theta
        	bound = atan2(sin(u),cos(u))      	
		w = min(0.5 , max(-0.5, wgain*bound))
		#print('diff_robot: x=%4.1f,y=%4.1f dist=%4.2f, cmd.v=%4.2f, cmd.w=%4.2f'%(pose[0],pose[1],distance,v,w))
    elif (distance == 0) | (distance <= distThresh):
	v = 0.0
       	w = 0,0
   	
 	
    msg.linear.x = v
    msg.angular.z = w
    pub.publish(msg)
    
    
def fourwaypoints(final1):
	goal1 = final1
	goal2 = (goal1[0]+4),(goal1[1]+0)
# Setup publisher
	
	cmdmsg1 = geometry_msgs.msg.Twist()
	cmdpub1 = rospy.Publisher('/cmd_vel_2',geometry_msgs.msg.Twist, queue_size=10)
	cmdmsg2 = geometry_msgs.msg.Twist()
	cmdpub2 = rospy.Publisher('/cmd_vel',geometry_msgs.msg.Twist, queue_size=10)

# Setup subscription
	rospy.Subscriber('/odom_2',nav_msgs.msg.Odometry,diffOdomCallback,(cmdpub1,cmdmsg1,goal1))
	rospy.Subscriber('/odom',nav_msgs.msg.Odometry,diffOdomCallback,(cmdpub2,cmdmsg2,goal2))
	rospy.sleep(10.)
	if goal1 == [0,0]:
		print('final position reached')
		#rospy.signal_shutdown('final position reached-----!!!!!')
	


# Setup subscription
	#rospy.Subscriber('/odom_2',nav_msgs.msg.Odometry,diffOdomCallback,(cmdpub2,cmdmsg2,goal2))
	#rospy.sleep(10.)
	if goal2 == [1,0]:
		print('final position reached')
		
#################################################################

rospy.init_node('ControlTurtleBot',anonymous=True)


goals = [[0, 2], [1,4], [4,4], [4,1], [2,0],[0,0]]
#goal = input ('enter required goal  ')
#print ('goal:', goal)

for i in goals:
	print i
	fourwaypoints(i)
	#rospy.sleep(10.)
	continue

rospy.spin()

