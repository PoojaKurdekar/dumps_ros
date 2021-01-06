#!/usr/bin/env python
import serial
import rospy, tf
import geometry_msgs.msg, nav_msgs.msg
from math import *
from time import sleep
import numpy as np
global distance_sum
distance_sum = 0.0
def controller(cur,des,e1_sum_error):
	posi = cur
	goali = des
	#print posi
	#print goali
	wgain = 0.8 # Gain for the angular velocity 
    	K_P = 0.5# Gain for Linear velocity 
    	distThresh = 0.1 # Distance treshold [m]
	K_I = 0.01
	K_D = 0.1
    	v = 0 
    	w = 0 
        e1_sum_error = 0
    	distance = sqrt((posi[0]-goali[0])**2+(posi[1]-goali[1])**2)
        
    	if (distance > distThresh):
		
		
		v = (K_P * distance)
		v = max(min(1,v), 0)
    		
        	desireYaw = atan2(goali[1]-posi[1],goali[0]-posi[0])
		u = desireYaw-posi[2]
        	bound = atan2(sin(u),cos(u))      	
		w = min(0.5 , max(-0.5, wgain*bound))
		print('diff_robot: x=%4.1f,y=%4.1f dist=%4.2f, cmd.v=%4.2f, cmd.w=%4.2f'%(posi[0],posi[1],distance,v,w))

	
	return v,w,distance
	
	#print distance

def diffOdomCallback(message,cpms):
	pub,msg,goal = cpms
	pos = message.pose.pose
        quat = pos.orientation
   
	angles = tf.transformations.euler_from_quaternion((quat.x,quat.y,quat.z,quat.w))
    	theta = angles[2]
    
    	pose = [pos.position.x, pos.position.y, theta]  # X, Y, Theta 
	
	desired = controller(pose,goal,e1_sum_error)
	#s = error(desired[2])
	#print s
	#print ('distance_sum   ',distance_sum)
	#print ('distance_prev   ',distance_prev)
	msg.linear.x = desired[0]
    	msg.angular.z = desired[1]
    	pub.publish(msg)
	p = desired[2]
	p += p
	print p
	sleep(0.9)
	e1_sum_error += desired[2]
	print e1_sum_error


	 
##################################################################
rospy.init_node('ControlTurtleBot',anonymous=True)

goals = [(1,3),(2,4)]

for goal in goals:
	print ('goal:', goal)
	cmdmsg = geometry_msgs.msg.Twist()
	cmdpub = rospy.Publisher('/cmd_vel',geometry_msgs.msg.Twist, 	queue_size=10)


# Setup subscription - which implemets our controller.
	rospy.Subscriber('/odom',nav_msgs.msg.Odometry,diffOdomCallback, 
                 (cmdpub,cmdmsg,goal))

rospy.spin()

