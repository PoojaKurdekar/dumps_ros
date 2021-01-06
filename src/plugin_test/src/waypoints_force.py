#!/usr/bin/env python

import serial
import rospy, tf
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Wrench ,Twist
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
    
    inc_x = goal[0] -pose[0]
    inc_y = goal[1] -pose[1]
    distance1 = sqrt((inc_x)**2+(inc_y)**2)
    prev_inc_x = 0.0
    delta = 0.1
    force = Wrench()
    speed = Twist()
    r = rospy.Rate(5)
    while not rospy.is_shutdown():
    		inc_x = goal[0] -pose[0]
    		inc_y = goal[1] -pose[1]
    		distance = sqrt((inc_x)**2+(inc_y)**2)
    		derivative = (inc_x - prev_inc_x )/delta
    		angle_to_goal = atan2(inc_y, inc_x)

    		if (distance > (0.7*distance1)):  
        		derivative = (inc_x - prev_inc_x )/delta
        		prev_inc_x = inc_x
        		force.force.x = ((0.2*inc_x)+(0.3*derivative))

    		if (distance < (0.7*distance1)):    
        		derivative = (inc_x - prev_inc_x )/delta
        		prev_inc_x = inc_x
        		force.force.x = ((-0.2*inc_x)+(-0.3*derivative))
			if (distance < 0.3):
    				force.force.x = (-0.05*inc_x)
        		if (distance < 0.2):
				force.force.x = (-0.01*inc_x)
                		if(goal[0] > x):
		  		 	force.force.x = +0.01
        	  		 	break

    print('diff_robot: x=%4.1f,y=%4.1f dist=%4.2f, cmd.v=%4.2f, cmd.w=%4.2f'%(pose[0],pose[1],distance,v,w))

    pub.publish(force)
    
    
def fourwaypoints(final):
	goal = final

# Setup publisher
	
	force = Wrench()
	cmdpub = rospy.Publisher("/topic_name", Wrench, queue_size = 1)
     

# Setup subscription
	rospy.Subscriber('/odom',Odometry,diffOdomCallback,(cmdpub,force,goal))
	rospy.sleep(5.)
	if goal == [0,0]:
		print('final position reached')
		
#################################################################

rospy.init_node('ControlTurtleBot',anonymous=True)
    
goals = [[1, 0], [3,0], [5,0]]
#goal = input ('enter required goal  ')
#print ('goal:', goal)

for i in goals:
	print i
	fourwaypoints(i)
	#rospy.sleep(10.)
	continue

rospy.spin()

