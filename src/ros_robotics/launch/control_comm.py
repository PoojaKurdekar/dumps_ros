#!/usr/bin/env python

import serial
import rospy, tf
import geometry_msgs.msg, nav_msgs.msg
from math import *
from time import sleep
import numpy as np

def diffOdomCallback(message,cargs):
   
    pub,msg,goal = cargs
    
    # Tunable parameters
    wgain = 0.8 
    vconst = 0.5
    distThresh = 0.1 

    # Generates a simplified pose
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
        	v = vconst
        	desireYaw = atan2(goal[1]-pose[1],goal[0]-pose[0])
		u = desireYaw-theta
        	bound = atan2(sin(u),cos(u))
        	w = min(0.5 , max(-0.5, wgain*bound))
   		
		print('diff_robot: x=%4.1f,y=%4.1f dist=%4.2f, cmd.v=%4.2f, cmd.w=%4.2f'%(pose[0],pose[1],distance,v,w))
    
    if (distance <= distThresh):
    	print ('Goal reached')
		
    msg.linear.x = v
    msg.angular.z = w
    pub.publish(msg)
    

#################################################################

rospy.init_node('ControlTurtleBot',anonymous=True)

goal = input ('enter required goal  ')
print ('goal:', goal)

# Setup publisher
	
cmdmsg = geometry_msgs.msg.Twist()
cmdpub = rospy.Publisher('/cmd_vel',geometry_msgs.msg.Twist, queue_size=10)
cmdmsg1 = geometry_msgs.msg.Twist()
cmdpub1 = rospy.Publisher('/cmd_vel_1',geometry_msgs.msg.Twist, queue_size=10)
cmdmsg2 = geometry_msgs.msg.Twist()
cmdpub2 = rospy.Publisher('/cmd_vel_2',geometry_msgs.msg.Twist, queue_size=10)
cmdmsg3 = geometry_msgs.msg.Twist()
cmdpub3 = rospy.Publisher('/cmd_vel_3',geometry_msgs.msg.Twist, queue_size=10)
cmdmsg4 = geometry_msgs.msg.Twist()
cmdpub4 = rospy.Publisher('/cmd_vel_4',geometry_msgs.msg.Twist, queue_size=10)
cmdmsg5 = geometry_msgs.msg.Twist()
cmdpub5 = rospy.Publisher('/cmd_vel_5',geometry_msgs.msg.Twist, queue_size=10)
cmdmsg6 = geometry_msgs.msg.Twist()
cmdpub6 = rospy.Publisher('/cmd_vel_6',geometry_msgs.msg.Twist, queue_size=10)



rospy.Subscriber('/odom',nav_msgs.msg.Odometry,diffOdomCallback, 
                 (cmdpub,cmdmsg,goal))

rospy.Subscriber('/odom_1',nav_msgs.msg.Odometry,diffOdomCallback,(cmdpub1,cmdmsg1,goal))

rospy.Subscriber('/odom_2',nav_msgs.msg.Odometry,diffOdomCallback,(cmdpub2,cmdmsg2,goal))

rospy.Subscriber('/odom_3',nav_msgs.msg.Odometry,diffOdomCallback,(cmdpub3,cmdmsg3,goal))

rospy.Subscriber('/odom_4',nav_msgs.msg.Odometry,diffOdomCallback,(cmdpub4,cmdmsg4,goal))

rospy.Subscriber('/odom_5',nav_msgs.msg.Odometry,diffOdomCallback,(cmdpub5,cmdmsg5,goal))

rospy.Subscriber('/odom_6',nav_msgs.msg.Odometry,diffOdomCallback,(cmdpub6,cmdmsg6,goal))


rospy.spin()

