#!/usr/bin/env python

import rospy, tf
import geometry_msgs.msg, nav_msgs.msg
from math import *
from time import sleep
from sensor_msgs.msg import LaserScan


def controller(cur,des):
	posi = cur
	goali = des
	#print posi
	#print goali
	wgain = 0.8 # Gain for the angular velocity 
    	vconst = 0.2# Gain for Linear velocity 
    	distThresh = 0.5 # Distance treshold [m]
    	v = 0 
    	w = 0 
    	ki_distance = 0.01
	total_distance = 0
    	distance = sqrt((posi[0]-goali[0])**2+(posi[1]-goali[1])**2)
    
    	if (distance > distThresh):
		   	
		v = vconst*distance 
        	desireYaw = atan2(goali[1]-posi[1],goali[0]-posi[0])
		u = desireYaw-posi[2]
        	bound = atan2(sin(u),cos(u))      	
		w = min(0.5 , max(-0.5, wgain*bound))
		print('diff_robot: x=%4.1f,y=%4.1f dist=%4.2f, cmd.v=%4.2f, cmd.w=%4.2f'%(posi[0],posi[1],distance,v,w))

	if (distance <= distThresh):
    	     print ('Goal reached')
		
	
	return v,w




def diffOdomCallback(message,cpms):
	pub,msg,goal = cpms
	pos = message.pose.pose
        quat = pos.orientation
   
	angles = tf.transformations.euler_from_quaternion((quat.x,quat.y,quat.z,quat.w))
    	theta = angles[2]
    
    	pose = [pos.position.x, pos.position.y, theta]  # X, Y, Theta 

	desired = controller(pose,goal)
	
	msg.linear.x = min(desired[0],0.5)
    	msg.angular.z = desired[1]
    	pub.publish(msg)

	

def controllerscan(cur,des):
	posi = cur
	goali = des
	#print posi
	#print goali
	wgain = 0.8 # Gain for the angular velocity 
    	vconst = 0.5# Gain for Linear velocity 
    	distThresh = 0.5 # Distance treshold [m]
    	v = 0 
    	w = 0 
    	ki_distance = 0.01
	total_distance = 0
    	distance = sqrt((posi[0]-goali[0])**2+(posi[1]-goali[1])**2)
    
    	if (distance > distThresh):
		   	
		v = vconst*distance 
        	desireYaw = atan2(goali[1]-posi[1],goali[0]-posi[0])
		u = desireYaw-posi[2]
        	bound = atan2(sin(u),cos(u))      	
		w = min(0.5 , max(-0.5, wgain*bound))
		print('diff_robot: x=%4.1f,y=%4.1f dist=%4.2f, cmd.v=%4.2f, cmd.w=%4.2f'%(posi[0],posi[1],distance,v,w))

	if (distance <= distThresh):
    	     print ('Goal reached')
		
	
	return v,w


def diffOdomCallbackscan(message,cpms):
	pub,msg,goal = cpms
	
	
	pos = message.pose.pose
        quat = pos.orientation
   	
	angles = tf.transformations.euler_from_quaternion((quat.x,quat.y,quat.z,quat.w))
    	theta = angles[2]
    
    	pose = [pos.position.x, pos.position.y, theta]  # X, Y, Theta 
	
	desired = controllerscan(pose,goal)
	
	msg.linear.x = min(desired[0],0.5)
    	msg.angular.z = desired[1]
    	pub.publish(msg)



########################################
# Main Script

rospy.init_node('ControlTurtleBot',anonymous=True)
    

goal = input ('enter required goal  ')
print ('goal:  ',goal)

head = [goal[0]+1,goal[1]]
print ('head:  ',head)

# Setup publisher
cmdmsg = geometry_msgs.msg.Twist()
cmdpub = rospy.Publisher('/cmd_vel',geometry_msgs.msg.Twist, queue_size=10)

rospy.Subscriber('/odom_2',nav_msgs.msg.Odometry,diffOdomCallback, 
                 (cmdpub,cmdmsg,goal))

#rospy.init_node('ControlTurtleBotscan',anonymous=True)

# Setup publisher
cmdmsg = geometry_msgs.msg.Twist()
cmdpub = rospy.Publisher('/cmd_vel',geometry_msgs.msg.Twist, queue_size=10)


# Setup subscription - which implemets our controller.
# We pass the publisher, the message to publish and the goal as 
# additional parameters to the callback function.
#scan_sub = rospy.Subscriber('/scan_1', LaserScan, scan_callback)
rospy.Subscriber('/odom',nav_msgs.msg.Odometry,diffOdomCallbackscan,(cmdpub,cmdmsg,goal))

rospy.spin()

