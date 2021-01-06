#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
import rospy, tf
import geometry_msgs.msg, nav_msgs.msg
from math import *
from time import sleep
import numpy as np


def get_rotation (msg,cpms):
    pub,msgs,goal = cpms
    global roll, pitch, yaw
    pos = msg.pose.pose
    quat = pos.orientation
    #print(quat.x,quat.y,quat.z,quat.w)
    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion((quat.x,quat.y,quat.z,quat.w))
    #print(roll,pitch,yaw)
    pose = quat.x,quat.y,yaw
    velo = controller(pose,goal)
    print velo
    msgs.linear.x = velo[0]
    msgs.angular.z = velo[1]
    pub.publish(msg)
    
def controller(cur,des):
	pose = cur
	goal = des
	wgain = 0.8 # Gain for the angular velocity 
    	vconst = 0.5# Gain for Linear velocity 
    	distThresh = 0.1 # Distance treshold [m]
        theta = 0.5
    	v = 0 # default linear velocity
    	w = 0 # default angluar velocity
    	distance = sqrt((pose[0]-goal[0])**2+(pose[1]-goal[1])**2)
    
    	if (distance > distThresh):
		      	
		v = vconst
        	desireYaw = atan2(goal[1]-pose[1],goal[0]-pose[0])
		u = desireYaw-theta
        	bound = atan2(sin(u),cos(u))      	
		w = min(0.5 , max(-0.5, wgain*bound))
		#print('diff_robot: x=%4.1f,y=%4.1f dist=%4.2f, cmd.v=%4.2f, cmd.w=%4.2f'%(pose[0],pose[1],distance,v,w))

        return v,w



rospy.init_node('my_quaternion_to_euler')

goal = input ('enter required goal  ')
print ('goal:', goal)

cmdmsg = geometry_msgs.msg.Twist()
cmdpub = rospy.Publisher('/cmd_vel',geometry_msgs.msg.Twist, queue_size=10)

sub = rospy.Subscriber ('/odom', nav_msgs.msg.Odometry, get_rotation,(cmdpub,cmdmsg,goal))


r = rospy.Rate(1)

while not rospy.is_shutdown():
    r.sleep()

