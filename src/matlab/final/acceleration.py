#!/usr/bin/env python

import rospy, tf
import geometry_msgs.msg, nav_msgs.msg
from math import *
from time import sleep


def diffOdomCallback(message,poo):
    
    pub,msg,goal = poo
  
    
    wgain = 0.5 # Gain for the angular velocity [rad/s / rad]
    vconst = 0.2# Linear velocity when far away [m/s]
    distThresh = 0.1 # Distance treshold [m]

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
    print distance
    if (distance > distThresh):
        	v = (vconst * distance)
        	desireYaw = atan2(goal[1]-pose[1],goal[0]-pose[0])
		u = desireYaw-theta
        	bound = atan2(sin(u),cos(u))
        	w = min(0.5 , max(-0.5, wgain*bound))
   # Publish
    msg.linear.x = v
    msg.angular.z = w
    pub.publish(msg)
    
    # Reporting
    print('diff_robot: x=%4.1f,y=%4.1f dist=%4.2f, cmd.v=%4.2f, cmd.w=%4.2f'%(pose[0],pose[1],distance,v,w))


# Main Script

rospy.init_node('ControlTurtleBot',anonymous=True)
    
goal = input ('enter required goal  ')
print ('goal:',goal)

# Setup publisher
cmdmsg = geometry_msgs.msg.Twist()
print cmdmsg
cmdpub = rospy.Publisher('/cmd_vel',geometry_msgs.msg.Twist, queue_size=10)


# Setup subscription - which implemets our controller.
# We pass the publisher, the message to publish and the goal as 
# additional parameters to the callback function.
rospy.Subscriber('/odom',nav_msgs.msg.Odometry,diffOdomCallback, 
                 (cmdpub,cmdmsg,goal))


# spin() simply keeps python from exiting until this node is stopped
rospy.spin()

