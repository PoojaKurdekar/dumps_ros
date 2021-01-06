#!/usr/bin/env python
import rospy, tf
import geometry_msgs.msg, nav_msgs.msg
from math import *
from time import sleep
from sensor_msgs.msg import LaserScan

def scan_callback(msg):
    global g_range_ahead
    g_range_ahead = min(msg.ranges)
    a = g_range_ahead
    print ('distance to obstacle: ',a) 


def diffOdomCallback(message,cargs):
    # Implementation of proportional position control 
    # For comparison to Simulink implementation
	
    # Callback arguments 
    pub,msg = cargs
    
    # Tunable parameters
    w = 0.5 # Gain for the angular velocity [rad/s / rad]
    
    v = 1 # Linear velocity when far away [m/s]
    
    # Generate a simplified pose
    pos = message.pose.pose
    quat = pos.orientation
    
    if (g_range_ahead >= 0.8):
	msg.linear.x = v
    	msg.angular.z = w
    	pub.publish(msg)
    else:
	msg.linear.x = 0.0
    	msg.angular.z = 0.2
    	pub.publish(msg)
    
    # Reporting
    #print('diff_robot: x=%4.1f,y=%4.1f dist=%4.2f, cmd.v=%4.2f, cmd.w=%4.2f'%(pos[0],pos[1],distance,v,w))


rospy.init_node('ControlTurtleBot',anonymous=True)
    
# Setup publisher
cmdmsg = geometry_msgs.msg.Twist()
cmdpub = rospy.Publisher('/cmd_vel',geometry_msgs.msg.Twist, queue_size=10)


# Setup subscription - which implemets our controller.
# We pass the publisher, the message to publish and the goal as 
# additional parameters to the callback function.
scan_sub = rospy.Subscriber('/scan', LaserScan, scan_callback)
rospy.Subscriber('/odom',nav_msgs.msg.Odometry,diffOdomCallback,(cmdpub,cmdmsg))

# spin() simply keeps python from exiting until this node is stopped
rospy.spin()
