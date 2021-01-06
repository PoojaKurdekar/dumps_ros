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
    
    if (a >= 0.5):
	

def diffOdomCallback(message,cargs):
    print a
    # Implementation of proportional position control 
    # For comparison to Simulink implementation
	
    # Callback arguments 
    pub,msg = cargs
    
    # Tunable parameters
    wgain = 0.5 # Gain for the angular velocity [rad/s / rad]
    
   
    # Generate a simplified pose
    pos = message.pose.pose
    quat = pos.orientation
    angles = tf.transformations.euler_from_quaternion((quat.x,quat.y,quat.z,quat.w))
    
    theta = angles[2]
    pose = [pos.position.x, pos.position.y, theta]  # X, Y, Theta 
    
    vconst = 0.4 # Linear velocity when far away [m/s]
    distance = sqrt((pose[0]-goal[0])**2+(pose[1]-goal[1])**2)
    t_dist = 0
    if (distance >= 0.8):
	v = vconst*distance+0.01*t_dist
        desireYaw = atan2(goal[1]-pose[1],goal[0]-pose[0])
	u = desireYaw-theta
        bound = atan2(sin(u),cos(u))
        w = min(0.5 , max(-0.5, wgain*bound))
   
	
	print('diff_robot: x=%4.1f,y=%4.1f dist=%4.2f, cmd.v=%4.2f, cmd.w=%4.2f'%(pose[0],pose[1],distance,v,w))
	msg.linear.x = v
    	msg.angular.z = w
    	pub.publish(msg)
   
    
    # Reporting
    #print('diff_robot: x=%4.1f,y=%4.1f dist=%4.2f, cmd.v=%4.2f, cmd.w=%4.2f'%(pos[0],pos[1],distance,v,w))


rospy.init_node('ControlTurtleBotscan',anonymous=True)
goal = input ('enter required goal  ')
print ('goal:  ',goal)
    
# Setup publisher
cmdmsg = geometry_msgs.msg.Twist()
cmdpub = rospy.Publisher('/cmd_vel_2',geometry_msgs.msg.Twist, queue_size=10)


# Setup subscription - which implemets our controller.
# We pass the publisher, the message to publish and the goal as 
# additional parameters to the callback function.
scan_sub = rospy.Subscriber('/scan_2', LaserScan, scan_callback)
rospy.Subscriber('/odom_2',nav_msgs.msg.Odometry,diffOdomCallback,(cmdpub,cmdmsg))

# spin() simply keeps python from exiting until this node is stopped
rospy.spin()
