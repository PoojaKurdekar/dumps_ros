#!/usr/bin/env python
import serial
import rospy, tf
import geometry_msgs.msg, nav_msgs.msg
from math import *
from time import sleep
import numpy as np

kp_distance = 1
ki_distance = 0.01
kd_distance = 0.5

kp_angle = 1
ki_angle = 0.03
kd_angle = 0.05


def controller(cur,des,err):
    pose = cur
    goal = des

    last_rotation = 0
    linear_speed = 1    #kp_distance
    angular_speed = 1  #kp_angular

    goal_z = atan2(goal[1],goal[0])

    goal_distance = sqrt((pose[0]-goal[0])**2+(pose[1]-goal[1])**2)
    #distance is the error for length, x,y
    distance = goal_distance

    total_distance = 0 
    
    
    total_angle = 0
       

    if (distance > 0.2):
       
	path_angle = atan2(goal[1]-pose[1],goal[0]-pose[0])


        distance = sqrt((pose[0]-goal[0])**2+(pose[1]-goal[1])**2)
	
	control_signal_distance = min((kp_distance*distance + ki_distance*total_distance),0.5) 

        control_signal_angle = kp_angle*path_angle + ki_angle*total_angle 
   	
	last_rotation = pose[2]
	
	#sleep(4)
        
        total_distance = total_distance + distance
        print("Current positin and rotation are: ", (pose))
	
    
    return control_signal_distance, control_signal_angle,total_distance
           
 

def diffOdomCallback(message,cpms):
	pub,msg,goal = cpms
	pos = message.pose.pose
        quat = pos.orientation
   
	angles = tf.transformations.euler_from_quaternion((quat.x,quat.y,quat.z,quat.w))
    	theta = angles[2]
    
    	pose = [pos.position.x, pos.position.y, theta]  # X, Y, Theta 
	prev_err = 0
	desired = controller(pose,goal,prev_err)
	prev_err = desired[2]
	msg.linear.x = desired[0]
    	msg.angular.z = desired[1]
    	pub.publish(msg)

	

##################################################################
rospy.init_node('ControlTurtleBot',anonymous=True)
goal = input ('Enter required goal  ')
print ('goal:', goal)

# Setup publisher
cmdmsg = geometry_msgs.msg.Twist()
cmdpub = rospy.Publisher('/cmd_vel_1',geometry_msgs.msg.Twist, queue_size=10)


# Setup subscription - which implemets our controller.
rospy.Subscriber('/odom_1',nav_msgs.msg.Odometry,diffOdomCallback, 
                 (cmdpub,cmdmsg,goal))

rospy.spin()

