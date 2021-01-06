#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Wrench ,Twist
from math import *

x = 0.0
y = 0.0 
theta = 0.0


def newOdom(msg):
    global x
    global y
    global theta

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

rospy.init_node("force_controller")

sub = rospy.Subscriber("/odom", Odometry, newOdom)
pub = rospy.Publisher("/topic_name", Wrench, queue_size = 1)

force = Wrench()
speed = Twist()
r = rospy.Rate(5)

#goal = Point()
goal = input("enter goal")
#goal[0] = 1
#goal[1] = 0
inc_x1 = goal[0] -x
inc_y1= goal[1] -y
distance1 = sqrt((inc_x1)**2+(inc_y1)**2)
#print ('distance1  :  ',distance1)
print distance1
prev_inc_x = 0.0
prev_inc_y = 0.0
delta = 0.01
while not rospy.is_shutdown():
    inc_x = goal[0] -x
    inc_y = goal[1] -y
    distance = sqrt((inc_x)**2+(inc_y)**2)
    #print ('distance  :  ',distance)
    angle_to_goal = atan2(inc_y, inc_x)

    if (distance > (0.7*distance1)):  
        	derivative_x = (inc_x - prev_inc_x )/delta
        	prev_inc_x = inc_x
        	force.force.x = ((0.2*inc_x)+(0.3*derivative_x))
        	derivative_y = (inc_y - prev_inc_y )/delta
        	prev_inc_y = inc_y
        	force.force.y = ((0.2*inc_y)+(0.3*derivative_y))

    if (distance < (0.7*distance1)):    
        	derivative_x = (inc_x - prev_inc_x )/delta
        	prev_inc_x = inc_x
        	force.force.x = ((-0.3*inc_x)+(-0.4*derivative_x))
        	derivative_y = (inc_y - prev_inc_y )/delta
        	prev_inc_y = inc_y
        	force.force.y = ((-0.3*inc_y)+(-0.4*derivative_y))

		if (distance < 0.3):
    			force.force.x = (-0.05*inc_x)
			force.force.y = (-0.05*inc_y)
        	if (distance < 0.2):
			force.force.x = (-0.01*inc_x)
			force.force.y = (-0.01*inc_y)
                	if(goal[0] > x):
		  		force.force.x = +0.01
        	  		break
        #if ((round(speed.linear.x,3)) == 0.000) & (inc_x < 0.01):
         #force.force.x = 0
    #else:
       #force.force.x = (-0.2*inc_x)
       #force.torque.z = 0.0
       #r.sleep()
       #break
       #
        #r.sleep()
        #break
       
    		
    print('forceapply: x=%4.2f,y=%4.2f dist=%4.2f, cmd.vx=%4.3f,cmd.vy=%4.3f,cmd.linear.x=%4.2f\n'%(x,y,distance,force.force.x,force.force.y,speed.linear.x))
    pub.publish(force)
    r.sleep()    

