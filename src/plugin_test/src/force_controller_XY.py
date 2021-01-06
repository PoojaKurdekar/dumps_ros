#!/usr/bin/env python
import rospy, tf
import geometry_msgs.msg, nav_msgs.msg
from math import *
def floatingbox(message,cargs):
    # Implementation of proportional position control
    # For comparison to Simulink implementation
    # Callback arguments
    pub,msg,goal = cargs
    # Tunable parameters

    wgain = 40.0 # Gain for the angular velocity [rad/s / rad]
    vconst = 70.0 # Linear velocity when far away [m/s]
    distThresh = 0.5 # Distance treshold [m]
    # Generate a simplified pose
    pos = message.pose.pose
    quat = pos.orientation
    # From quaternion to Euler
    angles = tf.transformations.euler_from_quaternion((quat.x,quat.y,
                                                       quat.z,quat.w))
    theta = angles[2]
    pose = [pos.position.x, pos.position.y, theta]  # X, Y, Theta
     
    # Proportional Controller
    vx = 0
    vy = 0
    v = 0 # default linear velocity
    w = 0 # default angluar velocity
    distancex = abs(goal[0]-pose[0])
    distancey = abs(goal[1]-pose[1])
    if (distancex > 0):
        vx = (vconst*distancex)
	if (distancey > 0):
           vy = (vconst*distancey)
        desireYaw = atan2(goal[1]-pose[1],goal[0]-pose[0])
        u = desireYaw-theta
        bound = atan2(sin(u),cos(u))
        print bound
        w = min(50 , max(-50, wgain*bound))
        rospy.sleep(1.0)
    if (distancex < 0):
        vx = (-vconst*distancex)
	if (distancey < 0):
           vy = (-vconst*distancey)
    # Publish
    msg.force.x = min(vx ,70)
    msg.force.y = min(vy ,70)
    msg.torque.z = w
    pub.publish(msg)
     
    # Reporting
    print('huskyOdomCallback: x=%4.1f,y=%4.1f distx=%4.2f, disty=%4.2f, cmd.vx=%4.2f,cmd.vy=%4.2f, cmd.w=%4.2f\n'%(pose[0],pose[1],distancex,distancey,vx,vy,w))
########################################
# Main Script
# Initialize our node
rospy.init_node('floatingbox_control',anonymous=True)
     
# Set waypoint for Husky to drive to
goal = input("enter goal")  # Goal position in x/y
# Setup publisher
cmdmsg = geometry_msgs.msg.Wrench()
cmdpub = rospy.Publisher('/topic_name',geometry_msgs.msg.Wrench, queue_size=1)
# Setup subscription - which implemets our controller.
# We pass the publisher, the message to publish and the goal as
# additional parameters to the callback function.
rospy.Subscriber('/odom',nav_msgs.msg.Odometry,floatingbox,
                 (cmdpub,cmdmsg,goal))
# spin() simply keeps python from exiting until this node is stopped
rospy.spin()
