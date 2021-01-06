#!/usr/bin/env python
import rospy, tf
import geometry_msgs.msg, nav_msgs.msg
from math import *
def forceapply(message,cargs):
    
    pub,msg,goal = cargs
    
    force = 1.0 # force when far away [N]
    distThresh = 0.1 # Distance treshold [m]
    # Generate a simplified pose
    pos = message.pose.pose
    quat = pos.orientation
    # From quaternion to Euler
    angles = tf.transformations.euler_from_quaternion((quat.x,quat.y,quat.z,quat.w))
    theta = angles[2]
    pose = [pos.position.x, pos.position.y, theta]  # X, Y, Theta
     
    # Proportional Controller
    
    vx = 0
    vy = 0
    distance = sqrt((pose[0]-goal[0])**2+(pose[1]-goal[1])**2)
    x = abs(pose[0]-goal[0])
    y = abs(pose[1]-goal[1])
    if (distance > 1.5):
        vx = min(0.5,max(-0.5,(force*x)))
        vy = min(0.5,max(-0.5,(force*y)))
        #if(vx == 0):
           #vx=0
    if (distance < 1.5):
        vx = min(-0.5,max(-0.5,(force*x)))
        vy = min(-0.5,max(-0.5,(force*y)))
	#if (vx == 0)
        #rospy.sleep(2)
        #vx = 0
        #vy = 0

    # Publish
    msg.force.x = vx
    msg.force.y = vy
    
    pub.publish(msg)
     
    # Reporting
    print('forceapply: x=%4.1f,y=%4.1f dist=%4.2f, cmd.vx=%4.2f,cmd.vy=%4.2f\n'%(pose[0],pose[1],distance,vx,vy))
########################################
# Main Script
# Initialize our node
rospy.init_node('floating_control',anonymous=True)
     
# Set waypoint for Husky to drive to
goal = input("enter goal")  # Goal position in x/y
# Setup publisher
cmdmsg = geometry_msgs.msg.Wrench()
cmdpub = rospy.Publisher('/topic_name',geometry_msgs.msg.Wrench, queue_size=10)
# Setup subscription - which implemets our controller.
# We pass the publisher, the message to publish and the goal as
# additional parameters to the callback function.
rospy.Subscriber('/odom',nav_msgs.msg.Odometry,forceapply,(cmdpub,cmdmsg,goal))
# spin() simply keeps python from exiting until this node is stopped
rospy.spin()
