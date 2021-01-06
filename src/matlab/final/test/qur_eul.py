#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

def get_rotation (msg):
    global roll, pitch, yaw
    pos = msg.pose.pose
    quat = pos.orientation
    #print(quat.x,quat.y,quat.z,quat.w)
    (roll, pitch, yaw) = quaternion_from_euler(quat.x,quat.y,quat.z,quat.w)
    #print(roll,pitch,yaw)
    print(quat.x,quat.y,yaw)
    

rospy.init_node('my_quaternion_to_euler')

sub = rospy.Subscriber ('/odom', Odometry, get_rotation)
print sub

r = rospy.Rate(1)

while not rospy.is_shutdown():
    r.sleep()

