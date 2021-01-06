#!/usr/bin/env python

import rospy, tf
import geometry_msgs.msg, nav_msgs.msg
from math import *
from time import sleep

angles = tf.transformations.euler_from_quaternion((quat.x,quat.y,quat.z,quat.w))

theta = angles[2]
pose = [pos.position.x, pos.position.y, theta]  # X, Y, Theta 
