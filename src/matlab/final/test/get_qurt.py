
#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry

def get_rotation (msg):
    print msg.pose.pose.orientation

rospy.init_node('my_quaternion_to_euler')

sub = rospy.Subscriber ('/odom', Odometry, get_rotation)

r = rospy.Rate(1)
while not rospy.is_shutdown():
    r.sleep()
