#! /usr/bin/evn python

import rospy
from geometry_msgs.msg import Twist,Point
from nav_msgs.msg import Odometry
from tf import transformations
import math 

position_ = Point()
yaw_ = 0
state_ = 0

desired_position_ = Point()
desired_position_.x = 0
desired_position_.y = 2
desired_position_.z = 0 #always zero for ground robots

yaw_precision_ = math.pi / 90
dist_precision_ = 0.3
pub = None

def clbk_odom(msg):
  global position_
  global yaw_
  position_ = msg.pose.pose.position
  quaternion = (
     msg.pose.pose.orientation.x,
     msg.pose.pose.orientation.y,
     msg.pose.pose.orientation.z,
     msg.pose.pose.orientation.w)
  euler = transformations.euler_from_quaternion(quaternion)
  yaw_ = euler[2]

def change_state(state):
    global state_
    state_ = state 
    print 'State changed to [%s]' % state_

def fix_yaw(des_pos_,pos_):
  global yaw_, pub, yaw_precision_, state_
  desired_yaw = math.atan2(des_pos_.y - pos_.y,des_pos_.x - pos_.x)
  if ((des_pos_.x < 0) and (des_pos_.y == 0)):
      desired_yaw = abs(math.atan2(des_pos_.y - pos_.y,des_pos_.x - pos_.x))
  elif ((desired_yaw == 0) and (yaw_ == 0)):
      desired_yaw = -3.141592653589793
  else: 
      desired_yaw = math.atan2(des_pos_.y - pos_.y,des_pos_.x - pos_.x)
  print 'Turn: Desired Yaw [%s]' % desired_yaw
  err_yaw = desired_yaw - yaw_
  twist_msg = Twist()
  if math.fabs(err_yaw) > yaw_precision_:
  	if err_yaw > 0:
            twist_msg.angular.z = -0.1
        elif err_yaw < 0:
            twist_msg.angular.z = 0.1
        else :
            twist_msg.angular.z = 0

  pub.publish(twist_msg)

  if math.fabs(err_yaw) <= yaw_precision_:
        print 'Go straight Yaw error: [%s]' % err_yaw
        change_state(1)

def go_straight_ahead(des_pos_,pos_):
  global yaw_, pub, yaw_precision_, state_
  err_pos = math.sqrt(pow(des_pos_.y - pos_.y,2) + pow(des_pos_.x - pos_.x,2))
  if err_pos > dist_precision_:
     twist_msg = Twist()
     twist_msg.linear.x = 0.2
     pub.publish(twist_msg)
  
  elif err_pos < dist_precision_:
     print 'Stop: Position error:[%s]' % err_pos
     change_state(2)

  desired_yaw = math.atan2(des_pos_.y - pos_.y,des_pos_.x - pos_.x)
  err_yaw = desired_yaw - yaw_

  if math.fabs(err_yaw) > yaw_precision_:
     print 'Yaw error:[%s]' % err_yaw
     change_state(0)
  
def done():
  twist_msg = Twist()
  twist_msg.linear.x = 0
  twist_msg.angular.z = 0
  pub.publish(twist_msg)

def main():
  global pub
  rospy.init_node('goto_point')
  pub = rospy.Publisher('/cmd_vel_1', Twist, queue_size = 1)
  sub_odom = rospy.Subscriber('/odom_1', Odometry, clbk_odom)

  rate = rospy.Rate(20)
  while not rospy.is_shutdown():
     if state_ == 0:
     	fix_yaw(desired_position_,position_)
     elif state_ == 1:
     	go_straight_ahead(desired_position_,position_)
     elif state_ == 2:
     	done()
     	pass
     else:
          rospy.logerr('Unknown State')
     rate.sleep()

if __name__ == '__main__':
  main()

