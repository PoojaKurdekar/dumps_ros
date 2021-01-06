#!/usr/bin/env python
import roslaunch
import rospy

rospy.init_node('multi_robot', anonymous = True)
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
n = input('enter number of robots required:')
for i in range(n):
	if i == 0 :
		print i 
		launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/user/catkin_ws/src/ros_robotics/launch/diff_wheeled_gazebo_full.launch"])
		launch.start()
		rospy.loginfo("started")
		continue
	else:
		print i
		launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/user/catkin_ws/src/ros_robotics/launch/diff_wheeled_gazebo_full_1.launch"])
		launch.start()
		rospy.loginfo("started")
		continue
  

rospy.sleep(10000)
# 3 seconds later
launch.shutdown()



 

