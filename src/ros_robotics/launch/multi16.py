#!/usr/bin/env python
import roslaunch
import rospy

rospy.init_node('multi_robot', anonymous = True)
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)

n = input('enter number of robots required:')
for i in range(n):
  if i == 0: 
		launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/user/catkin_ws/src/ros_robotics/launch/robots/diff_wheeled_gazebo_full.launch"])
		launch.start()
		rospy.loginfo("started")
                continue;
  if i == 1:
                launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/user/catkin_ws/src/ros_robotics/launch/robots/diff_wheeled_gazebo_full_1.launch"])
		launch.start()
		rospy.loginfo("started")
                continue; 
  if i == 2:
                launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/user/catkin_ws/src/ros_robotics/launch/robots/diff_wheeled_gazebo_full_2.launch"])
		launch.start()
		rospy.loginfo("started")
                continue;
  if i == 3:
                launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/user/catkin_ws/src/ros_robotics/launch/robots/diff_wheeled_gazebo_full_3.launch"])
		launch.start()
		rospy.loginfo("started")
                continue;
  if i == 4:
                launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/user/catkin_ws/src/ros_robotics/launch/robots/diff_wheeled_gazebo_full_4.launch"])
		launch.start()
		rospy.loginfo("started")
                continue;
  if i == 5:
                launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/user/catkin_ws/src/ros_robotics/launch/robots/diff_wheeled_gazebo_full_5.launch"])
		launch.start()
		rospy.loginfo("started")
                continue;
   
  if i == 6:
                launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/user/catkin_ws/src/ros_robotics/launch/robots/diff_wheeled_gazebo_full_6.launch"])
		launch.start()
		rospy.loginfo("started")
                continue;
  if i >= 7:
                print("Invalid day of number")
      
    

rospy.sleep(50000000)
# 3 seconds later
launch.shutdown()



 

