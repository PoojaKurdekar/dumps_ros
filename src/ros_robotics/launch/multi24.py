#!/usr/bin/env python
import roslaunch
import rospy

rospy.init_node('en_Mapping', anonymous=True)
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
arg0 = '/home/user/catkin_ws/src/ros_robotics/launch/diff_wheeled_gazebo_full13.launch'
arg1 = 'mav_name:=robot'

cli_args = [arg0, arg1]
roslaunch_args = cli_args[1:]
roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]

parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)

parent.start()
