#!/usr/bin/python

import rospy
import roslaunch

uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)

cli_args = ['/home/user/catkin_ws/src/ros_robotics/launch/diff_arg_1.launch','nr:=1']
roslaunch_args = cli_args[1:]  # empty but doesn't affect the outcome
roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(cli_args)
parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
parent.start()
rospy.sleep(60000)  # prevent from immediately shutting down

import roslaunch

uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)

cli_args = ['ros_robotics', 'diff_arg_1.launch', 'arg1:=arg1', 'arg2:=arg2']
roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(cli_args)
roslaunch_args = cli_args[2:]
parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file, roslaunch_args=[roslaunch_args])

parent.start()
