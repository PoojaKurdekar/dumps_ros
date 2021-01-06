#!/usr/bin/python

import rospy
import roslaunch

uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)

cli_args1 = ['ros_robotics', 'diff_arg.launch', 'nr:=3']
cli_args2 = ['ros_robotics', 'diff_arg_1.launch', 'nr:=2']
roslaunch_file1 = roslaunch.rlutil.resolve_launch_arguments(cli_args1)[0]
roslaunch_args1 = cli_args1[2:]

roslaunch_file2 = roslaunch.rlutil.resolve_launch_arguments(cli_args2)[0]
roslaunch_args2 = cli_args2[2:]

launch_files = [(roslaunch_file1, roslaunch_args1), (roslaunch_file2, roslaunch_args2)]

parent = roslaunch.parent.ROSLaunchParent(uuid, launch_files)

parent.start()
rospy.sleep(60000)
