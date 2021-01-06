#!/usr/bin/python

import rospy
import roslaunch

uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)


n = input('enter number of robots required:')
for i in range(n):
  
  cli_args1 = ['ros_robotics', 'diff_arg.launch', 'nr:=0']
  roslaunch_file1 = roslaunch.rlutil.resolve_launch_arguments(cli_args1)[0]
  roslaunch_args1 = cli_args1[2:]
 

  if i == 1:
       		cli_args2 = ['ros_robotics', 'diff_arg_1.launch', 'nr:=1','x:=1','y:=1']
       		
  if i == 2:
       		cli_args2 = ['ros_robotics', 'diff_arg_1.launch', 'nr:=2','x:=2','y:=1']
  if i == 3:
       		cli_args2 = ['ros_robotics', 'diff_arg_1.launch', 'nr:=3','x:=3','y:=1']
  if i == 4:
       		cli_args2 = ['ros_robotics', 'diff_arg_1.launch', 'nr:=4','x:=4','y:=1']
  if i == 5:
       		cli_args2 = ['ros_robotics', 'diff_arg_1.launch', 'nr:=5','x:=5','y:=1']
  if i == 6:
       		cli_args2 = ['ros_robotics', 'diff_arg_1.launch', 'nr:=6','x:=6','y:=1']

  if i == 7:
       		cli_args2 = ['ros_robotics', 'diff_arg_1.launch', 'nr:=7','x:=7','y:=1']
  if i == 8:
       		cli_args2 = ['ros_robotics', 'diff_arg_1.launch', 'nr:=8','x:=8','y:=1']
  if i == 9:
       		cli_args2 = ['ros_robotics', 'diff_arg_1.launch', 'nr:=9','x:=9','y:=1']
  if i == 10:
       		cli_args2 = ['ros_robotics', 'diff_arg_1.launch', 'nr:=10','x:=3','y:=2']
  if i == 11:
       		cli_args2 = ['ros_robotics', 'diff_arg_1.launch', 'nr:=11','x:=3','y:=3']
  if i == 12:
       		cli_args2 = ['ros_robotics', 'diff_arg_1.launch', 'nr:=12','x:=3','y:=4']
  if i == 13:
       		cli_args2 = ['ros_robotics', 'diff_arg_1.launch', 'nr:=13','x:=3','y:=5']
  if i == 14:
       		cli_args2 = ['ros_robotics', 'diff_arg_1.launch', 'nr:=14','x:=3','y:=6']
  if i == 15:
       		cli_args2 = ['ros_robotics', 'diff_arg_1.launch', 'nr:=15','x:=3','y:=7']
  if i == 16:
       		cli_args2 = ['ros_robotics', 'diff_arg_1.launch', 'nr:=16','x:=3','y:=8']
  if i == 17:
       		cli_args2 = ['ros_robotics', 'diff_arg_1.launch', 'nr:=17','x:=3','y:=9']
  if i == 18:
       		cli_args2 = ['ros_robotics', 'diff_arg_1.launch', 'nr:=18','x:=3','y:=10']


  if i == 0:
      launch_files = [(roslaunch_file1, roslaunch_args1)]
      parent = roslaunch.parent.ROSLaunchParent(uuid, launch_files)
      parent.start()
      continue; 
  if i >= 1:
      roslaunch_file2 = roslaunch.rlutil.resolve_launch_arguments(cli_args2)[0]
      roslaunch_args2 = cli_args2[2:]
      launch_files = [(roslaunch_file2, roslaunch_args2)]
      parent = roslaunch.parent.ROSLaunchParent(uuid, launch_files)
      parent.start()
      continue; 

rospy.sleep(60000)


