     (position, rotation) = self.get_odom()
    

     last_rotation = 0
     linear_speed = 1    #kp_distance
     angular_speed = 1  #kp_angular






    goal_z = atan2(goal[1],goal[0])

    goal_distance = sqrt((pose[0]-goal[0])**2+(pose[1]-goal[1])**2)
    #distance is the error for length, x,y
    distance = goal_distance
    previous_distance = 0
    total_distance = 0 
    
    previous_angle = 0
    total_angle = 0
       

    while (distance > distThresh):
        (position, rotation) = self.get_odom()



	path_angle = atan2(goal[1]-pose[1],goal[0]-pose[0])
	











	diff_angle = path_angle - previous_angle
        diff_distance = distance - previous_distance

        distance = sqrt((pose[0]-goal[0])**2+(pose[1]-goal[1])**2)
	
	control_signal_distance = kp_distance*distance + ki_distance*total_distance + kd_distance*diff_distance

        control_signal_angle = kp_angle*path_angle + ki_angle*total_angle + kd_distance*diff_angle

	msg.angular.z = (control_signal_angle) - pose[2]
	#msg.linear.x = min(linear_speed * distance, 0.1)
	msg.linear.x = min(control_signal_distance, 0.1)
	
	if msg.angular.z > 0:
                msg.angular.z = min(msg.angular.z, 1.5)
        else:
                msg.angular.z = max(msg.angular.z, -1.5)
   	
	last_rotation = pose[2]
	pub.publish(msg)
	r.sleep()
        previous_distance = distance
        total_distance = total_distance + distance
        print("Current positin and rotation are: ", (pose))
    
    (position, rotation) = self.get_odom()
    print("Current positin and rotation are: ", (position, rotation))

    print("reached :)   ^_^")

    while abs(pose[2] - goal_z ) > 0.05:
            (position, rotation) = self.get_odom()
            if goal_z >= 0:
                if pose[2] <= goal_z and pose[2] >= goal_z - pi:
                    msg.linear.x = 0.00
                    msg.angular.z = 0.5
                else:
                    msg.linear.x = 0.00
                    msg.angular.z = -0.5
            else:
                if pose[2] <= goal_z + pi and pose[2] > goal_z:
                    msg.linear.x = 0.00
                    msg.angular.z = -0.5
                else:
                    msg.linear.x = 0.00
                    msg.angular.z = 0.5
            pub.publish(msg)
            r.sleep()

 # rospy.loginfo("Stopping the robot...")
        pub.publish(Twist())
        return
