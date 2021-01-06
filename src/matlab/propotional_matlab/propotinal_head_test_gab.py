def scan_callback(msg):
    global g_range_ahead
    g_range_ahead = min(msg.ranges)
    a = g_range_ahead
    print ('distance to obstacle: ',a)
    # For comparison to Simulink implementation
	
    # Callback arguments 
    pub,msg = cargs
    
    # Tunable parameters
    wgain = 0.5 # Gain for the angular velocity [rad/s / rad]
    
   
    # Generate a simplified pose
    pos = message.pose.pose
    quat = pos.orientation

def diffOdomCallbackscan(message,cargs):
    
    pub,msg,goal = cargs
    
    # Tunable parameters
    wgain = 0.5 
    vconst = 0.5
    distThresh = 0.1 

    # Generates a simplified pose
    pos = message.pose.pose
    quat = pos.orientation
  
    # From quaternion to Euler
    angles = tf.transformations.euler_from_quaternion((quat.x,quat.y,quat.z,quat.w))
    
    theta = angles[2]
    pose = [pos.position.x, pos.position.y, theta]  # X, Y, Theta 
    
    # Proportional Controller
    v = 0 # default linear velocity
    w = 0 # default angluar velocity
    distance = ((sqrt((pose[0]-goal[0])**2+(pose[1]-goal[1])**2)) - 1)
    t_dist = 0
    
    if (distance > distThresh):
        	v = vconst*distance
        	desireYaw = atan2(goal[1]-pose[1],goal[0]-pose[0])
		u = desireYaw-theta
        	bound = atan2(sin(u),cos(u))
        	w = min(0.5 , max(-0.5, wgain*bound))
   		#sleep(1)
		#distance = sqrt((pose[0]-goal[0])**2+(pose[1]-goal[1])**2)
		#t_dist = t_dist + distance
		print('diff_robot: x=%4.1f,y=%4.1f dist=%4.2f, cmd.v=%4.2f, cmd.w=%4.2f'%(pose[0],pose[1],distance,v,w))
    		
    elif (distance == 0) | (distance <= distThresh):
	v = 0.0
       	desireYaw = atan2(head[1]-pose[1],head[0]-pose[0])
	u = desireYaw-theta
       	bound = atan2(sin(u),cos(u))
       	w = min(0.5 , max(-0.5, wgain*bound))
   	print ('Taking head')
    
    if (distance <= distThresh):
    	print ('Goal reached')
 
    # Publish
    msg.linear.x = min(v,0.5)
    msg.angular.z = w
    pub.publish(msg)

	
def diffOdomCallback(message,cargs):
    
    pub,msg,goal = cargs
    
    # Tunable parameters
    wgain = 0.5 
    vconst = 0.5
    distThresh = 0.1 

    # Generates a simplified pose
    pos = message.pose.pose
    quat = pos.orientation
  
    # From quaternion to Euler
    angles = tf.transformations.euler_from_quaternion((quat.x,quat.y,quat.z,quat.w))
    
    theta = angles[2]
    pose = [pos.position.x, pos.position.y, theta]  # X, Y, Theta 
    
    # Proportional Controller
    v = 0 # default linear velocity
    w = 0 # default angluar velocity
    distance = sqrt((pose[0]-goal[0])**2+(pose[1]-goal[1])**2)
    t_dist = 0
    
    if (distance > distThresh):
        	v = vconst*distance
        	desireYaw = atan2(goal[1]-pose[1],goal[0]-pose[0])
		u = desireYaw-theta
        	bound = atan2(sin(u),cos(u))
        	w = min(0.5 , max(-0.5, wgain*bound))
   		#sleep(1)
		#distance = sqrt((pose[0]-goal[0])**2+(pose[1]-goal[1])**2)
		#t_dist = t_dist + distance
		print('diff_robot: x=%4.1f,y=%4.1f dist=%4.2f, cmd.v=%4.2f, cmd.w=%4.2f'%(pose[0],pose[1],distance,v,w))
    		
    elif (distance == 0) | (distance <= distThresh):
	v = 0.0
       	desireYaw = atan2(head[1]-pose[1],head[0]-pose[0])
	u = desireYaw-theta
       	bound = atan2(sin(u),cos(u))
       	w = min(0.5 , max(-0.5, wgain*bound))
   	print ('Taking head')
    
    if (distance <= distThresh):
    	print ('Goal reached')
 
    # Publish
    msg.linear.x = min(v,0.5)
    msg.angular.z = w
    pub.publish(msg)

