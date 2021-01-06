# Proportional Controller
    wgain = 0.8 # Gain for the angular velocity 
    vconst = 0.5# Gain for Linear velocity 
    distThresh = 0.1 # Distance treshold [m]
    v = 0 
    w = 0 
    
    distance = sqrt((pose[0]-goal[0])**2+(pose[1]-goal[1])**2)
    
    if (distance > distThresh):
		   	
		v = vconst
        	desireYaw = atan2(goal[1]-pose[1],goal[0]-pose[0])
		u = desireYaw-theta
        	bound = atan2(sin(u),cos(u))      	
		w = min(0.5 , max(-0.5, wgain*bound))
		print('diff_robot: x=%4.1f,y=%4.1f dist=%4.2f, cmd.v=%4.2f, cmd.w=%4.2f'%(pose[0],pose[1],distance,v,w))
		

