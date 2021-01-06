import rospy
from geometry_msgs.msg import Twist

class CircleMode():
    def __init__(self):
        # initiliaze
        rospy.init_node('CircleMode', anonymous=False)

	# tell user how to stop Myrobot
	rospy.loginfo("To stop Myrobot CTRL + C")

        # What function to call when you ctrl + c    
        #rospy.on_shutdown(self.shutdown)
        
	# Create a publisher which can "talk" to Myrobot and tell it to move
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
     
	#Myrobot will stop if we don't keep telling it to move.  How often should we tell it to move? 10 HZ
        r = rospy.Rate(5);

        # Twist is a datatype for velocity
        move_cmd = Twist()
	# let's go forward at 0.2 m/s
    

        for i in range(50):
		if (i >= 0):
		   move_cmd.linear.x = min((0.1 + i),0.6)
	# let's turn at 0 radians/s
	           move_cmd.angular.z = 0.0

	# let's turn at 0 radians/s
        #move_cmd.linear.x = 0.1

	# as long as you haven't ctrl + c keep going...
        

        while not rospy.is_shutdown():
	    # publish the velocity
            self.cmd_vel.publish(move_cmd)
	    # wait for 0.1 seconds (10 HZ) and publish again
            r.sleep()
                        
        
def shutdown(self):
        # stop Myrobot
        rospy.loginfo("Stop Myrobot")
	# a default Twist has linear.x of 0 and angular.z of 0.  So it'll stop Myrobot
        self.cmd_vel.publish(Twist())
	# sleep just makes sure Myrobot receives the stop command prior to shutting down the script
        rospy.sleep(1)
 
if __name__ == '__main__':
    try:
        CircleMode()
    except:
        rospy.loginfo("CircleMode node terminated.")

