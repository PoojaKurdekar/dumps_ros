#!/usr/bin/env python
import sys
import rospy
import math
import tf
import geometry_msgs.msg

if __name__ == '__main__':
    rospy.init_node('tf_listener_turtle')

    listener = tf.TransformListener()

    if len(sys.argv) < 3:
        print("usage: turtle_tf_listener.py follower_model_name model_to_be_followed_name")
    else:
        follower_model_name = sys.argv[1]
        model_to_be_followed_name = sys.argv[2]

        turtle_vel = rospy.Publisher('/topic_name', geometry_msgs.msg.Wrench,queue_size=1)

        rate = rospy.Rate(10.0)
        ctrl_c = False

        follower_model_frame = follower_model_name
        model_to_be_followed_frame = model_to_be_followed_name

        def shutdownhook():
            # works better than the rospy.is_shut_down()
            global ctrl_c
            print "shutdown time! Stop the robot"
            cmd = geometry_msgs.msg.Wrench()
            cmd.force.x = 0.0
            cmd.torque.z = 0.0
            turtle_vel.publish(cmd)
            ctrl_c = True

        rospy.on_shutdown(shutdownhook)

        while not ctrl_c:
            try:
                (trans,rot) = listener.lookupTransform(follower_model_frame, model_to_be_followed_frame, rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            print trans
	    dist = (math.sqrt(trans[0] ** 2 + trans[1] ** 2))
	    print dist
	    print('distance between robots 11 and 22: x=%2.1f'%(dist))
	    if (dist > 1):
            	#torque = math.atan2(trans[1], trans[0])
		force = min(math.sqrt(trans[0] ** 2 + trans[1] ** 2)*0.8,0.5)
            	#force = min(math.sqrt(trans[0] ** 2 + trans[1] ** 2)*1,0.8)
		#force = math.sqrt(trans[0] ** 2 + trans[1] ** 2)*2
	   
	    else:
		#torque = 0.0
            	force = 0.0

            cmd = geometry_msgs.msg.Wrench()
            cmd.force.x = force
            #cmd.torque.z = torque
            turtle_vel.publish(cmd)

            rate.sleep()
