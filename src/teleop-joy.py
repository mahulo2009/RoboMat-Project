#!/usr/bin/env python

import rospy

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D



vel_pub = rospy.Publisher('/car/cmd_vel', Twist,queue_size=10)
sonar_pub = rospy.Publisher('/car/sonar_pose', Pose2D,queue_size=10)

def joy_callback(data):

	print(data.axes[0])
	print(data.buttons[0])

	if (data.buttons[0] == 0) :

		twist = Twist() 

		twist.linear.x =  0.5 * data.axes[1]
  		twist.angular.z = 1.5 * data.axes[0]

		print(twist)
	  	vel_pub.publish(twist)

	else:
		pose = Pose2D()

		pose.x=0;
		pose.y=0;
		pose.theta = 1.7 * data.axes[0]

		print(pose)
		sonar_pub.publish(pose)

		


if __name__ == "__main__":

	print("Hello Teleop Joy")

	rospy.init_node("telop_joy",anonymous=True)

	rospy.Subscriber("/joy",Joy,joy_callback)

 

	rospy.spin()



	
