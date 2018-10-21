#!/usr/bin/env python

import rospy

from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist

pub = rospy.Publisher('/car/cmd_vel', Twist,queue_size=10)

def odom_callback(data):
	print(data.range)

	twist = Twist()
	twist.linear.x=0.0
	twist.linear.y=0.0
	twist.linear.z=0.0

	twist.angular.x=0.0
	twist.angular.y=0.0
	twist.angular.z=0.0

	if data.range > 0.5:
		twist.linear.x=0.25
	else:
		twist.linear.x=0.0
		twist.angular.z=1.0


	print(twist)

	pub.publish(twist)


if __name__ == '__main__':

	rospy.init_node("robot_mat_navigate",anonymous=True)

	rospy.Subscriber('/car/ultrasound', Range, odom_callback)
	#rospy.Subscriber('/car/ultrasound', Vector3, callback)

	
	print("Hola!!!")

	rospy.spin()
