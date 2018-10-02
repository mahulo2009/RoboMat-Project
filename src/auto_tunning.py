#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Vector3

def callback(data):
	
	target = data.x
	current = data.y
	demanded = data.z


	rospy.loginfo(target)
	rospy.loginfo(current)
	rospy.loginfo(demanded)

def listener():
	pub = rospy.Publisher('/car/pid_params', Vector3, queue_size=10)
	rospy.init_node("robot_mat_auto_tunning",anonymous=True)
	#rospy.Subscriber("/car/pid_telemetry",Vector3,callback)

	pid_params = Vector3()
	pid_params.x = 0.5
	pid_params.y = 0.0
	pid_params.z = 0.0

	rate = rospy.Rate(0.01) # 10hz
	while not rospy.is_shutdown():
		pub.publish(pid_params)
		rate.sleep()
 



if __name__ == '__main__':
	listener()


