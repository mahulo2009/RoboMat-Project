#!/usr/bin/env python

import rospy
import roslib; roslib.load_manifest("rosserial_arduino")
import actionlib
from geometry_msgs.msg import Vector3

from rosserial_arduino.srv import *


def callback(data):
	target = data.x
	current = data.y
	demanded = data.z
	rospy.loginfo(target)
	rospy.loginfo(current)
	rospy.loginfo(demanded)

def listener():
	rospy.init_node("robot_mat_auto_tunning",anonymous=True)
	rospy.Subscriber('/car/pid_telemetry_wheel_1_msg', Vector3, callback)

	rospy.set_param('/robomat/pid_kp', 0.7)
	rospy.set_param('/robomat/pid_ki', 0.0)	
	rospy.set_param('/robomat/pid_kd', 0.0)

	rospy.wait_for_service('test_srv')
	service = rospy.ServiceProxy('test_srv', Test)
	result = service("hola")

	rospy.spin()
	

if __name__ == '__main__':
	listener()


