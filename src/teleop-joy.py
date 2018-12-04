#!/usr/bin/env python

import rospy

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

vel_pub = rospy.Publisher('/car/cmd_vel', Twist,queue_size=10)

def joy_callback(data):

	#print(data)

	twist = Twist() 

	twist.linear.x =  0.5 * data.axes[1]
  	twist.angular.z = 1.5 * data.axes[0]

	#print(twist)

  	vel_pub.publish(twist);

if __name__ == "__main__":

	print("Hello Teleop Joy")

	rospy.init_node("telop_joy",anonymous=True)

	rospy.Subscriber("/joy",Joy,joy_callback)

 

	rospy.spin()



	
