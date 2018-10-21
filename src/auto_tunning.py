#!/usr/bin/env python

import numpy as np

import rospy
import roslib; roslib.load_manifest("rosserial_arduino")
import actionlib
from geometry_msgs.msg import Vector3Stamped
from rosserial_arduino.srv import *


recording = False
values = []
times = []
count = 0
count_max = 200
time_sampling = 100
start_recording_time = 0

def analysis(data,t,target):
	global time_sampling

	#print(data)

	npa = np.array(data)

	rise_time = ( (np.nonzero(npa>target))[0][0] * time_sampling ) / ( 1000.0 )
	over_shoot = np.max(npa)


	print "OS: %f%s"%((npa.max()/npa[-1]-1)*100,'%')
	print "Tr: %fs"%(t[next(i for i in range(0,len(npa)-1) if npa[i]>npa[-1]*.90)]-t[0])
	print "Ts: %fs"%(t[next(len(npa)-i for i in range(2,len(npa)-1) if abs(npa[-i]/npa[-1])>1.02)]-t[0])
	
	#print("rise_time:",rise_time)
	#print("over_shoot",over_shoot)	
	

#	settling_time_array = np.where( (npb>target-0.5) & (npb<target+0.5) )
#	print("settling_time_array",settling_time_array)

def callback(data):
	global recording
	global count_max
	global count

	target = data.vector.x
	current = data.vector.y
	demanded = data.vector.z

	if recording:
		if count < count_max:
			if data.header.stamp.secs > start_recording_time.secs:  
		
				values.append(current)
				times.append(data.header.stamp.secs)
			count = count + 1
		else:
			count_max =	0 
			recording = False
			analysis(values,times,target)

def listener():
	global recording
	global start_recording_time

	rospy.init_node("robot_mat_auto_tunning",anonymous=True)
	rospy.Subscriber('/car/pid_telemetry_wheel_1_msg', Vector3Stamped, callback)

	rospy.set_param('/robomat/pid_kp', 0.7)
	rospy.set_param('/robomat/pid_ki', 0.5)	
	rospy.set_param('/robomat/pid_kd', 0.0)

	start_recording_time = rospy.Time.now()
	print("start recording time:",start_recording_time)
	recording = True	

	try:
		rospy.wait_for_service('test_srv')
		service = rospy.ServiceProxy('test_srv', Test)
		result = service("hola")
	except:
		pass
 

	print(recording)
	rospy.spin()

if __name__ == '__main__':
	listener()


