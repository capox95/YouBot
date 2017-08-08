#!/usr/bin/env python

import rospy
import sys
import copy
import numpy as np
import math
import time
import logging
import std_msgs.msg
from geometry_msgs.msg import Pose
from brics_actuator.msg import JointPositions, JointValue
from tf.transformations import euler_from_quaternion

from lib_inverse_kinematics_mod import Inverse_Kinematics


class arm_kinematics:
	
	

	def __init__(self):
		rospy.init_node('arm_kinematics', anonymous = True)
		self.armPub = rospy.Publisher('/arm_controller/position_command', JointPositions, queue_size=10)
		self.gripPub = rospy.Publisher('/gripper_controller/position_command', JointPositions, queue_size=10)

		rospy.Subscriber('/ee_pose', Pose, self.end_effector_pose)
		rospy.Subscriber('/box_pose', Pose, self.kinematics)


	
	def end_effector_pose(self, pose):
		ee_p =[pose.position.x, pose.position.y, pose.position.z]
		self.end_effector = ee_p
		

	def publish_arm_joint_positions(self, joint_positions):

        	desiredPositions = JointPositions()

        	jointCommands = []

        	for i in range(5):
            		joint = JointValue()
            		joint.joint_uri = "arm_joint_" + str(i+1)
            		joint.unit = "rad"
            		joint.value = joint_positions[i]

           		jointCommands.append(joint)
            
        	desiredPositions.positions = jointCommands

       		self.armPub.publish(desiredPositions)

	def trajectory_calc(self, end_effector, target, number, offset_grasp):
		number_real = number + number/5
		self.q = [0 for i in range(number_real)]

		v = [end_effector[0] - target[0], end_effector[1] - target[1], end_effector[2] - abs(target[2])]
		dim =  np.linalg.norm(v)
		print ("Magnitude of Vector: ", dim) 
	
		i = 0
		k = 0
		t = 0
		inc = 1.0/number
		for i in range(number):
			i = i + 1
			t = 0 + i*inc 
			p = np.array(end_effector) - (t*np.array(v))
			self.q[i-1] = Inverse_Kinematics(p, self.inclination, self.rotation)

		i = 0
		inc2 = 1.0/(number/5.0)
		for i in range(number/5):
			i = i + 1
			k = 0 + i*inc2
			p2 = [p[0], p[1], (p[2] - k*(offset_grasp/1000.0))]
			self.q[i-1+number] = Inverse_Kinematics(p2, self.inclination, self.rotation)

		print("self.q",self.q[i])

	
	def trajectory_pub(self, joint_q, frequency, samples):
		samples_tot = samples + samples/5
		r = rospy.Rate(frequency)
		i = 0
		for i in range(samples_tot):
			i = i + 1 
			self.publish_arm_joint_positions(joint_q[i-1])
			r.sleep()
		return 1





        

	def kinematics(self, pose):
		self.target= [pose.position.x, pose.position.y, pose.position.z]
		(roll,pitch,yaw) = euler_from_quaternion([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
		yaw = yaw + 3.14
		self.rotation = yaw
		print("yaw", yaw)

		self.inclination = - 1.57

		hertz = 150	#frequenza del loop nel pubblicare ogni punto
		samples = 100 	#numero di punti nella traiettoria
		offset = 10	# in millimetri, di quanto scendere con la gripper per la presa
		self.trajectory_calc(self.end_effector, self.target, samples, offset)
		self.ok = self.trajectory_pub(self.q, hertz, samples)


		print("__END__")
		rospy.sleep(20)	
		
		
		

def main(args):

	arm_kinematics()
	#logging.disable(logging.CRITICAL)

	try:
		rospy.spin()
	except KeyboardInterrupt:
        	print("Shutting down")

if __name__ == '__main__':
	main(sys.argv)
