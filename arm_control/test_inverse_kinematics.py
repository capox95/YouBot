#!/usr/bin/env python


#############################################################

# this python allow you to test the inverse kinematics library
# you need to set the target position for the end effector in self.target variable

#############################################################




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

from lib_inverse_kinematics import Inverse_Kinematics



class arm_kinematics:
	
	

	def __init__(self):
		rospy.init_node('arm_kinematics', anonymous = True)
		self.armPub = rospy.Publisher('/arm_controller/position_command', JointPositions, queue_size=10)
		self.gripPub = rospy.Publisher('/gripper_controller/position_command', JointPositions, queue_size=10)

		self.kinematics()





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
		print(desiredPositions)
       		self.armPub.publish(desiredPositions)


        

	def kinematics(self):
		self.target= [0.2, 0.0, 0.65]
		self.rotation = 0
		self.inclination =  + 1.57
		
		self.value =Inverse_Kinematics(self.target, self.inclination, self.rotation)
		print(self.value)
		rospy.sleep(0.2)
		self.publish_arm_joint_positions(self.value)


		print("FINITO")
		rospy.sleep(20)	
		
		
		

def main(args):

	arm_kinematics()
	logging.disable(logging.CRITICAL)

	try:
		rospy.spin()
	except KeyboardInterrupt:
        	print("Shutting down")

if __name__ == '__main__':
	main(sys.argv)
