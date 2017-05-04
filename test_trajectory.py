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



class arm_kinematics:
	
	

	def __init__(self):
		rospy.init_node('arm_kinematics', anonymous = True)
		self.armPub = rospy.Publisher('/arm_controller/position_command', JointPositions, queue_size=10)
		self.gripPub = rospy.Publisher('/gripper_controller/position_command', JointPositions, queue_size=10)

		self.kinematics()

	def Inverse_Kinematics(self, position, inclination_link, rotation_cube):
	
		mode_reversed = 0		
		pose_x = position[0] * 1000
		pose_y = position[1] * 1000
		pose_z = position[2] * 1000
		gamma = inclination_link

		if rotation_cube < -1.57:
			teta_5 = rotation_cube + 1.57
		if rotation_cube > 1.57:
			teta_5 = rotation_cube - 1.57
		else:
		 	teta_5 = rotation_cube

	
		a2 = 155
		a3 = 135
		a4 = 176

		link_12_xy = 33 # vector between arm_link_1 and arm_link_2
		
		#ANGLE JOINT_1 CALCULATION
		zeta = math.atan2(pose_y,(pose_x - 167))
		if zeta > 1.57:
			zeta = - (3.14 - zeta)
			mode_reversed = 1
			print "MODE_REVERSED ON"
		print("ZETA",math.degrees(zeta))

		arm_2_x = link_12_xy * math.cos(-zeta)
		arm_2_y = link_12_xy * math.sin(-zeta)
	
		print(arm_2_x, arm_2_y)

		# distance to arm_link_2
		ox = math.sqrt((pose_x - 167 - arm_2_x)**2 + (pose_y + arm_2_y)**2)
		if mode_reversed == 1:
			ox = -math.sqrt((pose_x - 167 - arm_2_x)**2 + (pose_y + arm_2_y)**2)
	
		oy = - 200 + (pose_z) 
		fi = gamma
		print("OX and OY :" , ox,oy)


		xw = ox - a4*math.cos(fi)
		yw = oy - a4*math.sin(fi)
		l = math.sqrt(xw**2 + yw**2)
		print("XW,YW,L",xw,yw,l)

		#teorema dei coseni, angolo joint 3
		cos_teta3 = (xw**2 + yw**2 - a2**2 - a3**2)/(2*a2*a3)
		print("cos_teta3 ", cos_teta3)
		sin_teta3 = + math.sqrt(1 - cos_teta3**2)

		teta_3 =  math.atan2(sin_teta3,cos_teta3)
		print("angolo joint 3 appena calcolato" , math.degrees(teta_3))


		teta2_c = a2 + a3 * cos_teta3
		teta2_s = a3 * sin_teta3

		teta2_in = math.atan2(teta2_s, teta2_c)
		teta2_out = math.atan2(yw, xw)

		teta_2 = teta2_out - teta2_in
		print("angolo joint 2", teta_2, math.degrees(teta_2))

		teta_4 = fi - (teta_2 + teta_3)
		joint_4 = 4.930555 - (3.14 + teta_4)
		print("angolo joint 4", teta_4, math.degrees(teta_4))
		if mode_reversed == 1:		
			teta_4 = 6.28 + teta_4
			print("teta_4 modificato", teta_4, math.degrees(teta_4))

		if ((teta_2 < 0) or (joint_4 > 3.4)) and (mode_reversed != 1):
			print("NEGATIVE SINE MODE ON!")	
			sin_teta3 = - math.sqrt(1 - cos_teta3**2)
			teta_3 =  math.atan2(sin_teta3,cos_teta3)
			teta2_c = a2 + a3 * cos_teta3
			teta2_s = a3 * sin_teta3

			teta2_in = math.atan2(teta2_s, teta2_c)
			teta2_out = math.atan2(yw, xw)

			teta_2 = teta2_out - teta2_in
			print("NEW teta_2 ", teta_2, math.degrees(teta_2))
			teta_4 = fi - (teta_2 + teta_3)
			
		


		teta_1 = zeta
		teta_tool = gamma

		#-----------------------------
		# MAPPING for Vrep
		joint_1 = 2.94961 - teta_1
		joint_2 = 2.70526 - teta_2
		joint_3 = 0.523599 - (3.14 + teta_3)
		joint_4 = 4.930555 - (3.14 + teta_4)
		joint_5 = 2.92 + teta_5 - zeta  #to compensate if x*y is different from zero

		if mode_reversed == 1:
			joint_5 = 0


		
		print("JOINT_1: ", teta_1, math.degrees(teta_1))
		print("JOINT_2: ", teta_2, math.degrees(teta_2))
		print("JOINT_3: ", teta_3, math.degrees(teta_3))
		print("JOINT_4: ", teta_4, math.degrees(teta_4))



		if joint_5 < 0:
			joint_5 = joint_5 + 3.14
		if joint_5 > 4.71:
			joint_5 = joint_5 - 3.14
		
		joint_q = (joint_1, joint_2, joint_3, joint_4, joint_5)

		return joint_q




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


        

	def kinematics(self):
		self.target= [0.4, 0.1, 0.05]
		self.rotation = 0
		self.inclination =  - 1.57
		
		#hertz = 100	#frequenza del loop nel pubblicare ogni punto
		#samples = 1000 	#numero di punti nella traiettoria
		#offset = 35 	# in millimetri, di quanto scendere con la gripper per la presa
		#self.trajectory_calc(self.end_effector, self.target, samples, offset)
		#self.ok = self.trajectory_pub(self.q, hertz, samples)
		
		self.value = self.Inverse_Kinematics(self.target, self.inclination, self.rotation)
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
