#!/usr/bin/env python

import rospy
import sys
import copy
import numpy as np
import math
import time
import logging
import std_msgs.msg
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import Pose
from brics_actuator.msg import JointPositions, JointValue
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Quaternion


def Inverse_Kinematics(position, inclination_link, rotation_cube):
	
				
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

		link_12_xy = 33 # vector btw arm_link_1 and arm_link_2
		
		#teta_1 calculation
		zeta = math.atan2(pose_y,(pose_x - 167))
		#print(zeta)

		arm_2_x = link_12_xy * math.cos(-zeta)
		arm_2_y = link_12_xy * math.sin(-zeta)
	


		# distance to arm_link_2
		ox = math.sqrt((pose_x - 167 - arm_2_x)**2 + (pose_y + arm_2_y)**2)	
		oy = - 200 + (pose_z) 
		fi = - 1.57
		#print("ox and oy :" , ox,oy)


		xw = abs(ox) - a4*math.cos(fi)
		if ox < 0:
			xw = - xw
		yw = oy - a4*math.sin(fi)
		l = math.sqrt(xw**2 + yw**2)
		#print(xw,yw,l)

		cos_teta3 = (xw**2 + yw**2 - a2**2 - a3**2)/(2*a2*a3)
		#print("cos_teta3 ", cos_teta3)
		sin_teta3 = + math.sqrt(1 - cos_teta3**2)

		teta_3 =  math.atan2(sin_teta3,cos_teta3)
		#print("teta_2 appena calcolata" , math.degrees(teta_3))


		teta2_c = a2 + a3 * cos_teta3
		teta2_s = a3 * sin_teta3

		teta2_in = math.atan2(teta2_s, teta2_c)
		teta2_out = math.atan2(yw, xw)

		teta_2 = teta2_out - teta2_in
		teta_4 = fi - (teta_2 + teta_3)
		joint_4 = 4.930555 - (3.14 + teta_4)

		if (teta_2 < 0) or (joint_4 > 3.4):
			#print("teta1 negative!" , math.degrees(teta_2))	
			sin_teta3 = - math.sqrt(1 - cos_teta3**2)
			teta_3 =  math.atan2(sin_teta3,cos_teta3)
			teta2_c = a2 + a3 * cos_teta3
			teta2_s = a3 * sin_teta3

			teta2_in = math.atan2(teta2_s, teta2_c)
			teta2_out = math.atan2(yw, xw)

			teta_2 = teta2_out - teta2_in
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

		if joint_5 < 0:
			joint_5 = joint_5 + 3.14
		if joint_5 > 4.71:
			joint_5 = joint_5 - 3.14
		
		joint_q = (joint_1, joint_2, joint_3, joint_4, joint_5)

		return joint_q

	
