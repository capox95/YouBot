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
	
		link = [0, 33, 155, 135, 176] #links lenght

		mode_reversed = 0 # equal to 1 if not (-135 < teta_1 < 135)
		
		pose_x = position[0] * 1000
		pose_y = position[1] * 1000
		pose_z = position[2] * 1000
		gamma = inclination_link

		teta_5 = rotation_cube
	
		#ANGLE JOINT_1 CALCULATION
		teta_1 = math.atan2(pose_y,(pose_x - 167))
		
		if (teta_1 > 2.35) or (teta_1 < -2.35):
			teta_1 = - (3.14 - teta_1)
			mode_reversed = 1
			#print "_________ MODE_REVERSED ON _____________"

		link_fix = [(link[1] * math.cos(-teta_1)), (link[1] * math.sin(-teta_1))]



		# RRR JOINT 2-3-4 CALCULATION
		target_xy = math.sqrt((pose_x - 167 - link_fix[0])**2 + (pose_y + link_fix[1])**2)
		if mode_reversed == 1:
			target_xy = -target_xy
	
		target_z = - 200 + (pose_z) 
		fi = gamma


		joint_4x = (target_xy - link[4]*math.cos(fi))
		joint_4y = (target_z - link[4]*math.sin(fi))		
		distance_joint_4 = math.sqrt(joint_4x**2 + joint_4y**2)

		#teorema dei coseni, angolo joint 3
		cos_teta3 = (distance_joint_4**2 - link[2]**2 - link[3]**2)/(2*link[2]*link[3])
		sin_teta3 = + math.sqrt(1 - cos_teta3**2)
		teta_3 =  math.atan2(sin_teta3,cos_teta3)


		cos_teta2 = link[2] + link[3] * cos_teta3
		sin_teta2 = link[3] * sin_teta3

		#teta2_in = math.atan2(sin_teta2, cos_teta2)
		#teta2_out = math.atan2(vector_wrist[1], vectort_wrist[0])

		teta_2 = math.atan2(joint_4y, joint_4x) - math.atan2(sin_teta2, cos_teta2)

		teta_4 = fi - (teta_2 + teta_3)
		joint_4 = 4.930555 - (3.14 + teta_4)
		if mode_reversed == 1:		
			teta_4 = 6.28 + teta_4

		if ((teta_2 < 0) or (joint_4 > 3.4)) and (mode_reversed != 1):
			#print(" __________ NEGATIVE SINE MODE ON! _________ ")	
			sin_teta3 = - math.sqrt(1 - cos_teta3**2)
			teta_3 =  math.atan2(sin_teta3,cos_teta3)
			cos_teta2 = link[2] + link[3] * cos_teta3
			sin_teta2 = link[3] * sin_teta3
			teta_2 = math.atan2(joint_4y, joint_4x) - math.atan2(sin_teta2, cos_teta2)
			teta_4 = + fi - (teta_2 + teta_3)
			


		if (mode_reversed == 1) and (teta_2 < -3.14):
			#print " _________ ANGOLO ESTREMO __________ teta_2: ", teta_2
			teta_2 = 6.28 + teta_2 
			teta_4 = 6.28 - (-fi + (teta_2) + teta_3)


		#-----------------------------
		# MAPPING for Vrep
		joint_1 = 2.94961 - teta_1
		joint_2 = 2.70526 - teta_2
		joint_3 = 0.523599 - (3.14 + teta_3)
		joint_4 = 4.930555 - (3.14 + teta_4)
		joint_5 = 2.92 + teta_5 - teta_1  #to compensate joint_1 rotation

		
		joint_q = (joint_1, joint_2, joint_3, joint_4, joint_5)


		return joint_q
	
