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









class arm_kinematics:
	
	

	def __init__(self):
  		moveit_commander.roscpp_initialize(sys.argv)
		rospy.init_node('arm_kinematics', anonymous = True)
		self.armPub = rospy.Publisher('/arm_controller/position_command', JointPositions, queue_size=10)
		self.gripPub = rospy.Publisher('/gripper_controller/position_command', JointPositions, queue_size=10)
		rospy.Subscriber('/box_pose', Pose, self.kinematics)
 		self.robot = moveit_commander.RobotCommander()
 		self.scene = moveit_commander.PlanningSceneInterface()
  		self.group = moveit_commander.MoveGroupCommander("arm_1")
  		self.group.set_goal_position_tolerance(0.01)
  		self.group.set_goal_orientation_tolerance(0.01)


		self.home = [3.0, 1.25, -1.0, 2.8, 3.07]
		self.gripperWidthAtGrasp = 0.0012
		self.value = [0 for i in range(6)]
	

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


                  
	def publish_gripper_width(self, width):
                  
       		desiredPositions = JointPositions()

        	jointCommands = []

        	joint = JointValue()
        	joint.joint_uri = "gripper_finger_joint_l"
        	joint.unit = "m"
        	joint.value = width
        	jointCommands.append(joint)

        	joint = JointValue()
        	joint.joint_uri = "gripper_finger_joint_r"
        	joint.unit = "m"
        	joint.value = width
        	jointCommands.append(joint)

        	desiredPositions.positions = jointCommands

        	self.gripPub.publish(desiredPositions)




	def kinematics(self, pose):

		#print(pose)

		pose_target = Pose()

		pose_target.orientation.x = pose.orientation.x
  		pose_target.orientation.y = pose.orientation.y
  		pose_target.orientation.z = pose.orientation.z
  		pose_target.orientation.w = pose.orientation.w
		
  		pose_target.position.x = pose.position.x + 0.02
  		pose_target.position.y = pose.position.y + 0.02
  		pose_target.position.z = pose.position.z + 0.07


  		self.group.set_pose_target(pose_target)
  		
		plan1 = self.group.plan()
  		#print(plan1.joint_trajectory.points[-1].positions)

  		self.group.go(wait=True)
		
		self.value[:] = plan1.joint_trajectory.points[-1].positions
		
		
		rospy.sleep(0.5)
		self.publish_arm_joint_positions(self.value)     # Grasp position - 7cm above
		print("----- PUBLISHED ----- GRASP POSITION 7cm -----")
		# Open gripper
		rospy.sleep(0.1)
		gripper_width = 0.0114
		self.publish_gripper_width(gripper_width)
		print("----- PUBLISHED ----- GRIPPER OPEN -----")
		rospy.sleep(0.2)

		
		# Grasp position
		pose_target.position.z = pose.position.z + 0.03
  		self.group.set_pose_target(pose_target)
  		
		plan1 = self.group.plan()
		self.group.go(wait=True)
		self.value[:] = plan1.joint_trajectory.points[-1].positions

		rospy.sleep(0.1)
		self.publish_arm_joint_positions(self.value)
		print("----- PUBLISHED ----- GRASP POSITION -----")


		# Closing gripper
		rospy.sleep(0.5)
		gripper_width = 0.0012
		self.publish_gripper_width(gripper_width)
		print("----- PUBLISHED ----- GRIPPER CLOSED -----")
            	rospy.sleep(0.5)
		

		# Home Position
		self.publish_arm_joint_positions(self.home)
		print("----- PUBLISHED ----- HOME POSITION -----")

		
		moveit_commander.roscpp_shutdown()


def main(args):

	arm_kinematics()
	logging.disable(logging.CRITICAL)

	try:
		rospy.spin()
	except KeyboardInterrupt:
        	print("Shutting down")

if __name__ == '__main__':
	main(sys.argv)
