#!/usr/bin/env python

import rospy
import std_msgs.msg
from geometry_msgs.msg import PoseStamped, Pose
from sensor_msgs.msg import JointState
from brics_actuator.msg import JointPositions, JointValue, JointVelocities
import sys
import copy
import numpy as np
import math
import time
import moveit_commander
import moveit_msgs.msg







class arm_kinematics:
	
	

	def __init__(self):
  		moveit_commander.roscpp_initialize(sys.argv)
		rospy.init_node('arm_kinematics', anonymous = False)
		self.armPub = rospy.Publisher('/arm_controller/position_command', JointPositions, queue_size=10)
		rospy.Subscriber('/aruco_single/pose', PoseStamped, self.kinematics)
 		robot = moveit_commander.RobotCommander()
 		scene = moveit_commander.PlanningSceneInterface()
  		self.group = moveit_commander.MoveGroupCommander("arm_1")
  		self.group.set_goal_position_tolerance(0.01)
  		self.group.set_goal_orientation_tolerance(0.01)

		
		self.pos = JointValue()
		self.uri = 0
		self.value = [0 for i in range(6)]
		


	def kinematics(self, posestamped):

		print(posestamped.pose)
		
		pose_target = Pose()

		pose_target.orientation.x = posestamped.pose.orientation.x
  		pose_target.orientation.y = posestamped.pose.orientation.y
  		pose_target.orientation.z = posestamped.pose.orientation.z
  		pose_target.orientation.w = posestamped.pose.orientation.w
		
  		pose_target.position.x = posestamped.pose.position.x
  		pose_target.position.y = posestamped.pose.position.y
  		pose_target.position.z = posestamped.pose.position.z + 0.1

  		self.group.set_pose_target(pose_target)
  		
		plan1 = self.group.plan()
  		#print(plan1.joint_trajectory.points[-1].positions)

  		self.group.go(wait=True)

		self.value[:] = plan1.joint_trajectory.points[-1].positions
		

		print(self.value)

		jp = JointPositions()	

		for i in range(5):
			jv = JointValue()
			jv.joint_uri = "arm_joint_" + str(i + 1)
			jv.unit = "rad"
			jv.value = self.value[i]
			jp.positions.append(jv)

		rospy.sleep(0.5)
		self.armPub.publish(jp)
		print("published")

		moveit_commander.roscpp_shutdown()

def main(args):
	arm_kinematics()

	try:
		
		rospy.spin()
	except KeyboardInterrupt:
        	print("Shutting down")

if __name__ == '__main__':
	main(sys.argv)
