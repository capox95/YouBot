#!/usr/bin/env python


#############################################################
# this python allow you publish specific values for the arm #
# you need to set them on self.value variable               #
#############################################################




import sys
import rospy
from brics_actuator.msg import JointPositions, JointValue
import std_msgs.msg



class arm_position:
	
	

	def __init__(self):
		rospy.init_node('arm_position_command', anonymous = True)
		self.armPub = rospy.Publisher('/arm_controller/position_command', JointPositions, queue_size=10)
		self.gripPub = rospy.Publisher('/gripper_controller/position_command', JointPositions, queue_size=10)

		self.command()
        

	def command(self):
		self.value = [2.15, 1.0, -1.5, 3.6, 2.92]
		print("Target for the joint positions", self.value)
		target = JointPositions()

        	jointCommands = []

        	for i in range(5):
            		joint = JointValue()
            		joint.joint_uri = "arm_joint_" + str(i+1)
            		joint.unit = "rad"
            		joint.value = self.value[i]

           		jointCommands.append(joint)
            
        	target.positions = jointCommands
		rospy.sleep(0.2) #without not publishing well	
       		self.armPub.publish(target)

		print("published")
		rospy.sleep(2)	
		
		
		

def main(args):

	arm_position()

	try:
		rospy.spin()
	except KeyboardInterrupt:
        	print("Shutting down")

if __name__ == '__main__':
	main(sys.argv)
