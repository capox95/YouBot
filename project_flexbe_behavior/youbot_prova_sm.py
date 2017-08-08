#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from project_flexbe_states.JointValues import JointValuePub
from project_flexbe_states.IK_Solver import IKSolver
from project_flexbe_states.GripperWidth import GripperStateWidth
from project_flexbe_states.GripperCheck import GripperStateEffort
from project_flexbe_states.publish_pose_state_IK import PublishPoseStateIK
from project_flexbe_states.IK_Solver_Trajectory import IKSolverTrajectory
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sat Apr 08 2017
@author: Alessio Caporali
'''
class Youbot_ProvaSM(Behavior):
	'''
	Invese Kinematics State Machine for the Kuka Youbot
	'''


	def __init__(self):
		super(Youbot_ProvaSM, self).__init__()
		self.name = 'Youbot_Prova'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		pose_cube = "/box_pose"
		inclination = 1.57
		height_above = 0
		height_grasp = 30
		j_search = [2.95, 1.5, -1.6, 3.5, 2.95]
		width_open = 0.0115
		j_ground = [2.95, 2.35, -1.6, 2.6, 2.95]
		hertz = 150
		samples = 100
		offset = 45
		th = 5
		time = 1500
		width = 0.004
		wait = 2
		target_pose = [-0.05, 0.0, 0.14]
		rotation_ee = 0
		target_pose_above = [-0.025, 0.0, 0.155]
		inclination_ee = 1.57
		waipoint1 = [0.2, 0.0, 0.0]
		waipoint2 = [0.2, 0.0, 0.0]
		# x:23 y:231, x:226 y:305
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:46 y:85
			OperatableStateMachine.add('Search_Target',
										JointValuePub(target_pose=j_search),
										transitions={'done': 'Open_Gripper', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:657 y:117
			OperatableStateMachine.add('IK_Plate_Platform',
										IKSolver(target_pose=target_pose, inclination_ee=inclination_ee, rotation_ee=rotation_ee),
										transitions={'found': 'Gripper_Plate_Release', 'unavailable': 'failed'},
										autonomy={'found': Autonomy.Off, 'unavailable': Autonomy.Off})

			# x:451 y:199
			OperatableStateMachine.add('Gripper_Plate_Release',
										GripperStateWidth(width=width_open),
										transitions={'done': 'IK_Plate_2', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:311 y:133
			OperatableStateMachine.add('IK_Plate_2',
										IKSolver(target_pose=target_pose_above, inclination_ee=inclination_ee, rotation_ee=rotation_ee),
										transitions={'found': 'Home', 'unavailable': 'failed'},
										autonomy={'found': Autonomy.Off, 'unavailable': Autonomy.Off})

			# x:803 y:78
			OperatableStateMachine.add('Close_Check_Gripper',
										GripperStateEffort(width=width, threshold=th, time=time),
										transitions={'done': 'IK_Plate_Platform', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:199 y:29
			OperatableStateMachine.add('Open_Gripper',
										GripperStateWidth(width=width_open),
										transitions={'done': 'Pose_Target', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:142 y:173
			OperatableStateMachine.add('Home',
										JointValuePub(target_pose=j_search),
										transitions={'done': 'finished', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:364 y:32
			OperatableStateMachine.add('Pose_Target',
										PublishPoseStateIK(topic=pose_cube),
										transitions={'received': 'Trajectory', 'unavailable': 'failed'},
										autonomy={'received': Autonomy.Off, 'unavailable': Autonomy.Off},
										remapping={'output_value': 'output_value', 'message': 'message'})

			# x:591 y:28
			OperatableStateMachine.add('Trajectory',
										IKSolverTrajectory(hertz=hertz, samples=samples, offset=offset, inclination_ee=inclination_ee),
										transitions={'found': 'Close_Check_Gripper', 'unavailable': 'failed'},
										autonomy={'found': Autonomy.Off, 'unavailable': Autonomy.Off},
										remapping={'pose': 'output_value'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
