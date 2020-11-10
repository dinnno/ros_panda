import rospy
import tf
import numpy as np
import time
import math 
import sys 

#import moveit_pkg, franka_pkg
import moveit_commander
import actionlib
from moveit_commander import MoveGroupCommander
from franka_interface.gripper import GripperInterface

#import rosmsg
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String, Float64
from franka_gripper.msg import GraspAction, GraspGoal, GraspEpsilon, MoveGoal


class gai_gripper_controller(object): 
    def __init__(self):
        self.gripper_interface = GripperInterface()
        self._gripper_grasp = self.gripper_interface._grasp_action_client
        self._gripper_move = self.gripper_interface._move_action_client        

        moveit_commander.roscpp_initialize(sys.argv)
        robot = moveit_commander.RobotCommander()
        group_name = "hand" #(1)hand (2)panda_arm (3)panda_arm_hand
        move_group = moveit_commander.MoveGroupCommander(group_name)  
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                       moveit_msgs.msg.DisplayTrajectory,
                                                       queue_size=20)   
    
    def gripper_move(self, width,
                    epsilon_inner=0.05, epsilon_outer=0.05,
                    speed=0.1, force=60):
        goal = MoveGoal(width=width, speed=speed)
        rospy.loginfo('Moving gripper:\n{}'.format(goal))
        self._gripper_move.send_goal(goal)
        self._gripper_move.wait_for_result()
        if not self._gripper_move.get_result().success:
            rospy.logerr("Couldn't move gripper")
            sys.exit(1)

    def gripper_grasp(self, width,
                    epsilon_inner=0.05, epsilon_outer=0.05,
                    speed=0.1, force=60):
        epsilon = GraspEpsilon(inner=epsilon_inner, outer=epsilon_outer)
        goal = GraspGoal(width=width,
                         epsilon=epsilon,
                         speed=speed,
                         force=force)
        rospy.loginfo('Grasping:\n{}'.format(goal))
        self._gripper_grasp.send_goal(goal)
        self._gripper_grasp.wait_for_result()

        while not self._gripper_grasp.get_result().success:
            rospy.logerr('Failed to grasp, retry')
            self.gripper_move(width=0.025)
            self._gripper_grasp.send_goal(goal)
            self._gripper_grasp.wait_for_result()
            if rospy.is_shutdown():
                sys.exit(0)