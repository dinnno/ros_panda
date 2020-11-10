#!/usr/bin/env python

import rospy
import tf
import numpy as np
import time
from math import pi
import sys 

#import moveit_pkg, franka_pkg
import moveit_commander
import actionlib
from moveit_commander import MoveGroupCommander
from moveit_commander.conversions import pose_to_list

#import rosmsg
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String, Float64

#import custom pkg
from gai_franka_example.srv import MoveArm
from utils import pose_quat2rpy

def all_close(goal, actual, tolerance): # using homing, go_to_pose_goal 

    all_equal = True
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

    return True

class gai_arm_controller(object):
    def __init__(self):
        '''
        initialize moveit_commander and a ros node
        ''' 
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('gai_arm_controller', anonymous=True)
        robot = moveit_commander.RobotCommander()
        group_name = "panda_arm" 
        move_group = moveit_commander.MoveGroupCommander(group_name)  
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                       moveit_msgs.msg.DisplayTrajectory,
                                                       queue_size=20)      #for visualize to RViz 
        self.move_group = move_group

    def homing(self): #for reset

        move_group = self.move_group
        move_group.set_max_velocity_scaling_factor(0.1)
        move_group.set_max_acceleration_scaling_factor(0.1)

        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = 0
        joint_goal[1] = -pi/2
        joint_goal[2] = 0
        joint_goal[3] = -pi/2
        joint_goal[4] = 0
        joint_goal[5] = pi/2
        joint_goal[6] = 0
        move_group.go(joint_goal, wait=True)
        move_group.stop()


        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

    def go_to_pose_goal(self, x, y, z): #rot_x, rot_y, rot_z): # require x,y,z input, for go to goal

        move_group = self.move_group
        move_group.set_max_velocity_scaling_factor(0.1)
        move_group.set_max_acceleration_scaling_factor(0.1)
        
        curr_pose = move_group.get_current_pose().pose
        print move_group.get_current_pose().pose.orientation

        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation = curr_pose.orientation
        pose_goal.position.x = x
        pose_goal.position.y = y
        pose_goal.position.z = z

        move_group.set_pose_target(pose_goal)
        plan = move_group.go(wait=True)
        move_group.stop()
        move_group.clear_pose_targets()


        current_pose = self.move_group.get_current_pose().pose
        print(current_pose)
        return all_close(pose_goal, current_pose, 0.01)

    
if __name__ == "__main__":
    
    con = gai_arm_controller()
    while True:
        curr_pose = con.move_group.get_current_pose().pose
        print(curr_pose)
        print(pose_quat2rpy(curr_pose))
        time.sleep(2)
