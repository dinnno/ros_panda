#!/usr/bin/env python

import rospy
from gai_arm_function import gai_arm_controller
from gai_gripper_function import gai_gripper_controller
from gai_franka_example.srv import *

def move_eef_server(move_point): # move_point: class::MoveArmRequest

    gai_arm = gai_arm_controller() # gai_arm: class::gai_arm_controller
    rospy.loginfo("try to move to goal")
    move_arm = gai_arm.go_to_pose_goal(x=move_point.x, 
                                       y=move_point.y, 
                                       z=move_point.z, 
                                       rot_x=move_point.rot_x, 
                                       rot_y=move_point.rot_y,
                                       rot_z=move_point.rot_z) # move_arm: bool
    move_success = False 
    
    if move_success:
        return MoveArmResponse(True)
    else :
        return MoveArmResponse(False)
    

def grasp_target_server(grasp_width): #grasp_point: class::GraspObjectRequest

    rospy.loginfo("try to grasp target")
    gai_gripper = gai_gripper_controller()
    grasp = gai_gripper.gripper_grasp(width=grasp_width.width)
    grasp_success = False

    if grasp_success:
        return GraspObjectResponse(True)
    else: 
        return GraspObjectResponse(False)



if __name__ == "__main__":
    rospy.init_node('franka_controller')
    s_move_eef = rospy.Service('go_to_goal', MoveArm, move_eef_server)
    print('ready to go to goal')
    s_grasp_object = rospy.Service('grasp_object', GraspObject,grasp_target_server)
    print('ready to grasp')
    rospy.spin()






