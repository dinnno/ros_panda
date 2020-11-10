#!/usr/bin/env python
import rospy
import time
from gai_arm_function import gai_arm_controller
from gai_gripper_function import gai_gripper_controller

def main():
    gai_arm = gai_arm_function()
    gai_gripper = gai_gripper_function()


    gai_arm.homing()
    gai_arm.go_to_pose_goal(0.3, 0.0, 0.5)
    time.sleep(2)
    gai_gripper.gripper_grasp(width = 0.025)
    #time.sleep(2)
    #gai_arm.go_to_pose_goal(0.3, 0.1, 0.3)
    #gai_gripper.gripper_grasp(width=0.04)


if __name__ =='__main__':
    main()