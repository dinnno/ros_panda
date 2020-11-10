#!/usr/bin/env python
import rospy
import sys
import time
import numpy as np 
from gai_franka_example.srv import *


def move_eef_client(x,y,z,rot_x, rot_y, rot_z):
    rospy.loginfo("waiting for move service")
    rospy.wait_for_service('go_to_goal')
    try: 
        go_to_goal = rospy.ServiceProxy('go_to_goal', MoveArm)
        request_move = MoveArmRequest(x, y, z, rot_x, rot_y, rot_z)
        resp_move = go_to_goal(request_move)
        print"respond.."
        return resp_move
    except rospy.ServiceException, e:
        print "call failed move_eef"


def grasp_target_client(width):
    rospy.loginfo("waiting for grasp service")
    rospy.wait_for_service('grasp_obect')
    try: 
        grasp_object = rospy.ServiceProxy('grasp_object', GraspObject)
        reqest_grasp = GraspObjectRequest(width)
        resp_grasp = grasp_object(reqest_grasp)
        print"respond...."
        return resp_grasp
    except rospy.ServiceException, e:
        print "call failded grasp_object"

if __name__ == "__main__":

    rospy.init_node('franka_control_client')
    rospy.loginfo('start client')
    rate = rospy.Rate(10)

    if len(sys.argv) == 7:
        x = float(sys.argv[1])
        y = float(sys.argv[2])
        z = float(sys.argv[3])
        rot_x = float(sys.argv[4])
        rot_y = float(sys.argv[5])
        rot_z = float(sys.argv[6])
    else:
        sys.exit()
    
    print "requesting %s,%s,%s,%s,%s,%s"%(x, y, z, rot_x, rot_y, rot_z)
    move_eef_client(x, y, z, rot_x, rot_y, rot_z)
    time.sleep(2)
    width = 0.025
    grasp_target_client(width)
    

























