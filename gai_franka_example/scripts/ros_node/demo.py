#!/usr/bin/env python
import rospy
import sys
import time
import numpy as np
from gai_franka_example.srv import *
'''
client for captured image
'''
def capture_image():
    image_saver = rospy.ServiceProxy('capture_image', ImagePath)
    rospy.loginfo("waiting for image save server...")
    image_saver.wait_for_service()
    request = ImagePathRequest(True)
    response = image_saver(request)    
    image_path = response.image_path

    return image_path


'''
client for move_eef and get_grasp_point
'''
def get_grasp_point(image_path):
    grasp_generator = rospy.ServiceProxy('generate_grasp', GenerateGrasp)
    rospy.loginfo("waiting for grasp generator server...")
    grasp_generator.wait_for_service()
    request = GenerateGraspRequest(image_path)
    response = grasp_generator(request)
    grasp_point = response # type(grasp_point) ~ GenerateGraspResponse

    return grasp_point

def grasp_target(grasp_point):
    # move end effector to target pose
    response_move = _move_eef_client(x=grasp_point.x,
                                     y=grasp_point.y,
                                     z=grasp_point.z,
                                     rot_x=grasp_point.rot_x,
                                     rot_y=grasp_point.rot_y,
                                     rot_z=grasp_point.rot_z)
    if not response_move.success:
        rospy.loginfo("fail to move eef")
    # grasp
    _grasp_target_client(width=)



def _move_eef_client(x,y,z,rot_x, rot_y, rot_z):
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


def _grasp_target_client(width):
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



if __name__ == '__main__':
    rospy.init_node('gai_demo', anonymous=True)
    rospy.loginfo("start demo")
    rate = rospy.Rate(10) # 10hz

    image_path = capture_image()
    grasp_point = get_grasp_point(image_path)
    is_success = grasp_target(grasp_point)

    if is_success:
        rospy.loginfo("success to grasp")
