#!/usr/bin/env python
import rospy
from gai_franka_example.srv import GenerateGrasp, GenerateGraspResponse


def generate_grasp(req):
    """
    type(req) == GenerateGraspRequest
    """
    #TODO:
    image_path = req.image_path
    rospy.loginfo("generate grasp from " + image_path)
    
    return GenerateGraspResponse(0., 0., 0., 0., 0., 0.)

def generate_grasp_server():
    rospy.init_node('generate_grasp_server')
    s = rospy.Service('generate_grasp', GenerateGrasp, generate_grasp)
    print("ready to generate grasp")
    rospy.spin()

if __name__ == '__main__':
    generate_grasp_server()