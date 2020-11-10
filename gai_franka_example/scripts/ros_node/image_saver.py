#!/usr/bin/env python
import rospy
from gai_franka_example.srv import ImagePath, ImagePathResponse

def capture_image(req):
    #TODO: save image from camera
    image_path = "%s" % rospy.get_time() + ".png"
    rospy.loginfo("save image" + image_path)
    return ImagePathResponse(image_path)

def capture_image_server():
    rospy.init_node('image_saver')
    s = rospy.Service('capture_image', ImagePath, capture_image)
    print("ready to capture image")
    rospy.spin()


if __name__ == '__main__':
    capture_image_server()