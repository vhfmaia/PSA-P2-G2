#!/usr/bin/env python3

import rospy
import cv2
import os
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

bridge = CvBridge
def image_callback(msg):
    print("Received an image!")
    try:
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError, e:
        print(e)
    else:
        # Save your OpenCV2 image as a jpeg
        cv2.imwrite('camera_image.jpeg', cv2_img)

def camera():
    rospy.init_node("camera_images")
    rate = rospy.Rate(10)
    images = rospy.Publisher("image_Carla", Image, queue_size=10)
    rospy.Subscriber(images, Image, image_callback)

    while not rospy.is_shutdown():
        rate.sleep()



if __name__ == '__main__':
    try:
        camera()
    except rospy.ROSInterruptException:
        pass