#!/usr/bin/env python3

import rospy
from camera_carro import handle_image
from camera_carro import display
from camera_carro import image

def camera():
    rospy.init_node("camera_images")

    loop_rate = rospy.Rate(10)

    images = rospy.Publisher(handle_image(display, image))

    while not rospy.is_shutdown():
        loop_rate.sleep()


if __name__ == '__main__':
    try:
        camera()
    except rospy.ROSInterruptException:
        pass
