#! /usr/bin/python3

# rospy for the subscriber
import rospy
import numpy as np
# OpenCV2 for saving an image
import cv2
from geometry_msgs.msg import Twist
# ROS Image message
from sensor_msgs.msg import Image

# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError

# Global variables
# Instantiate CvBridge
bridge = CvBridge()
publisher = None


def imageCallback(msg):
    print("Received an image!")

    try:
        # Convert your ROS Image message to OpenCV2
        img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except:
        print('Could not convert image')
        return

    h = img.shape[0]
    w = img.shape[1]
    print('height:', h, '\nwidth:', w, '\nchannel count:', img.shape[2], '\n')
    vertices = [
        (w * 0.1, h),
        (w * 0.35, h * 0.3),
        (w * 0.65, h * 0.3),
        (w, h * 0.8),
        (w, h)
    ]
    mask = np.zeros_like(img)
    match = (255,) * img.shape[2]
    cv2.fillPoly(mask, np.array([vertices], np.int32), match)
    masked = cv2.bitwise_and(img, mask)
    gray = cv2.cvtColor(masked, cv2.COLOR_RGB2GRAY)
    canny = cv2.Canny(gray, threshold1=300, threshold2=300, apertureSize=3, L2gradient=False)

    lines = cv2.HoughLinesP(canny,
                            rho=1,
                            theta=np.pi / 90,
                            threshold=5,
                            lines=np.array([]),
                            minLineLength=1,
                            maxLineGap=1)
    i = 0
    lista = []
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            x = (y2 - y1) / (x2 - x1)
            theta = 180 * np.arctan(x) / np.pi
            if -90 <= theta <= -30 or 30 <= theta <= 90:
                # lista[i] = theta
                cv2.line(img, (x1, y1), (x2, y2), (255, 0, 0), thickness=3)
                # i = i + 1
                # print(i)
                # print(lista)
        # avg_theta = np.mean(lista)

    # x3, y3, x4, y4 = np.mean(line[0])
    # cv2.line(img, (x3, y3), (x4, y4), (255, 0, 255), thickness=3)
    # Save your OpenCV2 image as a jpeg
    cv2.imshow('ROI', canny)
    cv2.imshow('front camera', img)
    cv2.waitKey(20)
    # How to process the image to define the best angle and speed?

    # make a driving decision
    angle = 0
    speed = 0

    # build a twist msg to publish
    twist = Twist()
    twist.linear.x = speed
    twist.angular.z = angle
    global publisher
    # publisher.publish(twist)


def main():
    rospy.init_node('driver')
    # Define your image topic
    image_topic = "/front_camera/rgb/image_raw"
    # Set up your subscriber and define its callback
    rospy.Subscriber(image_topic, Image, imageCallback)
    global publisher
    publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    # Spin until ctrl + c
    rospy.spin()


if __name__ == '__main__':
    main()
