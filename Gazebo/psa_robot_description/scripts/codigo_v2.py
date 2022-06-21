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

global time_cross
time_cross = 0


def imageCallback(msg):
    try:
        # Convert your ROS Image message to OpenCV2
        img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except:
        print('Could not convert image')
        return

    # Get window size
    h = img.shape[0]
    w = img.shape[1]

    # Define transform points
    # Length of the top line must be updated if the camera changes (0 -> 1)
    top_len = 0.25

    points = [
        [w * (0.5 - top_len/2), h * 0.4],
        [w * (0.5 + top_len/2), h * 0.4],
        [0, h],
        [w, h]]

    # Draw lines that connect the transform points
    lines = [[0, 1], [0, 2], [1, 3], [2, 3]]
    img2 = img.copy()
    for i in range(len(lines)):
        point_1 = lines[i][0]
        point_2 = lines[i][1]
        point_x1 = int(points[point_1][0])
        point_y1 = int(points[point_1][1])
        point_x2 = int(points[point_2][0])
        point_y2 = int(points[point_2][1])
        img2 = cv2.line(img2, (point_x1, point_y1), (point_x2, point_y2), (0, 255, 0), 1)

    # Display img2: Original Image + Transform region lines
    cv2.imshow("Front Camera", img2)

    # Transform original image
    points_first = np.float32(points)
    points_second = np.float32([[0, 0], [w, 0], [0, h], [w, h]])
    matrix = cv2.getPerspectiveTransform(points_first, points_second)
    img3 = cv2.warpPerspective(img, matrix, (w, h))

    # Region of Interest: lower zone of the camera, region height (0 -> 1)
    region_height = 0.1

    vertices = [
        (0, h),
        (0, h * (1 - region_height)),
        (w, h * (1 - region_height)),
        (w, h)]

    # Canny + Hough Lines
    mask = np.zeros_like(img3)
    match = (255,) * img3.shape[2]
    cv2.fillPoly(mask, np.array([vertices], np.int32), match)
    masked = cv2.bitwise_and(img3, mask)
    gray = cv2.cvtColor(masked, cv2.COLOR_RGB2GRAY)
    (thresh, black_white) = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)
    img4 = cv2.Canny(black_white, threshold1=300, threshold2=300, apertureSize=3, L2gradient=False)

    lines = cv2.HoughLinesP(img4,
                            rho=1,
                            theta=np.pi / 90,
                            threshold=5,
                            lines=np.array([]),
                            minLineLength=7,
                            maxLineGap=1)

    # Select lines with valid angles
    max_theta = 70

    lista = []
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            if y2 - y1 != 0:
                theta = 180 * np.arctan((x2 - x1) / (y2 - y1)) / np.pi

                # If angle is valid -> save line and draw green / else: discard and draw red (except border lines)
                if abs(theta) < max_theta:
                    lista.append(theta)
                    cv2.line(img3, (x1, y1), (x2, y2), (0, 150, 0), 3)
                elif abs(theta) < 85:
                    cv2.line(img3, (x1, y1), (x2, y2), (0, 0, 100), 3)

    # Get average of valid angles
    avg_theta = 0
    avg_color = (150, 0, 0)
    if len(lista) > 1:
        avg_theta = np.mean(lista)

        # # V situations: if the angles are too different, turn
        # if max(lista) - min(lista) > 80:
        #     avg_color = (0, 120, 220)
        #     avg_theta = -80

    # Display average angle with a line
    if 0 < abs(avg_theta) < 90:
        x_diff = int(h / np.tan((90-avg_theta) * np.pi / 180))
    else:
        x_diff = 0

    cv2.line(img3, (int(w/2), h), (int(w/2) - x_diff, 0), avg_color, 2)

    # Display img3: transformed image + hough lines + average angle line
    cv2.imshow('Lines', img3)
    cv2.waitKey(20)

    # Change driving angle depending on average line angle
    min_theta = 5

    if abs(avg_theta) > min_theta:
        angle = 0.005 * avg_theta
    else:
        angle = 0

    # Detect crosswalk
    global time_cross
    time_diff = rospy.get_rostime().secs - time_cross

    line_count = 0
    if time_diff > 15:
        if lines is not None:
            line_count = len(lines)

        if line_count > 40:
            time_cross = rospy.get_rostime().secs

    # Speed: stop on crosswalk / slow down on curves
    if time_diff < 5:
        speed = 0
        print("Crosswalk: " + str(5 - time_diff))

    else:
        speed = 0.05 + 0.2 * (1 - abs(avg_theta) / 90)

    print("Lines: " + str(line_count) + " -- Angle: " + str(round(avg_theta)) + "º -- Speed: " + str(round(speed, 2)))

    # ROS
    # Build a twist msg to publish
    twist = Twist()
    twist.linear.x = speed
    twist.angular.z = angle
    global publisher
    publisher.publish(twist)


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
