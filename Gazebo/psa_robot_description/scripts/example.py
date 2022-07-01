#! /usr/bin/python3

# rospy for the subscriber
import math
import threading
import ctypes
import struct
from email import header
from copy import deepcopy
from re import I
from xml.etree.ElementTree import PI

import numpy as np
import rospy

# OpenCV2 for saving an image
import cv2
import sensor_msgs.point_cloud2 as pc2
from colorama import Fore, Style
from vertical_stacking import verticalStacking
from line_tracker import LineTracker
from geometry_msgs.msg import Twist, Point

# ROS Image message
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import String, Header

# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
from simple_pid import PID
from visualization_msgs.msg import Marker, MarkerArray


class Driver():

    def __init__(self):
        rospy.init_node('driver')

        # Define your image topic
        image_topic = "/front_camera/rgb/image_raw"

        self.M = np.load('/home/mike/catkin_ws/src/PSA-P2-21-22/psa_robot_description/scripts/ipm_file.npy')

        rospy.sleep(1)
        self.groups = None
        self.previous_angle = 0

        self.reference_y = 260
        self.tracker_ll = LineTracker(50, self.reference_y, 'LT')
        self.tracker_ml = LineTracker(50, self.reference_y, 'MT')
        self.tracker_rl = LineTracker(50, self.reference_y, 'RT')

        self.image_gui = None
        self.image_stacked = None
        self.bridge = CvBridge()
        self.lane_width = 250  # in pixels, on the ipm image.
        self.driving_lane = 'MIDDLE'  # can be 'MIDDLE', 'LEFT' and 'RIGHT'

        self.errors = [0]*5

        image_msg = rospy.wait_for_message(image_topic, Image)
        try:
            # Convert your ROS Image message to OpenCV2
            image_rgb = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        except:
            print('Could not convert image')

        h, w, _ = image_rgb.shape

        kp = 0.003
        ki = 0.0001
        kd = 0.001
        self.pid = PID(kp, ki, kd, setpoint=w/2)

        self.ground_threshold = -0.25
        # Set up your subscriber and define its callback
        self.subscriber = rospy.Subscriber(image_topic, Image, self.imageCallback)
        self.subscriber_drive_lane = rospy.Subscriber('drive_lane', String, self.driveLaneCallback)
        self.cloud_sub = rospy.Subscriber("/lidar3d/points", PointCloud2,
                                          self.pointCloudCallback, queue_size=1, buff_size=52428800)

        self.publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.publisher_visualization = rospy.Publisher('Markers', MarkerArray, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.decisionCallback)

    def pointCloudCallback(self, ros_point_cloud):
        print('Received point cloud')
        # xyz = np.array([[0, 0, 0]])
        xyz = []
        # rgb = np.array([[0, 0, 0]])
        # self.lock.acquire()
        gen = pc2.read_points(ros_point_cloud, skip_nans=True)
        int_data = list(gen)

        for x in int_data:
            test = x[3]
            # cast float32 to int so that bitwise operations are possible
            s = struct.pack('>f', test)
            i = struct.unpack('>l', s)[0]
            # you can get back the float value by the inverse operations
            pack = ctypes.c_uint32(i).value
            r = (pack & 0x00FF0000) >> 16
            g = (pack & 0x0000FF00) >> 8
            b = (pack & 0x000000FF)
            # prints r,g,b values in the 0-255 range
            # x,y,z can be retrieved from the x[0],x[1],x[2]

            if x[2] > self.ground_threshold:
                # xyz = np.append(xyz, [[x[0], x[1], x[2]]], axis=0)
                # rgb = np.append(rgb, [[r, g, b]], axis=0)
                xyz.append((x[0], x[1], x[2]))

        print('xyz len ' + str(len(xyz)))
        # print(xyz)

        ma = MarkerArray()
        m = Marker(header=Header(stamp=rospy.Time.now(), frame_id=ros_point_cloud.header.frame_id))
        m.ns = 'markers'
        m.id = 0
        m.type = Marker.POINTS
        m.action = Marker.ADD
        m.pose.orientation.w = 1
        m.scale.x = 0.05
        m.scale.y = 0.05
        m.color.r = 1
        m.color.g = 0
        m.color.b = 0
        m.color.a = 1

        for point in xyz:
            x = point[0]
            y = point[1]
            z = point[2]

            p = Point()
            p.x = x
            p.y = y
            p.z = z
            m.points.append(p)

        ma.markers.append(m)
        self.publisher_visualization.publish(ma)

    def driveLaneCallback(self, msg):
        if msg.data not in ['LEFT', 'MIDDLE', 'RIGHT']:
            print('Incorrect lane to drive ... keeping on ' + self.driving_lane)
        else:
            self.driving_lane = msg.data
            print('Changing driving lane to ' + self.driving_lane)

    def decisionCallback(self, msg):

        print('Decision callback')
        if self.image_gui is None or self.image_stacked is None:
            return

        height, width, _ = self.image_gui.shape

        # -------------------------------------------
        # Find new lines by searching or using other lines
        # -------------------------------------------
        if self.tracker_ml.state == 'LOST' and self.tracker_ll.state == 'LOST' and self.tracker_rl.state == 'LOST':
            # No line is tracked, must search for lines
            middle_line, left_line, right_line = self.searchLines(self.image_stacked, self.image_gui)

            if self.tracker_ml.state == 'LOST' and middle_line != None:
                self.tracker_ml.restart(middle_line['xavg'])

            if self.tracker_ll.state == 'LOST' and left_line != None:
                self.tracker_ll.restart(left_line['xavg'])

            if self.tracker_rl.state == 'LOST' and right_line != None:
                self.tracker_rl.restart(right_line['xavg'])
        else:

            if self.tracker_ll.state == 'LOST':
                if self.tracker_ml.state == 'TRACKING':  # restart ll tracker using ml tracker
                    self.tracker_ll.restart(self.tracker_ml.x - self.lane_width)
                elif self.tracker_rl.state == 'TRACKING':  # restart ll tracker using rl tracker
                    self.tracker_ll.restart(self.tracker_rl.x - 2*self.lane_width)

            if self.tracker_ml.state == 'LOST':
                if self.tracker_ll.state == 'TRACKING':  # restart ml tracker using ll tracker
                    self.tracker_ml.restart(self.tracker_ll.x + self.lane_width)
                elif self.tracker_rl.state == 'TRACKING':  # restart ml tracker using rl tracker
                    self.tracker_ml.restart(self.tracker_rl.x - self.lane_width)

            if self.tracker_rl.state == 'LOST':
                if self.tracker_ml.state == 'TRACKING':  # restart rl tracker using ml tracker
                    self.tracker_rl.restart(self.tracker_ml.x + self.lane_width)
                elif self.tracker_ll.state == 'TRACKING':  # restart rl tracker using ll tracker
                    self.tracker_rl.restart(self.tracker_ll.x + 2*self.lane_width)

        # -------------------------------------------
        # Update tracking
        # -------------------------------------------
        self.tracker_ml.track(self.image_stacked, self.image_gui)
        self.tracker_ll.track(self.image_stacked, self.image_gui)
        self.tracker_rl.track(self.image_stacked, self.image_gui)

        # -------------------------------------------
        # Controller (make a driving decision)
        # -------------------------------------------
        # decide which line to follow, from the available ones

        cv2.putText(self.image_gui, 'Driving on ' + self.driving_lane + ' lane', (10, 60),
                    cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 255), 1, cv2.LINE_AA)

        if self.tracker_ml.state == 'TRACKING' and \
                self.tracker_ml.time_tracking >= self.tracker_ll.time_tracking and \
                self.tracker_ml.time_tracking >= self.tracker_rl.time_tracking:

            cv2.putText(self.image_gui, 'Following middle line', (10, 30),
                        cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 255), 1, cv2.LINE_AA)
            go_to_x = self.getGoTox(self.tracker_ml.x, 'MIDDLE', self.image_gui)

        elif self.tracker_ll.state == 'TRACKING' and \
                self.tracker_ll.time_tracking >= self.tracker_ml.time_tracking and \
                self.tracker_ll.time_tracking >= self.tracker_rl.time_tracking:

            cv2.putText(self.image_gui, 'Following left line', (10, 30),
                        cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 255), 1, cv2.LINE_AA)
            go_to_x = self.getGoTox(self.tracker_ll.x, 'LEFT', self.image_gui)

        elif self.tracker_rl.state == 'TRACKING' and \
                self.tracker_rl.time_tracking >= self.tracker_ll.time_tracking and \
                self.tracker_rl.time_tracking >= self.tracker_ml.time_tracking:

            cv2.putText(self.image_gui, 'Following right line', (10, 30),
                        cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 255), 1, cv2.LINE_AA)
            go_to_x = self.getGoTox(self.tracker_rl.x, 'RIGHT', self.image_gui)
        else:
            print('No tracker available ...')
            go_to_x = None

        if go_to_x is not None:
            angle = self.pid(go_to_x)
        else:
            angle = self.previous_angle

        # print("Errors=" + str(self.errors))
        print("Decision angle=" + str(angle))
        self.previous_angle = angle
        speed = 0.3

        # build a twist msg to publish
        twist = Twist()
        twist.linear.x = speed
        twist.angular.z = angle
        # self.publisher.publish(twist)

        cv2.putText(
            self.image_gui, 'Angle' + str(round(angle*180/math.pi, 2)), (10, 150),
            cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 255),
            1, cv2.LINE_AA)

        cv2.imshow('image_gui', self.image_gui)
        key = cv2.waitKey(20)
        print(key)
        if key == ord('q'):
            self.shutdown()
        elif key == ord('.'):
            if self.driving_lane == 'MIDDLE':
                self.driving_lane = 'RIGHT'
            elif self.driving_lane == 'LEFT':
                self.driving_lane = 'MIDDLE'
        elif key == ord(','):
            if self.driving_lane == 'MIDDLE':
                self.driving_lane = 'LEFT'
            elif self.driving_lane == 'RIGHT':
                self.driving_lane = 'MIDDLE'

        print("Decision finished")

    def getGoTox(self, x, followed_line, image_gui):
        if followed_line not in ['LEFT', 'MIDDLE', 'RIGHT']:
            print('Error, unknown line')
            return None

        if self.driving_lane == 'MIDDLE':
            if followed_line == 'MIDDLE':
                go_to_x = x
            elif followed_line == 'LEFT':
                go_to_x = x + self.lane_width
            elif followed_line == 'RIGHT':
                go_to_x = x - self.lane_width
        elif self.driving_lane == 'LEFT':
            if followed_line == 'MIDDLE':
                go_to_x = x - 0.5 * self.lane_width
            elif followed_line == 'LEFT':
                go_to_x = x + 0.5 * self.lane_width
            elif followed_line == 'RIGHT':
                go_to_x = x - 1.5 * self.lane_width
        elif self.driving_lane == 'RIGHT':
            if followed_line == 'MIDDLE':
                go_to_x = x + 0.5 * self.lane_width
            elif followed_line == 'LEFT':
                go_to_x = x + 1.5 * self.lane_width
            elif followed_line == 'RIGHT':
                go_to_x = x - 0.5 * self.lane_width

        cv2.line(image_gui, (int(go_to_x - 10), self.reference_y),
                 (int(go_to_x + 10), self.reference_y), (0, 0, 255), 4)
        cv2.line(image_gui, (int(go_to_x), self.reference_y - 10),
                 (int(go_to_x), self.reference_y + 10), (0, 0, 255), 4)

        return go_to_x

    def imageCallback(self, msg):

        try:
            # Convert your ROS Image message to OpenCV2
            image_rgb = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except:
            print('Could not convert image')
            return

        # Preprocessing
        image_rgb_ipm = cv2.warpPerspective(image_rgb, self.M, [900, 320])
        self.image_gui = deepcopy(image_rgb_ipm)

        image_gray = cv2.cvtColor(image_rgb_ipm, cv2.COLOR_BGR2GRAY)  # convert to grayscale
        _, image_thresh = cv2.threshold(image_gray, 127, 255, cv2.THRESH_BINARY)  # thresholding

        # How to process the image to define the best angle and speed?
        reference_y = 260
        reference_y_delta = 20
        height, width = image_thresh.shape
        minimum_number_white_pixels = 5

        self.image_stacked = verticalStacking(image=image_thresh,
                                              y_limits=[reference_y-reference_y_delta, reference_y + reference_y_delta])

    def searchLines(self, image_stacked, image_gui, draw=True):
        """Assumes camera is centered on the road
        """

        # search for white pixels in stacked image from left to right
        height, width, _ = image_gui.shape
        reference_y = 260
        reference_y_delta = 20
        middle_x = int(width / 2)
        white_xs = []
        minimum_number_white_pixels = 5
        for x in range(0, width):
            if image_stacked[x] > minimum_number_white_pixels:
                white_xs.append(x)

        groups = []
        middle_line = None
        left_line = None
        right_line = None

        # iterate through white_xs and create groups
        first = True
        group_idx = 0
        for x in white_xs:
            if first:
                group = {'idx': group_idx, 'xs': [x]}
                groups.append(group)
                group_idx += 1
                first = False
                continue

            # decide if a new group should be create
            last_x = groups[-1]['xs'][-1]
            if abs(x - last_x) > 1:  # create new group
                group = {'idx': group_idx, 'xs': [x]}
                groups.append(group)
                group_idx += 1
            else:
                groups[-1]['xs'].append(x)

        # Compute the average x for each group
        for group in groups:
            group['xavg'] = sum(group['xs']) / len(group['xs'])

        # Compute the distance between the average and the middle x
        for group in groups:
            group['dist_to_middle'] = abs(middle_x - group['xavg'])

        # select middle line as the group which is closed to the middle of the image
        smallest_distance = 9999
        for group in groups:
            if group['dist_to_middle'] < smallest_distance:
                smallest_distance = group['dist_to_middle']
                middle_line = group

        if not middle_line is None:
            # Compute the distance between the middle line and avgx
            for group in groups:
                group['dist_to_middle_line'] = middle_line['xavg'] - group['xavg']

            # Find the left line a the first on the left of the middle line
            smallest_distance = 9999
            for group in groups:
                if group['dist_to_middle_line'] <= 0:
                    continue
                elif group['dist_to_middle_line'] < smallest_distance:
                    smallest_distance = group['dist_to_middle_line']
                    left_line = group

            # Find the right line a the first on the right of the middle line
            smallest_distance = 9999
            for group in groups:
                if group['dist_to_middle_line'] >= 0:
                    continue
                elif abs(group['dist_to_middle_line']) < smallest_distance:
                    smallest_distance = group['dist_to_middle_line']
                    right_line = group

        # -------------------------------------------
        # Drawing
        # -------------------------------------------
        if draw:
            for x in white_xs:
                cv2.line(image_gui, (x, reference_y+10), (x, reference_y+10), (0, 0, 255), 4)

            if not middle_line == None:
                point = (int(middle_line['xavg']), reference_y+10)
                color = (0, 255, 255)
                cv2.line(image_gui, point, point, color, 4)
                cv2.putText(image_gui, 'ML', point, cv2.FONT_HERSHEY_SIMPLEX, 1, color, 1, cv2.LINE_AA)

            if not left_line == None:
                print('Drawing left line')
                point = (int(left_line['xavg']), reference_y+10)
                cv2.putText(image_gui, 'LL', point, cv2.FONT_HERSHEY_SIMPLEX, 1, color, 1, cv2.LINE_AA)

        print('return')
        return middle_line, left_line, right_line

    def shutdown(self):

        print(Fore.RED + 'Shutting down' + Style.RESET_ALL)
        twist = Twist()  # build a twist msg to make sure the robot is stopped
        twist.linear.x = 0
        twist.angular.z = 0
        self.publisher.publish(twist)

        rospy.signal_shutdown("Because I want to")


def main():

    driver = Driver()

    rospy.spin()

#     rate = rospy.Rate(10)
#     while not rospy.is_shutdown():
#
#         rate.sleep()
#
#         if driver.image_gui is not None:
#             cv2.imshow(driver.image_gui)
#         cv2.waitKey(50)


if __name__ == '__main__':
    main()