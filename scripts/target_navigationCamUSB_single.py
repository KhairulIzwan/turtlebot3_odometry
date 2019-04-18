#!/usr/bin/env python

from __future__ import print_function
from __future__ import division

import roslib
roslib.load_manifest('turtlebot3_odometry')

import sys
import rospy
import cv2
import numpy as np
import imutils

from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import RegionOfInterest
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import Twist

from camera_tutorials.msg import IntList
from camera_tutorials.msg import detailROI

from cv_bridge import CvBridge
from cv_bridge import CvBridgeError

import os

class navigation_node:
    def __init__(self):

        """  Initializing your ROS Node """
        rospy.init_node('navigation_node', anonymous=True)

        rospy.on_shutdown(self.shutdown)

        r = rospy.Rate(10) # 10hz

        self.tracking_seq = 0
        self.last_tracking_seq = -1

        """ The pan/tilt thresholds indicate how many pixels the ROI needs to be off-center
            before we make a movement. """
        self.pan_threshold = int(5)
        self.tilt_threshold = int(5)

        """  Subscribe to the green roi topic """
        # self.imgROI_sub = rospy.Subscriber("/roi_Green", detailROI, self.getGreenROI)

        """  Subscribe to the red roi topic """
        self.imgROI_sub = rospy.Subscriber("/roi_Red", detailROI, self.getROI)

        """ Subscribe to the camera info topic """
        self.imgRaw_sub = rospy.Subscriber("/camUSB/camera_info", CameraInfo, self.getCameraInfo)

        self.navi_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    """ Get the width and height of the image """
    def getCameraInfo(self, msg):
        self.image_width = msg.width    # 320
        self.image_height = msg.height  # 240

    """ Getting colored ROI info -- width, height, and etc """
    def getROI(self, msg):
        self.colorName = msg.colorName
        self.offsetX = msg.offsetX
        self.offsetY = msg.offsetY
        self.width = msg.width
        self.height = msg.height
        self.x = msg.x
        self.y = msg.y
        self.radius = msg.radius

        self.roiTrack()

    def roiTrack(self):
        self.spdNavi = Twist()

        """ Check to see if we have lost the ROI. """
        if self.width == 0 or self.height == 0 or self.width > self.image_width / 2 or self.height > self.image_height / 2:
            self.spdNavi.linear.x = 0.0
            self.spdNavi.linear.y = 0.0
            self.spdNavi.linear.z = 0.0

            self.spdNavi.angular.x = 0.0
            self.spdNavi.angular.y = 0.0
            self.spdNavi.angular.z = 0.0

            rospy.logerr("STOP!")
            return

        """ Compute the center of the ROI """
        COG_x = self.offsetX + self.width / 2 - self.image_width / 2
        COG_y = self.offsetY + self.height / 2 - self.image_height / 2

        """ Pan the camera only if the displacement of the COG exceeds the threshold. """
        if abs(COG_x) > self.pan_threshold:
            """ Set the pan speed proportion to the displacement of the horizontal displacement of the target. """
            self.spdNavi.angular.z = 0.5

            """ Set the target position to one of the min or max positions--we'll never get there since we are tracking using speed. """
            if COG_x > 0:
                self.spdNavi.angular.z = -self.spdNavi.angular.z
            else:
                self.spdNavi.angular.z = +self.spdNavi.angular.z
        else:
            self.spdNavi.angular.z = 0


        """ Tilt the camera only if the displacement of the COG exceeds the threshold. """
        if abs(COG_y) > self.tilt_threshold:
            """ Set the tilt speed proportion to the displacement of the vertical displacement of the target. """
            self.spdNavi.linear.x = 0.1

            """ Set the target position to one of the min or max positions--we'll never get there since we are tracking using speed. """
            if COG_y < 0:
                self.spdNavi.linear.x = +self.spdNavi.linear.x
            else:
                # self.head_cmd.position[1] = self.max_tilt
                self.spdNavi.linear.x = -self.spdNavi.linear.x
        else:
            self.spdNavi.linear.x = 0

        rospy.loginfo([COG_x, COG_y, self.spdNavi.linear.x, self.spdNavi.angular.z])
        self.navi_pub.publish(self.spdNavi)

    def shutdown(self):
        try:
            rospy.loginfo("Navigation node [OFFLINE]...")

        finally:
            self.spdNavi = Twist()

            self.spdNavi.linear.x = 0.0
            self.spdNavi.linear.y = 0.0
            self.spdNavi.linear.z = 0.0

            self.spdNavi.angular.x = 0.0
            self.spdNavi.angular.y = 0.0
            self.spdNavi.angular.z = 0.0

            self.navi_pub.publish(self.spdNavi)

def usage():
    print("%s" % sys.argv[0])

def main(args):
    vn = navigation_node()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Navigation node [OFFLINE]...")

if __name__ == '__main__':
    if len(sys.argv) < 1:
        print(usage())
        sys.exit(1)
    else:
        print("Navigation node [ONLINE]...")
        main(sys.argv)
