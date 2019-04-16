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

from camera_tutorials.msg import IntList
from camera_tutorials.msg import detailROI

from cv_bridge import CvBridge
from cv_bridge import CvBridgeError

import os

class navigation_node:
    def __init__(self):

        """  Initializing your ROS Node """
        rospy.init_node('navigation_node', anonymous=True)

        """  Subscribe to the green roi topic """
        # self.imgROI_sub = rospy.Subscriber("/roi_Green", detailROI, self.getGreenROI)

        """  Subscribe to the red roi topic """
        self.imgROI_sub = rospy.Subscriber("/roi_Red", detailROI, self.getRedROI)

    """ Getting green colored ROI info -- width, height, and etc """
    def getGreenROI(self, msg):
        self.colorName_Green = msg.colorName
        self.offsetX_Green = msg.offsetX
        self.offsetY_Green = msg.offsetY
        self.width_Green = msg.width
        self.height_Green = msg.height
        self.x_Green = msg.x
        self.y_Green = msg.y
        self.radius_Green = msg.radius

        self.ballGreenTrack()

    """ Getting green colored ROI info -- width, height, and etc """
    def getRedROI(self, msg):
        self.colorName_Red = msg.colorName
        self.offsetX_Red = msg.offsetX
        self.offsetY_Red = msg.offsetY
        self.width_Red = msg.width
        self.height_Red = msg.height
        self.x_Red = msg.x
        self.y_Red = msg.y
        self.radius_Red = msg.radius

        self.ballRedTrack()

    def ballGreenTrack(self):
        if self.colorName_Green is not None:
            rospy.loginfo(self.colorName_Green)

    def ballRedTrack(self):
        if self.colorName_Red is not None:
            rospy.loginfo(self.colorName_Red)
        else:
            rospy.loginfo("Empty")

def usage():
    print("%s" % sys.argv[0])

def main(args):
    vn = navigation_node()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Navigation node [OFFLINE]...")

    cv2.destroyAllWindows()

if __name__ == '__main__':
    if len(sys.argv) < 1:
        print(usage())
        sys.exit(1)
    else:
        print("Navigation node [ONLINE]...")
        main(sys.argv)
