#!/usr/bin/env python2

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
from cv_bridge import CvBridge, CvBridgeError

import cv2 as cv

#import cv2.aruco as aruco
import numpy as np
import sys, time, math
import matplotlib.pyplot as plt

import RPi.GPIO as GPIO
import time
from time import sleep

from picamera.array import PiRGBArray
from picamera import PiCamera

class Camera_c:
    def __init__(self):
        rospy.init_node('Sonar_camera')
        while not rospy.is_shutdown():
            self.bridge = CvBridge()
            self.pub_camera = rospy.Publisher('/my_project/camera', Image , queue_size=10)
            self.cam_m()
    def cam_m(self):
        time.sleep(0.01)
        for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
            image = frame.array
            #cv.imshow('aruco_detection', image)
            rawCapture.truncate(0)
            image_message = self.bridge.cv2_to_imgmsg(image,"bgr8")
            self.pub_camera.publish(image_message)
            #cv.waitKey(1)
            break



if __name__ == '__main__':
    """ Pi_Camera setup """
    camera = PiCamera()
    camera.resolution = (640, 480)
    camera.framerate = 40
    rawCapture = PiRGBArray(camera, size=(640, 480))
    kt = Camera_c()
    try:
        if not rospy.is_shutdown():
            rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)

