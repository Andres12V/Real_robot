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

class Controller:
    def __init__(self):
        rospy.init_node('Sonar_camera')
        while not rospy.is_shutdown():
            # Sonar sensor
            self.pub_laser1 = rospy.Publisher('/my_project/sonar1', LaserScan, queue_size=10)
            self.ls1_m()
            # Sonar sensor 2
            self.pub_laser2 = rospy.Publisher('/my_project/sonar2', Float64, queue_size=10)
            self.ls2_m()
            # Sonar sensor 3
            #self.pub_laser3 = rospy.Publisher('/my_project/sonar3', Float64, queue_size=10)
            #self.ls3_m()
            # Sonar sensor 4
            #self.pub_laser4 = rospy.Publisher('/my_project/sonar4', Float64, queue_size=10)
            #self.ls4_m()



    def ls1_m(self):
        time.sleep(0.1)
        GPIO.output(TRIG,1)
        time.sleep(0.00001)  # 10 ns pulse
        GPIO.output(TRIG,0)
        while GPIO.input(ECHO) ==0:
            pass
        start = time.time()

        while GPIO.input(ECHO) ==1:
            pass
        stop = time.time()
        dist1 = round((stop-start)*170, 4)
        print('Dist1', dist1)
        sonar1_msg = LaserScan()
        sonar1_msg.header.frame_id = S1_FRAME
        sonar1_msg.ranges = [dist1]
        self.pub_laser1.publish(sonar1_msg)
    def ls2_m(self):
        time.sleep(0.1)
        GPIO.output(TRIG2,1)
        time.sleep(0.00001)  # 10 ns pulse
        GPIO.output(TRIG2,0)
        while GPIO.input(ECHO2) ==0:
            pass
        start = time.time()

        while GPIO.input(ECHO2) ==1:
            pass
        stop = time.time()
        dist2 = round((stop-start)*170, 4)
        print('Dist2', dist2)
        self.pub_laser2.publish(dist2)

    def ls3_m(self):
        time.sleep(0.1)
        GPIO.output(TRIG3,1)
        time.sleep(0.00001)  # 10 ns pulse
        GPIO.output(TRIG3,0)
        while GPIO.input(ECHO3) ==0:
            pass
        start = time.time()

        while GPIO.input(ECHO3) ==1:
            pass
        stop = time.time()
        dist3 = round((stop-start)*170, 4)
        print('Dist3', dist3)
        self.pub_laser3.publish(dist3)
    def ls4_m(self):
        time.sleep(0.1)
        GPIO.output(TRIG4,1)
        time.sleep(0.00001)  # 10 ns pulse
        GPIO.output(TRIG4,0)
        while GPIO.input(ECHO4) ==0:
            pass
        start = time.time()

        while GPIO.input(ECHO4) ==1:
            pass
        stop = time.time()
        dist4 = round((stop-start)*170, 4)
        print('Dist4', dist4)
        self.pub_laser4.publish(dist4)




if __name__ == '__main__':

    # GPIO ports Setup
    GPIO.setmode(GPIO.BOARD)
    TRIG = 12
    ECHO = 16
    # sensor 2 con resistencia diferente
    TRIG2 = 7
    ECHO2 = 11
    # sensor 3
    TRIG3 = 18
    ECHO3 = 22

    # sensor 4
    TRIG4 = 13
    ECHO4 = 15



    GPIO.setup(TRIG,GPIO.OUT)
    GPIO.output(TRIG,0)
    GPIO.setup(ECHO,GPIO.IN)

    GPIO.setup(TRIG2,GPIO.OUT)
    GPIO.output(TRIG2,0)
    GPIO.setup(ECHO2,GPIO.IN)

    GPIO.setup(TRIG3,GPIO.OUT)
    GPIO.output(TRIG3,0)
    GPIO.setup(ECHO3,GPIO.IN)

    GPIO.setup(TRIG4,GPIO.OUT)
    GPIO.output(TRIG4,0)
    GPIO.setup(ECHO4,GPIO.IN)
    i=0
    R = 0.065
    L = 0.3
    S1_FRAME = rospy.get_param('~sonar_frame', 'sonar1_link')
    kt = Controller()
    try:
        if not rospy.is_shutdown():
            rospy.spin()
            GPIO.cleanup()

    except rospy.ROSInterruptException as e:
        print(e)
