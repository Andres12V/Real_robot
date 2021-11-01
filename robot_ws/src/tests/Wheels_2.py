#!/usr/bin/env python2
import matplotlib.pyplot as plt

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
from cv_bridge import CvBridge, CvBridgeError
# from KalmanFilter import X

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

class Motors_Controller:
    def __init__(self):
        rospy.init_node('Wheels')
        while not rospy.is_shutdown():
            ## Motor 1
            start_t = time.time()
            self.sub_vel_m1 = rospy.Subscriber('/my_project/m1_velocity_ref', Float64, self.ref1_callback)
            self.pub_vel_m1 = rospy.Publisher('/my_project/m1_velocity', Float64 , queue_size=10)
            self.pos_w1 = rospy.Publisher('/my_project/w1_position', Float64 , queue_size=10)
            self.m1_m()
            ## Motor 2
            self.sub_vel_m2 = rospy.Subscriber('/my_project/m2_velocity_ref', Float64, self.ref2_callback)
            self.pub_vel_m2 = rospy.Publisher('/my_project/m2_velocity', Float64 , queue_size=10)
            self.pos_w2 = rospy.Publisher('/my_project/w2_position', Float64 , queue_size=10)
            self.pub_odom = rospy.Publisher('/my_project/odom', Float64 , queue_size=10)
            self.pub_orientation_encoder = rospy.Publisher('/my_project/orientation_encoder', Float64 , queue_size=10)
            self.m2_m()
            stop_t = time.time()
            print('----------------;', stop_t-start_t)
            if int((stop_t-start_t))<0.05:
                my_T_needed=0.05-int((stop_t-start_t))
            else:
                my_T_needed=0
            time.sleep(my_T_needed)


    def m1_m(self):
        global t,ref_1,int_e_1,PPR,pos_m_1
        global x1e_m1,x2e_m1,x3e_m1,xc_m1
        global x_c_1, x1_p_1, x2_p_1, x3_p_1

        Pulses_1 = iA+iB
        print('The pulses_____1', Pulses_1)
        Turns_1 = Pulses_1/PPR
        Pos_rad_1 = Turns_1*2*np.pi
        Pos_radf.append(Pos_rad_1)
        RPM_1=(179*2*60)/PPR
        # try:
        # velocity_1 = ( (Pos_rad_1-Pos_radf[-2])/0.05 )
        """ Velocity estimation """
        try:
            velocity_1=x2e_m1
        except:
            velocity_1=0
        print('Velocity 1 rad/s', velocity_1)
        """ Error: """
        # if ref_1>=0:
        #     e_1 = ref_1-velocity_1
        # if ref_1<0:
        #     e_1 = velocity_1-ref_1
        e_1 = ref_1-velocity_1
        """ Virtual Plant  """
        try:
            vel_v_1 = x2_p_1
        except:
            vel_v_1 = 0
        e_v_1 = ref_1 - vel_v_1
        try:
            xc_f_1 = x_c_1+e_v_1
        except:
            xc_f_1 = e_v_1
        try:
            u_p_1=kc*x_c_1-k1p*x2_p_1-k2p*x3_p_1
        except:
            u_p_1=0
        try:
            x1p_f_1 = 1*x1_p_1+0.0342*x2_p_1+0.0413*x3_p_1+0.0003*u_p_1
            x2p_f_1 = 0*x1_p_1+0.4096*x2_p_1+1.2719*x3_p_1+0.0173*u_p_1
            x3p_f_1 = 0*x1_p_1-0.0688*x2_p_1+0.4560*x3_p_1+0.0150*u_p_1
        except:
            x1p_f = 0.0003*u_p_1
            x2p_f = 0.0173*u_p_1
            x3p_f = 0.0150*u_p_1
        x1_p_1 = x1p_f_1
        x2_p_1 = x2p_f_1
        x3_p_1 = x3p_f_1
        x_c_1 = xc_f_1
        # print("-----", x_c_1, x1_p_1, x2_p_1, x3_p_1)
        #################
        """ Controller """
        try:
            xcf_m1=xc_m1+e_1
        except:
            xcf_m1=e_1
        """ Cotrol law """
        try:
            #u_1=kc*xc_m1-k1p*x2e_m1-k2p*x3e_m1
            u_1=kc*x_c_1-k1p*x2_p_1-k2p*x3_p_1
        except:
            u_1 = 0
        """ Observer """
        try:
            x1f_m1=-0.4288*x1e_m1+0.0342*x2e_m1+0.0413*x3e_m1+0.0003*u_1+1.4288*Pos_rad_1
            x2f_m1=-4.8746*x1e_m1+0.4096*x2e_m1+1.2719*x3e_m1+0.0173*u_1+4.8746*Pos_rad_1
            x3f_m1=1.0529*x1e_m1-0.0688*x2e_m1+0.4560*x3e_m1+0.0150*u_1-1.0529*Pos_rad_1
        except:
            x1f_m1=0.0003*u_1+1.4288*Pos_rad_1
            x2f_m1=0.0173*u_1+4.8746*Pos_rad_1
            x3f_m1=0.0150*u_1-1.0529*Pos_rad_1
        u_11=np.abs(u_1)
        """ Saturation """
        if u_11>=50:
            u_11=50
        if u_11<=0:
            u_11=0

        x1e_m1=x1f_m1
        x2e_m1=x2f_m1
        print('x2e',x2e_m1)
        x3e_m1=x3f_m1
        xc_m1=xcf_m1

        if ref_1>0.05:
            M1R.stop()
            M1R.ChangeDutyCycle(0)
            M1L.start(0)
            M1L.ChangeDutyCycle(u_11)
        if ref_1<-0.05:
            M1L.stop()
            M1L.ChangeDutyCycle(0)
            M1R.start(0)
            M1R.ChangeDutyCycle(u_11)
        if  ref_1>=-0.05 and ref_1<=0.05:
            M1L.stop()
            M1L.ChangeDutyCycle(0)
            M1R.stop()
            M1R.ChangeDutyCycle(0)
        # pos.append(Pos_rad_1)
        ref1_vec.append(ref_1)
        u1_vec.append(u_1)
        vel_1.append(velocity_1)
        my_time.append(t)
        pos_m_1 = Pos_rad_1*0.035
        self.pub_vel_m1.publish(velocity_1)
        #self.pub_vel_m1.publish(x2_p_1)
        self.pos_w1.publish(pos_m_1)

    def m2_m(self):
        global t,ref_2,PPR
        global x1e_m2,x2e_m2,x3e_m2,xc_m2
        global x_c_2, x1_p_2, x2_p_2, x3_p_2

        Pulses_2 = iA2+iB2
        print('The pulses', Pulses_2)
        Turns_2 = Pulses_2/PPR
        Pos_rad_2 = Turns_2*2*np.pi
        Pos_radf2.append(Pos_rad_2)
        RPM_2=(179*2*60)/PPR
        try:
            velocity_2=x2e_m2
        except:
            velocity_2=0
        # velocity_2 = ( (Pos_rad_2-Pos_radf2[-2])/0.05 )
        print('Velocity 2 rad/s', velocity_2)
        """ Error: """
        # if ref_2>=0:
        e_2 = ref_2-velocity_2
        """ Virtual Plant  """
        try:
            vel_v_2 = x2_p_2
        except:
            vel_v_2 = 0
        e_v_2 = ref_2 - vel_v_2
        try:
            xc_f_2 = x_c_2+e_v_2
        except:
            xc_f_2 = e_v_2
        try:
            u_p_2=kc*x_c_2-k1p*x2_p_2-k2p*x3_p_2
        except:
            u_p_2=0
        try:
            x1p_f_2 = 1*x1_p_2+0.0342*x2_p_2+0.0413*x3_p_2+0.0003*u_p_2
            x2p_f_2 = 0*x1_p_2+0.4096*x2_p_2+1.2719*x3_p_2+0.0173*u_p_2
            x3p_f_2 = 0*x1_p_2-0.0688*x2_p_2+0.4560*x3_p_2+0.0150*u_p_2
        except:
            x1p_f_2 = 0.0003*u_p_2
            x2p_f_2 = 0.0173*u_p_2
            x3p_f_2 = 0.0150*u_p_2
        x1_p_2 = x1p_f_2
        x2_p_2 = x2p_f_2
        x3_p_2 = x3p_f_2
        x_c_2 = xc_f_2
        """ Controller """
        try:
            xcf_m2=xc_m2+e_2
        except:
            xcf_m2=e_2
        """ Cotrol law """
        try:
            #u_2=kc*xc_m2-k1p*x2e_m2-k2p*x3e_m2
            u_2=kc*x_c_2-k1p*x2_p_2-k2p*x3_p_2
        except:
            u_2=0
        """ Observer """
        try:
            x1f_m2=-0.4288*x1e_m2+0.0342*x2e_m2+0.0413*x3e_m2+0.0003*u_2+1.4288*Pos_rad_2
            x2f_m2=-4.8746*x1e_m2+0.4096*x2e_m2+1.2719*x3e_m2+0.0173*u_2+4.8746*Pos_rad_2
            x3f_m2=1.0529*x1e_m2-0.0688*x2e_m2+0.4560*x3e_m2+0.0150*u_2-1.0529*Pos_rad_2
        except:
            x1f_m2=0.0003*u_2+1.4288*Pos_rad_2
            x2f_m2=0.0173*u_2+4.8746*Pos_rad_2
            x3f_m2=0.0150*u_2-1.0529*Pos_rad_2
        u_22=np.abs(u_2)
        """ Saturation """
        if u_22>=50:
            u_22=50
        if u_22<=0:
            u_22=0
        x1e_m2=x1f_m2
        x2e_m2=x2f_m2
        x3e_m2=x3f_m2
        xc_m2=xcf_m2

        if ref_2>0.05:
            M2R.stop()
            M2R.ChangeDutyCycle(0)
            M2L.start(0)
            M2L.ChangeDutyCycle(u_22)
        if ref_2<-0.05:
            M2L.stop()
            M2L.ChangeDutyCycle(0)
            M2R.start(0)
            M2R.ChangeDutyCycle(u_22)
        if ref_2>=-0.05 and ref_2<=0.05:
            M2L.stop()
            M2L.ChangeDutyCycle(0)
            M2R.stop()
            M2R.ChangeDutyCycle(0)

        # pos.append(Pos_rad_2)
        ref2_vec.append(ref_2)
        u2_vec.append(u_2)
        vel_2.append(velocity_2)
        #my_time.append(t)
        pos_m_2 = -(Pos_rad_2*0.035)
        """ Odometry """
        L=0.3
        odom=(pos_m_1+pos_m_2)/2
        orientation=(pos_m_2-pos_m_1)/L
        self.pub_vel_m2.publish(velocity_2)
        #self.pub_vel_m2.publish(x2_p_2)
        self.pos_w2.publish(pos_m_2)
        self.pub_odom.publish(odom)
        self.pub_orientation_encoder.publish(orientation)

        t=t+0.05


    def ref1_callback(self, msg_ref):
        global ref_1
        try:
            ref_1 = msg_ref.data

            #print(ref)
        except:
            ref_1 = 0.0

    def ref2_callback(self, msg_ref2):
        global ref_2
        try:
            ref_2 = msg_ref2.data
            #print(ref)
        except:
            ref_2 = 0.0


if __name__ == '__main__':

    GPIO.setmode(GPIO.BOARD)

    # Wheel 1
    L_En = 26
    R_En = 24
    Enc_A = 32
    Enc_B = 36
    L_pwm = 40
    R_pwm = 38

    GPIO.setup(L_En, GPIO.OUT)
    GPIO.setup(R_En, GPIO.OUT)
    GPIO.setup(Enc_A, GPIO.IN)
    GPIO.setup(Enc_B, GPIO.IN)
    GPIO.setup(L_pwm, GPIO.OUT)
    GPIO.setup(R_pwm, GPIO.OUT)

    # Wheel 2
    L_En2 = 21
    R_En2 = 19
    L_pwm2 = 33
    R_pwm2 = 31
    Enc_A2 = 23
    Enc_B2 = 29


    GPIO.setup(L_En2, GPIO.OUT)
    GPIO.setup(R_En2, GPIO.OUT)
    GPIO.setup(Enc_A2, GPIO.IN)
    GPIO.setup(Enc_B2, GPIO.IN)
    GPIO.setup(L_pwm2, GPIO.OUT)
    GPIO.setup(R_pwm2, GPIO.OUT)

    GPIO.output(L_En,1)
    GPIO.output(R_En,1)
    GPIO.output(L_En2,1)
    GPIO.output(R_En2,1)

    M1L = GPIO.PWM(L_pwm, 4000)
    M1R = GPIO.PWM(R_pwm, 4000)
    M2L = GPIO.PWM(L_pwm2, 4000)
    M2R = GPIO.PWM(R_pwm2, 4000)

    M1L.start(0)
    M1R.start(0)
    M2L.start(0)
    M2R.start(0)

    x1e_m1=0
    x2e_m1=0
    x3e_m1=0
    xc_m1=0

    x1e_m2=0
    x2e_m2=0
    x3e_m2=0
    xc_m2=0

    x_c_1, x1_p_1, x2_p_1, x3_p_1 = 0,0,0,0
    x_c_2, x1_p_2, x2_p_2, x3_p_2 = 0,0,0,0

    my_time = []
    pos_1 = []
    pos_2 = []
    vel_1 = []
    vel_2 = []
    ref1_vec = []
    ref2_vec = []
    u1_vec = []
    u2_vec = []

    # Motors Variables
    Pos_radf = []
    Pos_radf2 = []
    iA = 0
    iB = 0
    iA2 = 0
    iB2 = 0
    global ref_1
    ref_1 = 0.0
    global ref_2
    ref_2 = 0.0

    def inputChng_M1A(channel):
        global iA
        if ref_1>=0:
            iA += 1
        if ref_1<0:
            iA = iA-1
    def inputChng_M1B(channel):
        global iB
        if ref_1>=0:
            iB += 1
        if ref_1<0:
            iB = iB-1
    def inputChng_M2A(channel):
        global iA2
        if ref_2>=0:
            iA2 += 1
        if ref_2<0:
            iA2 = iA2-1
    def inputChng_M2B(channel):
        global iB2
        if ref_2>=0:
            iB2 += 1
        if ref_2<0:
            iB2 = iB2-1


    global PPR
    kc=1.5267
    k1p=12.1417
    k2p=22.6520
    PPR = 34*22.0
    global t
    t = 0

    GPIO.add_event_detect(Enc_A, GPIO.RISING, callback=inputChng_M1A, bouncetime=1)
    GPIO.add_event_detect(Enc_B, GPIO.RISING, callback=inputChng_M1B, bouncetime=1)

    GPIO.add_event_detect(Enc_A2, GPIO.RISING, callback=inputChng_M2A, bouncetime=1)
    GPIO.add_event_detect(Enc_B2, GPIO.RISING, callback=inputChng_M2B, bouncetime=1)

    kt = Motors_Controller()
    # plt.figure(1)
    # plt.plot(my_time, ref1_vec, 'r')
    # plt.plot(my_time, vel_1, 'b')
    # plt.figure(2)
    # plt.plot(my_time, u1_vec, 'y')
    # plt.figure(3)
    # plt.plot(my_time, ref2_vec, 'b')
    # plt.plot(my_time, vel_2, 'r')
    # plt.figure(4)
    # plt.plot(my_time, u2_vec, 'g')
    # plt.show()

    try:
        if not rospy.is_shutdown():
            rospy.spin()
            M1L.stop()
            M1R.stop()
            M2L.stop()
            M2R.stop()

            GPIO.remove_event_detect(Enc_A)
            GPIO.remove_event_detect(Enc_B)
            GPIO.remove_event_detect(Enc_A2)
            GPIO.remove_event_detect(Enc_B2)
            GPIO.cleanup()


    except rospy.ROSInterruptException as e:
        print(e)
