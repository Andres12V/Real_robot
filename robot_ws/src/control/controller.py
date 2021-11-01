#!/usr/bin/env python2

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan
from cv_bridge import CvBridge, CvBridgeError
from time import sleep
import cv2 as cv
from tf.transformations import euler_from_quaternion
import threading
import time

from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

import cv2.aruco as aruco
import numpy as np
import sys, time, math
import matplotlib.pyplot as plt
import serial

from sympy.matrices import Matrix
from sympy import cos, symbols, sin, simplify, pi
def transl(x,y,z):
    T = Matrix([[1,0,0,x],[0,1,0,y],
                [0,0,1,z],[0,0,0,1]])
    return T
def trotx(ang):
    T = Matrix([[1, 0, 0, 0],
                [0,cos(ang), -sin(ang),0],
                [0,sin(ang), cos(ang),0],
                [0,0,0,1]])
    return T
def troty(ang):
    T = Matrix([[cos(ang), 0, sin(ang),0],
                [0,1, 0,0],
                [-sin(ang), 0, cos(ang),0],
                [0,0,0,1]])
    return T
def trotz(ang):
    T = Matrix([[cos(ang),-sin(ang),0,0],
                [sin(ang), cos(ang),0,0],
                [0, 0, 1,0],
                [0,0,0,1]])
    return T

def isRotationMatrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6
# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).

def truncate(number, digits):
    stepper = 10.0 ** digits
    return math.trunc(stepper * number) / stepper
def rotationMatrixToEulerAngles(R):
    assert (isRotationMatrix(R))

    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    return np.array([x, y, z])
def restaurant_path():
    tray_x=np.linspace(0, 0, num=5)
    tray_y=np.linspace(0.4, 0.8, num=5)
    tray_x2=np.linspace(0.4, 0.6, num=5)
    tray_y2=np.linspace(0.8, 1.1, num=5)
    tray_x3=np.linspace(0.93, 0.93, num=5)
    tray_y3=np.linspace(1.1, 1.35, num=5)
    tray_x4=np.linspace(0.93, 0.4, num=5)
    tray_y4=np.linspace(1.35, 1.35, num=5)
    tray_x5=np.linspace(0, 0, num=5)
    tray_y5=np.linspace(1.35, 0.9, num=5)
    tray_x6=np.linspace(0, 0, num=5)
    tray_y6=np.linspace(0, 0, num=5)

    for n in range(0,2):
        my_listx.append(tray_x[n])
    for n1 in range(0,2):
        my_listx.append(tray_x2[n1])
    for n2 in range(0,2):
        my_listx.append(tray_x3[n2])
    for n3 in range(0,2):
        my_listx.append(tray_x4[n3])
    for n4 in range(0,2):
        my_listx.append(tray_x5[n4])
    for n5 in range(0,2):
        my_listx.append(tray_x6[n5])
    for m in range(0,2):
        my_listy.append(tray_y[m])
    for m1 in range(0,2):
        my_listy.append(tray_y2[m1])
    for m2 in range(0,2):
        my_listy.append(tray_y3[m2])
    for m3 in range(0,2):
        my_listy.append(tray_y4[m3])
    for m4 in range(0,2):
        my_listy.append(tray_y5[m4])
    for m5 in range(0,2):
        my_listy.append(tray_y6[m5])
    return my_listx, my_listy, tray_y6, tray_x6
def inter_p_xd(pos_x,xd,k,num_p):
    x_list=np.linspace(pos_x, xd, num=num_p)
    return x_list[k]
def inter_p_yd(pos_y,yd,k,num_p):
    y_list=np.linspace(pos_y, yd, num=num_p)
    return y_list[k]
def aruco_num_request():
    global aruco_num
    while 1:
        A = ''
        List = ['1','2','3','4','5','6','B','7','8','9','C','*','0','#','D']
        ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
        ser.flush()
        while True:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8').rstrip()
                #print(line)
                if line in List:
                    if line == 'D':
                        print('---------')
                        A = A[:-1]
                    else:
                        A += line
                        print('Aruco_ID:', A)
                        #print(len(A))

                if line == 'A':
                    break
        aruco_num = str(A)
        #aruco_num = input('Enter ArUco Marker id: ')
        if str(aruco_num) in List_aruco_ids:
            break
        else:
            print('The requested table is not avilable, please enter a valid number')
            print('Avilables are 12,13,14,15,16,17,18,21,25')
            continue
    return aruco_num

class Controller:
    def __init__(self):
        rospy.init_node('control')
        # self.sub_orientation = rospy.Subscriber('/my_project/orientation', Float64, self.th_callback)
        self.sub_orientation_encoder = rospy.Subscriber('/my_project/orientation_encoder', Float64, self.orientation_callback)

        # Camera Image topic
        self.sub_image = rospy.Subscriber('/my_project/camera', Image, self.cam_callback)
        self.bridge = CvBridge()
        # POSE of the robot
        self.sub = rospy.Subscriber('/my_project/odom', Float64, self.control_callback)
        self.sub_vel1 = rospy.Subscriber('/my_project/m1_velocity', Float64 , self.motor1_callback)
        self.sub_vel2 = rospy.Subscriber('/my_project/m2_velocity', Float64 , self.motor2_callback)
        #self.pub = rospy.Publisher('/my_turtlebot_control/cmd_vel', Twist, queue_size=10)
        # Wheel velocity topics
        self.left_wheel_vel = rospy.Publisher('/my_project/m1_velocity_ref', Float64, queue_size=10)
        self.right_wheel_vel = rospy.Publisher('/my_project/m2_velocity_ref', Float64, queue_size=10)

        self.l_w_msg = Float64()
        self.r_w_msg = Float64()
        # self.msg = Twist()

        # Infrared sensors

        # self.sub_laser1 = rospy.Subscriber('/my_project/sonar1', Float64, self.ls1_callback)
        # self.sub_laser2 = rospy.Subscriber('/my_project/sonar2', Float64, self.ls2_callback)
        # self.sub_laser3 = rospy.Subscriber('/my_project/sonar3', Float64, self.ls3_callback)
        # self.sub_laser4 = rospy.Subscriber('/my_project/sonar4', Float64, self.ls4_callback)

        # Position and orientation of the ArUco Marker
        self.pub_aruco_pose = rospy.Publisher('/aruco_pose', Pose, queue_size=10)
        self.aruco_msg = Pose()

        # Frecuency of the loop:
        self.rate = rospy.Rate(10000)
    def orientation_callback(self,msgor):
        global theta
        th_e=0
        theta=msgor.data+th_e


        # if theta < -np.pi:
        #     theta=theta+np.pi
        #
        # if theta > np.pi:
        #     theta=theta-np.pi
        # th_rd = round(th_r*(180/np.pi),2)
        # print('Mein ang', th_r, th_rd)

    # def th_callback(self,msgth):
    #     global theta
    #     theta=round(msgth.data*np.pi/180,2)
    #     print('theta',theta)



    def control_callback(self, msg1):
        global Button, theta, t, int_e, state
        global i,k
        global xd, yd

        " Get the POSE of the robot: "
        x_e=0
        y_e=0
        pos_x = msg1.data*np.cos(theta)+x_e
        pos_y = msg1.data*np.sin(theta)+y_e

        " Get orientation: "
        theta_deg = round(theta*(180/np.pi),2)
        print('Pose: x,y,theta', pos_x, pos_y, theta_deg)

        " Flags that switch the go-to-goal behavior to obstacle avoidance behavior:"
        Flag_r = 1
        try:
            Aruco_list_x.append(pos_ar_x)
            Aruco_list_y.append(pos_ar_y)
            print('xar',pos_ar_x)
            print('yar',pos_ar_y)
            posd_ar_x = Aruco_list_x[0]
            posd_ar_y = Aruco_list_y[0]
            #T1 = transl(pos_x,pos_y,0)*trotz(theta)
            #T2 = transl(pos_ar_x,pos_ar_y,0)
            #T0_1 = T1*T2
            #global_x_ar, global_y_ar = T0_1[0,3], T0_1[1,3]
            #print('GARx,GARy',global_x_ar, global_y_ar)
        except:
            pass

        if Button==0:
            """If the marker is detected reach the marker"""
            if state=='M_detected':
                # xd = global_x_ar
                # yd = global_y_ar
                xd = inter_p_xd(Aruco_list_x[0],0,k,num_p=3)
                yd = inter_p_yd(Aruco_list_y[0],0,k,num_p=3)
            """If there is no marker detected follow the designed path"""
            if state=='No_M_detected':
                # xd = round(my_listx[i], 2)
                # yd = round(my_listy[i], 2)
                if i<=29:
                    xd = inter_p_xd(pos_x,0,k,num_p=3)
                    yd = inter_p_yd(pos_y,0,k,num_p=3)

            # if state=='No_obstacle_detected'
            #     xd = n_px
            #     yd = n_py
            print('Desired Pose xd yd', xd, yd)
            # except:
            #     pass
            # xd = round(self.aruco_msg.position.x, 2)
            # yd = -(round(self.aruco_msg.position.y, 2) - my_list[0])
            # xd = 2.0
            # yd = 2.0
            # xd = round(my_listx[i], 2)
            # yd = round(my_listy[i], 2)
            #print('Desired Pose xd, yd', xd, yd)
        elif Button==1:
            xd = inter_p_xd(pos_x,0,k,num_p=3)
            yd = inter_p_yd(pos_y,0,k,num_p=3)
            print('Desired Pose xd, yd', xd, yd)
        "Erros in the x & y coordinates:"
        ex = xd-pos_x
        ey = yd-pos_y
        "Calculation of the total error in X and Y positions"
        e_t=np.abs(ex)+np.abs(ey)
        print('e_t',e_t)
        if state=='M_detected':
            if e_t<=0.05:
                theta_d=theta
            else:
                theta_d=np.arctan2(ey, ex)
        if state=='No_M_detected':
            if e_t<=0.09:
                theta_d=theta
                print('theta_d=th')
            else:
                theta_d=np.arctan2(ey, ex)
        "Orientation error:"
        e_heading = (theta_d - theta)

        "Control signals:"
        e_diag = np.sqrt( (ex*ex) + (ey*ey) )
        print('e_diag',e_diag)
        int_e+=e_diag
        Vel = (0.04*e_diag)*Flag_r
        Omega = ((np.arctan2(np.sin(e_heading),np.cos(e_heading)))*0.17)*Flag_r
        # try:
        #     xcf=xc+e_diag
        # except:
        #     xcf=e_diag
        # xc=xcf
        #
        # if e_diag<0.05:
        #     xc=0

        # if np.abs(e_diag)<=0.5:

        #Vel = 0.05
        #Omega = 0

        # if np.abs(e_diag)>0.5:
        #     Vel = (e_diag*0.0065)*Flag_r
        #     Omega = ((np.arctan2(np.sin(e_heading),np.cos(e_heading)))*0.018)*Flag_r
        # if e_heading==0:
        #     Vel=0
        #     Omega=0
        #     self.left_wheel_vel.publish(self.l_w_msg)
        #     self.right_wheel_vel.publish(self.r_w_msg)

        "Get the required velocity of each wheel to reach the goal"
        self.l_w_msg = (1/(2*R))*(2*Vel-Omega*L)
        if self.l_w_msg >= 9:
             self.l_w_msg = 9
        if self.l_w_msg <= -9:
             self.l_w_msg = -9
        self.r_w_msg = -(1/(2*R))*(2*Vel+Omega*L)
        if self.r_w_msg >= 9:
             self.r_w_msg = 9
        if self.r_w_msg <= -9:
             self.r_w_msg = -9
        "Publish the messages to the topics:"
        self.left_wheel_vel.publish(self.l_w_msg)
        self.right_wheel_vel.publish(self.r_w_msg)
        "If ArUco marker is detected, reach the marker and wait for the confirmation"
        if state=='M_detected':
            """If the button have not been pressed and the total error is 0, the goal
            have been reached, so the program waits for the confirmation button"""
            if Button==0:
                if e_t<=0.09:
                    self.left_wheel_vel.publish(0)
                    self.right_wheel_vel.publish(0)
                    Button = input('Enter 1: ')
                    k = 1
                    Aruco_list_x.clear()
                    Aruco_list_y.clear()


            elif Button==1:
            #"""If the button have been pressed and the total error is 0, the robot
            #have been arrived to the initial position, then waits for a new table request"""
                if e_t<=0.05:
                    self.left_wheel_vel.publish(0)
                    self.right_wheel_vel.publish(0)
                    Button = 0
                    state = 'No_M_detected'
                    aruco_num_request()
        """ If no ArUco marker is detected, set the designed path points as the goal """
        if state=='No_M_detected':
            if e_diag<=0.15:
                int_e=0
                k = 1
                # self.left_wheel_vel.publish(0)
                # self.right_wheel_vel.publish(0)
                # while 1:
                #     Button = input('Enter 1: ')
                #     if Button==1:
                #         break
                if i<29:
                    i+=1
                else:
                    i=29
        "Append the position of the robot in each iteration for the plot "
        cord_x.append(pos_x)
        cord_y.append(pos_y)
        vec_th.append(theta)
        ref_th.append(theta_d)
        ref_x.append(xd)
        ref_y.append(yd)
        l.append(self.l_w_msg)
        r.append(self.r_w_msg)
        my_t.append(t)

        t+=0.0001

    def motor1_callback(self, msg_w1):
        l_motor.append(msg_w1.data)
    def motor2_callback(self, msg_w2):
        r_motor.append(msg_w2.data)
    def cam_callback(self, data):
        global state
        global pos_ar_x
        global pos_ar_y
        try:
            # Convert the image message from the simulated camera to cv image
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')

            ## This part is for the real camera ##
            # ret, frame=cap.read()
            # gray=cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
            # cv.imshow('Image', frame)
            # aruco_image = frame

            aruco_image = cv_image
            id_to_find = int(aruco_num)
            marker_size = 10 #-[cm]
            #  Get the camera calibration path
            calib_path='/home/pi/robot_ws/src/control/src/Calibpath/'
            camera_matrix= np.loadtxt(calib_path+'cameraMatrix.txt', delimiter=',')
            camera_distortion= np.loadtxt(calib_path+'cameraDistortion.txt', delimiter=',')

            #  180 deg rotation matrix aroud the x axis
            R_flip= np.zeros((3,3), dtype=np.float32)
            R_flip[0,0]=1.0
            R_flip[1,1]=-1.0
            R_flip[2,2]=-1.0

            # Define the aruco dictionary
            aruco_dict=aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
            parameters= aruco.DetectorParameters_create()
            font= cv.FONT_HERSHEY_PLAIN
            # Convert in gray scale
            gray=cv.cvtColor(aruco_image, cv.COLOR_BGR2GRAY)

            # FInd all the aruco markers in the image
            corners, ids, rejected=aruco.detectMarkers(image=gray, dictionary=aruco_dict, parameters=parameters)
            if ids is not None and ids[0] == id_to_find:
                state = 'M_detected'
                aruco_image = aruco.drawDetectedMarkers(aruco_image, corners)
                ret = aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, camera_distortion)
                try:
                    rvec, tvec=ret[0][0,0,:], ret[1][0,0,:]
                    aruco.drawAxis(aruco_image, camera_matrix, camera_distortion, rvec, tvec, 10)
                    str_position= 'Marker Postion x=%4.0f  y=%4.0f  z=%4.0f'%(tvec[0], tvec[1], tvec[2])
                    cv.putText(aruco_image, str_position, (0,290), font, 1, (0, 255, 255), 1, cv.LINE_AA)
                    #-- Obtain the rotation matrix tag->camera
                    R_ct    = np.matrix(cv.Rodrigues(rvec)[0])
                    R_tc    = R_ct.T
                    #-- Get the attitude in terms of euler 321 (Needs to be flipped first)
                    roll_marker, pitch_marker, yaw_marker = rotationMatrixToEulerAngles(R_flip*R_tc)
                    #-- Print the marker's attitude respect to camera frame
                    str_attitude = "Marker Attitude r=%4.0f  p=%4.0f  y=%4.0f"%(math.degrees(roll_marker),math.degrees(pitch_marker),
                                        math.degrees(yaw_marker))
                    cv.putText(aruco_image, str_attitude, (0, 315), font, 1, (0, 255, 255), 1, cv.LINE_AA)

                    #-- Now get Position and attitude f the camera respect to the marker
                    pos_camera = -R_tc*np.matrix(tvec).T

                    str_position = "CAMERA Position x=%4.0f  y=%4.0f  z=%4.0f"%(pos_camera[0], pos_camera[1], pos_camera[2])
                    cv.putText(aruco_image, str_position, (0, 340), font, 1, (255, 255, 0), 1, cv.LINE_AA)

                    #-- Get the attitude of the camera respect to the frame
                    roll_camera, pitch_camera, yaw_camera = rotationMatrixToEulerAngles(R_flip*R_tc)
                    str_attitude = "CAMERA Attitude r=%4.0f  p=%4.0f  y=%4.0f"%(math.degrees(roll_camera),math.degrees(pitch_camera),
                                        math.degrees(yaw_camera))
                    cv.putText(aruco_image, str_attitude, (0, 365), font, 1, (255, 255, 0), 1, cv.LINE_AA)
                    cv.putText(aruco_image, str(aruco_num) , (0, 380), font, 1, (0, 0, 255), 1, cv.LINE_AA)

                except:
                    print('No ArUco Detected')

            # Display the frame
            cv.imshow('frame', aruco_image)
            #cv.imwrite('/home/andresvergara/images_aruco/pics/img15.jpg', cv_image)
            try:
                pos_ar_y = 0.01*tvec[0]
                #print('Aruco_x',pos_ar_x)
                pos_ar_x = 0.01*tvec[2]
                #print('Aruco_y',pos_ar_y)
                self.aruco_msg.position.x = pos_ar_x
                self.aruco_msg.position.y = pos_ar_y
                self.aruco_msg.position.z = 0.5
                self.pub_aruco_pose.publish(self.aruco_msg)
            except:

                pass

        except CvBridgeError as e:
            print(e)
        k = cv.waitKey(1)
        if k == ord('r'):
            self.msg.linear.x = 0
            self.msg.linear.y = 0
            self.msg.angular.z = 0
            rospy.signal_shutdown('shutdown')
            cv.destroyAllWindows()
        #self.pub_image_test.publish(aruco_image2)

        self.rate.sleep()


    #
    # def cases():
    #     if dist1 and dist2 <0.3:
    #         self.l_w_msg = (1/(2*R))*(2*Vel-Omega*L)
    #         self.r_w_msg = (1/(2*R))*(2*Vel+Omega*L)
    #         print (self.l_w_msg)
    #         print (self.l_r_msg)
    #         Flag1 = 0
    #     else:
    #         Flag1 = 1
    #     self.left_wheel_vel.publish(self.l_w_msg)
    #     self.right_wheel_vel.publish(self.r_w_msg)





    def ls1_callback(self, msg_ls1):
        global Flag1
        global dist1
        # Get the distance reading from the sensor
        dist1 = round(msg_ls1.data, 2)
        print('Distancia 1',dist1)
        phi1 = (np.pi/2)+theta
        # def Timer_Interrupt():
        #     dist1=round(msg_ls1.ranges[359], 2)
        #     print('Posicion actualizada',dist1)
        #     threading.Timer(1, Timer_Interrupt).start()
        # threading.Timer(1, Timer_Interrupt).start()
        # If the distance is less than 30 cm, switch to the obstacle avoidance behavior

        if dist1<0.3 and dist3>0.3:



            # Control signals:
            # dist1_d=0.3
            # edist1=dist1_d-dist1
            # phi1_d = np.arctan2(edist1)
            # e_heading_dist1=phi1_d-phi1
            # Vel = ( 0.05*np.sqrt( (edist1*edist1) ) )
            # Omega= ( 100*-np.arctan2( np.sin(e_heading_dist1), np.cos(e_heading_dist1) ) )
            # self.l_w_msg = (1/(2*R))*(2*Vel-Omega*L)
            # self.r_w_msg = (1/(2*R))*(2*Vel+Omega*L)

            Flag1=0
            self.r_w_msg = -3.2
            self.l_w_msg = 3.2

            print ('Im here')
            # print ('Right speed',self.r_w_msg)
            # print ('Angle',theta)
            # Flag1 = 0

        else:
            time.sleep(0.0005)

            # print ('Left speed',self.l_w_msg)
            # print ('Right speed',self.r_w_msg)
            # print ('Angle',theta)
            # Flag1 = 0

            # Control signals:
        self.left_wheel_vel.publish(self.l_w_msg)
        self.right_wheel_vel.publish(self.r_w_msg)

    def ls2_callback(self, msg_ls2):
        global Flag2
        global dist2
        dist2 = round(msg_ls2.data, 2)
        phi2 = -(np.pi/2)+theta
        if dist2<0.2 and dist3>0.29:
            # Control signals:
            # dist2_d=0.6
            # edist2=dist2_d-dist2
            # phi2_d = np.arctan2(edist2)
            # e_heading_dist2=phi2_d-phi2
            # Vel = ( 0.05*np.sqrt( (edist2*edist2) ) )
            # Omega= ( 100*-np.arctan2( np.sin(e_heading_dist2), np.cos(e_heading_dist2) ) )
            Flag2 = 0
            self.r_w_msg = -3.2
            self.l_w_msg = 3.2



        else:
            time.sleep(0.0005)

        self.left_wheel_vel.publish(self.l_w_msg)
        self.right_wheel_vel.publish(self.r_w_msg)

    def ls3_callback(self, msg_ls3):
        global Flag1
        global Flag2
        global Flag3
        global Flag4
        global dist3
        # Get the distance reading from the sensor
        dist3 = round(msg_ls3.data, 2)
        phi3 = (3*np.pi/4)+theta
        # if dist1>0.3 and dist2>0.3 and dist3>0.3 and dist4>0.3:
        #     Flag1=1
        #     Flag2=1
        #     Flag3=1
        #     Flag4=1
        # If the distance is less than 30 cm, switch to the obstacle avoidance behavior
        if dist3<0.3:
            # Control signals:
            Flag3 = 0
            dist3_d=0.3
            edist3=dist3_d-dist3
            # phi3_d = np.arctan2(edist3)
            # e_heading_dist3=phi3_d-phi3
            Vel = 4.5*(edist3)
            # Omega= ( 100*-np.arctan2( np.sin(e_heading_dist3), np.cos(e_heading_dist3) ) )


            self.l_w_msg = (1/(2*R))*(2*Vel-0*L)
            self.r_w_msg = (1/(2*R))*(2*Vel+0*L)



        elif dist1>0.3 and dist2>0.3 and dist3==0.3 and dist4>0.3:
            while dist1>0.3:
                self.r_w_msg = 3.2
                self.l_w_msg = -2.9
                if dist1<0.3 and dist2>0.3 and dist3>0.3 and dist4>0.3:
                    break
        self.left_wheel_vel.publish(self.l_w_msg)
        self.right_wheel_vel.publish(self.r_w_msg)

    def ls4_callback(self, msg_ls4):
        global Flag4
        global dist4
        dist4 = round(msg_ls4.data, 2)
        phi4 = -(3*np.pi/4)+theta

        if dist4<0.2:
            # Control signals:
            # dist4_d=0.6
            # edist4=dist4_d-dist4
            # phi4_d = np.arctan2(edist4)
            # e_heading_dist4=phi4_d-phi4
            # Vel = ( 0.05*np.sqrt( (edist4*edist4) ) )
            # Omega= ( 100*-np.arctan2( np.sin(e_heading_dist4), np.cos(e_heading_dist4) ) )
            Flag4 = 0
            dist4_d=0.3
            edist4=dist4_d-dist4
            Vel = 5*(edist4)


            self.l_w_msg = (1/(2*R))*(2*Vel-0*L)
            self.r_w_msg = (1/(2*R))*(2*Vel+0*L)
            self.left_wheel_vel.publish(self.l_w_msg)
            self.right_wheel_vel.publish(self.r_w_msg)


        else:
            Flag4 = 1

        # for v in my_listx:
        #     pos_x-v=n_px
        #     for n_px in my_listx:




if __name__ == '__main__':
    # Number of the table request:
    # rospy.init_node('obstacle_avoidance_node') #initilize node
    List_aruco_ids = ['12','13','14','15','16','17','18','21','25']
    "Number of the table request:"
    aruco_num_request()
    Flag1,Flag2,Flag3,Flag4 = 0,0,0,0
    Button = 0
    int_e = 0
    k=1
    state = 'No_M_detected'
    i = 0
    R = 0.0325
    L = 0.3
    t=0

    Aruco_list_x = list()
    Aruco_list_y = list()
    cord_x=list()
    cord_y=list()

    # Path to follow when no ArUco marker is detected:
    my_listx = list()
    my_listy = list()
    l=list()
    r=list()
    l_motor=[]
    r_motor=[]
    my_t = list()
    ref_x = list()
    ref_y = list()
    ref_th = list()
    vec_th = list()

    xd = 0
    yd = 0

    my_listx, my_listy, tray_y6, tray_x6 = restaurant_path()

    ## This part is for the real camera ##
    # cap=cv.VideoCapture(0)
    # cap.set(cv.CAP_PROP_FRAME_WIDTH, 1280)
    # cap.set(cv.CAP_PROP_FRAME_HEIGHT, 720)

    kt = Controller()
    try:
        if not rospy.is_shutdown():
            rospy.spin()
            plt.plot(cord_x,cord_y, 'g', label='Robot path')
            # plt.plot(1,6, 'x')
            # plt.plot(2.4,-2, 'o')
            # plt.plot(2.4,-5.5, 'o')
            # plt.plot(2.4,-9, 'o')
            # plt.plot(-.2,-2, 'o')
            # plt.plot(-.2,-5.5, 'o')
            # plt.plot(-.2,-9, 'o')
            # plt.plot(-2.8,-2, 'o')
            # plt.plot(-2.8,-5.5, 'o')
            # plt.plot(-2.8,-9, 'o')
            plt.plot(xd,yd,'kD', label='Target point')
            #plt.plot(tray_x6[1],tray_y6[1], 'D')
            plt.figure(1)
            #plt.plot(my_listx,my_listy, 'b')
            plt.ylabel("Y coordinates (m)")
            plt.xlabel("X coordinates (m)")
            plt.savefig('/home/pi/robot_ws/src/control/src/Sim_figs/Tracking_map.eps', dpi=300, bbox_inches='tight')
            plt.legend()
            # plt.figure(2)
            # plt.plot(l,'g--', label='Velocity ref motor1')
            # plt.plot(l_motor,'g', label='Velocity motor1')
            # plt.plot(r,'r--', label='Velocity ref motor2')
            # plt.plot(r_motor,'r', label='Velocity  motor2')
            # plt.savefig('/home/pi/robot_ws/src/control/src/Sim_figs/Wheels_v_tracking.eps', dpi=300, bbox_inches='tight')
            # plt.legend()
            # plt.figure(3)
            # plt.plot(my_t, ref_x, 'r--', label='Ref x')
            # plt.plot(my_t, cord_x, 'g', label='Pos x')
            # plt.plot(my_t, ref_y, 'k--', label='Ref y')
            # plt.plot(my_t, cord_y, 'b', label='Pos y')
            # plt.ylabel("Position (m)")
            # plt.xlabel("Time (s)")
            # plt.savefig('/home/pi/robot_ws/src/control/src/Sim_figs/Tracking_x_y.eps', dpi=300, bbox_inches='tight')
            # plt.legend()
            # plt.figure(4)
            # plt.plot(my_t, ref_th, 'r--', label='Theta desired')
            # plt.plot(my_t, vec_th, 'g', label='Theta')
            # plt.ylabel("Orientation (rad)")
            # plt.xlabel("Time (s)")
            # plt.savefig('/home/pi/robot_ws/src/control/src/Sim_figs/Orientation_tracking.eps', dpi=300, bbox_inches='tight')
            # plt.legend()
            #
            # fig, (ax1, ax2) = plt.subplots(2)
            # fig.suptitle('Tracking')
            # ax1.plot(my_t, ref_x, 'r--', label='Ref x')
            # ax1.plot(my_t, cord_x, 'g', label='Pos x')
            # ax1.plot(my_t, ref_y, 'k--', label='Ref y')
            # ax1.plot(my_t, cord_y, 'b', label='Pos y')
            #
            # ax2.plot(0,0, 'cx',  label='Starting point')
            # ax2.plot(0.5,1, 'kD', label='Target point')
            # ax2.plot(cord_x,cord_y, 'g',  label='Robot path')
            # ax1.legend()
            # ax2.legend()
            # ax2.set(xlabel='X position (m)', ylabel='Y Position (m)')
            # ax1.set(xlabel='Time (s)', ylabel='Position (m)')
            plt.show()

    except rospy.ROSInterruptException as e:
        print(e)
