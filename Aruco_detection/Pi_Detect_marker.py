# This code is a adaptation of the original code taken from -> https://github.com/tizianofiorenzani/how_do_drones_work.git#

# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np
import cv2.aruco as aruco
import sys, math

# Define tag
id_to_find=25
marker_size=10 #-[cm]

#  Get the camera calibration path
calib_path='Calibpath/'
camera_matrix= np.loadtxt(calib_path+'cameraMatrix.txt', delimiter=',')
camera_distortion= np.loadtxt(calib_path+'cameraDistortion.txt', delimiter=',')

#  180 deg rotation matrix aroud the x axis
R_flip= np.zeros((3,3), dtype=np.float32)
R_flip[0,0]=1.0
R_flip[1,1]=-1.0
R_flip[2,2]=-1.0

# Definde the aruco dictionary
aruco_dict=aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
parameters= aruco.DetectorParameters_create()

font= cv2.FONT_HERSHEY_PLAIN

# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))
# allow the camera to warmup
time.sleep(0.1)
# capture frames from the camera
while True:
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        image = frame.array
        rawCapture.truncate(0)


        gray=cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        # FInd all the aruco markers in the image
        corners, ids, rejected=aruco.detectMarkers(image=gray, dictionary=aruco_dict, parameters=parameters)
        if ids is not None and ids[0] == id_to_find:
            # ret=[rvec, tvex, ?]
            # array of rotation and position of each marker in camera frame
            # rvec= [[rvec_1], [rvec_2], ...]  attitude of the marker respect to camera frame
            # tvec= [[tvec_1], [tvec_2], ...]  position of the marker respect to camera frame

            ret = aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, camera_distortion)

            # Unpack the output,get only the first
            rvec, tvec = ret[0][0, 0, :], ret[1][0, 0, :]

            # Draw th detected marker and put a reference frame over it
            aruco.drawDetectedMarkers(image, corners)
            aruco.drawAxis(image, camera_matrix, camera_distortion, rvec, tvec, 10)

            # Print the tag position in camera frame
            str_position = 'Marker Postion x=%4.0f  y=%4.0f  z=%4.0f' % (tvec[0], tvec[1], tvec[2])
            cv2.putText(image, str_position, (0, 290), font, 1, (0, 255, 0), 1, cv2.LINE_AA)

        # Display the frame
        cv2.imshow('aruco_detection', image)

        # Use 'q' to quit
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            cap.release()
            cv2.destroyAllWindows()
            break
