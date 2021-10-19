#!/usr/bin/env python3

#https://mecaruco2.readthedocs.io/en/latest/notebooks_rst/Aruco/sandbox/ludovic/aruco_calibration_rotation.html

#load opencv from lib folder
import os, sys
if '../lib/opencv+econ' not in os.environ['LD_LIBRARY_PATH']:
    os.environ['LD_LIBRARY_PATH'] = '../../lib/opencv+econ:'+os.environ['LD_LIBRARY_PATH']
    os.environ['PYTHONPATH'] = '../../lib/opencv+econ:'+os.environ['PYTHONPATH']
    os.execv(sys.argv[0], sys.argv)
    exit()

import numpy as np
import cv2, PIL, os
from cv2 import aruco
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import matplotlib as mpl
import pandas as pd

# generate checkerboard
aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
board = aruco.CharucoBoard_create(17, 17, 15, 12, aruco_dict)

#load images
datadir = "./charuco_img/"
images = np.array([datadir + f for f in os.listdir(datadir) if f.endswith(".jpg") ])

mtx = np.array([
    [189.09167097,   0.0       , 303.24887518],
    [  0.0       , 189.09167097, 238.29592283],
    [  0.0       ,   0.0       ,   1.0       ]

])

dist = np.array([
    [ 2.60795135e+00],
    [ 3.90439304e+00],
    [-4.19072992e-04],
    [ 2.97129516e-05],
    [ 2.12766848e-01],
    [ 2.65389817e+00],
    [ 4.01250767e+00],
    [ 7.48218890e-01],
    [ 0.00000000e+00],
    [ 0.00000000e+00],
    [ 0.00000000e+00],
    [ 0.00000000e+00],
    [ 0.00000000e+00],
    [ 0.00000000e+00]
])


i=2 # select image id
plt.figure()
frame = cv2.imread(images[i])
img_undist = cv2.undistort(frame,mtx,dist,None)
plt.subplot(1,2,1)
plt.imshow(frame)
plt.title("Raw image")
plt.axis("off")
plt.subplot(1,2,2)
plt.imshow(img_undist)
plt.title("Corrected image")
plt.axis("off")
plt.show()

gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
parameters =  aruco.DetectorParameters_create()
corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
# SUB PIXEL DETECTION
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.0001)
for corner in corners:
    cv2.cornerSubPix(gray, corner, winSize = (3,3), zeroZone = (-1,-1), criteria = criteria)
frame_markers = aruco.drawDetectedMarkers(frame.copy(), corners, ids)
plt.figure()
plt.imshow(frame_markers, interpolation = "nearest")
plt.show()
