#!/usr/bin/env python3

#load directory to custom opencv library binaries with see3cam support
import os, sys
if '/lib/opencv+econ' not in os.environ['LD_LIBRARY_PATH']:
    home = os.path.abspath(os.path.dirname(os.path.abspath(__file__))+"/..")
    os.environ['LD_LIBRARY_PATH'] = home+'/lib/opencv+econ:'+os.environ['LD_LIBRARY_PATH']
    os.environ['PYTHONPATH'] = home+'/lib/opencv+econ:'+os.environ['PYTHONPATH']
    os.execv(sys.argv[0], sys.argv)
    exit()

import threading
import math
import numpy as np
import cv2, PIL, os
from cv2 import aruco

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import matplotlib as mpl
import pandas as pd

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D, Pose


#camera properties
cam_offs_x = 0.0; cam_offs_y = 0.0;
cam_mat = np.array([
    [189.09167097,   0.0       , 303.24887518],
    [  0.0       , 189.09167097, 238.29592283],
    [  0.0       ,   0.0       ,   1.0       ]
])
cam_dis = np.array([
    [ 2.60795135e+00],[ 3.90439304e+00],[-4.19072992e-04],[ 2.97129516e-05],[ 2.12766848e-01],[ 2.65389817e+00],[ 4.01250767e+00],[ 7.48218890e-01],[ 0.00000000e+00],[ 0.00000000e+00],[ 0.00000000e+00],[ 0.00000000e+00],[ 0.00000000e+00],[ 0.00000000e+00]
])
cam_rot = np.array([
    [  0.0,  1.0 ,  0.0 ],
    [ -1.0,  0.0 ,  0.0 ],
    [  0.0,  0.0 ,  1.0 ]
])
#board properties (hardcoded)
board_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
board_obj = aruco.CharucoBoard_create(17, 17, 15, 12, board_dict)
board_offs_x = -0.2; board_offs_y = -1.1; board_offs_theta = -0.005 #offsets in board frame
#board properties (calculated)
board_cent = np.transpose([np.sum(board_obj.objPoints[63], 0)/4]) - np.array([[board_offs_x],[board_offs_y],[0.0]])
board_rot = np.array([
    [  1.0,  0.0 ,  0.0 ],
    [  0.0, -1.0 ,  0.0 ],
    [  0.0,  0.0 , -1.0 ]
]).dot( np.array([
    [  np.cos(board_offs_theta), -np.sin(board_offs_theta) ,  0.0 ],
    [  np.sin(board_offs_theta),  np.cos(board_offs_theta) ,  0.0 ],
    [  0.0,  0.0 , 1.0 ]
]) )

#program entry point
def main():

    #initialize node
    rospy.init_node('plate')
    rospy.on_shutdown(shutdown)
    #load parameters
    global param_rate; param_rate = float(rospy.get_param("~rate"))
    global param_mode; param_mode = rospy.get_param("~mode")
    global param_vicon_topic; param_vicon_topic = rospy.get_param("~vicon_topic")
    #initialize subscriptions
    rospy.Subscriber('odometry_drone', Odometry, callback_odom_drone)
    if param_mode=="vicon": rospy.Subscriber(param_vicon_topic, Odometry, callback_odom_plate_vicon)
    #initialize publishers
    global pub_odom_plate; pub_odom_plate = rospy.Publisher('odometry_plate', Pose2D, queue_size=256)

    #initialize camera
    if param_mode=="camera":
        global cap
        #camera capture
        cap = cv2.VideoCapture()
        cap.open(0)
        if not cap.isOpened(): rospy.logerr("failed to open camera")
        cap.setFormatType(1) #GREY 640x480 180fps
        cap.set(10, 10, 2)   #brightness 10 (1-240)
        cap.set(15, 140, 2)  #exposure 20 (1-1000)
        rospy.sleep(0.1)
        #capture thread
        def cam_grab():
            while True:
                ret = cap.grab()
                if not ret: break
        cam_grab_thread = threading.Thread(target=cam_grab)
        cam_grab_thread.daemon = True
        cam_grab_thread.start()
        rospy.sleep(0.1)


    #run plate tracker
    rate = rospy.Rate(param_rate)
    while not rospy.is_shutdown():
        odom_plate = Pose2D()

        # "none" mode: set plate position as equal to drone position
        if param_mode == "none":
            #get drone yaw
            odom_drone_quat = odom_drone.orientation
            odom_drone_rpy = euler_from_quaternion(odom_drone_quat.x,odom_drone_quat.y,odom_drone_quat.z,odom_drone_quat.w)
            odom_drone_yaw = odom_drone_rpy[2]
            #calculate plate odometry
            odom_plate.x = odom_drone.position.x
            odom_plate.y = odom_drone.position.y
            odom_plate.theta = odom_drone_yaw

        # "vicon" mode: use vicon odometry of plate directly
        elif param_mode == "vicon":
            #get drone yaw
            odom_plate_quat = odom_plate_vicon.orientation
            odom_plate_rpy = euler_from_quaternion(odom_plate_quat.x,odom_plate_quat.y,odom_plate_quat.z,odom_plate_quat.w)
            odom_plate_yaw = odom_plate_rpy[2]
            #calculate plate odometry
            odom_plate.x = odom_plate_vicon.position.x
            odom_plate.y = odom_plate_vicon.position.y
            odom_plate.theta = odom_plate_yaw

        # "camera" mode: use camera and charuco markers for plate odometry
        elif param_mode == "camera":

            #get frame
            ret, frame_orig = cap.retrieve()
            frame = cv2.cvtColor(frame_orig, cv2.COLOR_BGR2GRAY)
            #find aruco markers
            mrk_crn, mrk_ids, _ = aruco.detectMarkers(frame, board_dict); mrk_num = len(mrk_crn)
            mrk_ret, mrk_rvec, mrk_tvec = cv2.aruco.estimatePoseBoard(mrk_crn, mrk_ids, board_obj, cam_mat, cam_dis)
            #find chessboard corners
            if mrk_num > 0:
                chs_num, chs_crn, chs_ids = cv2.aruco.interpolateCornersCharuco(mrk_crn, mrk_ids, frame, board_obj)
                _, chs_rvec, chs_tvec = cv2.aruco.estimatePoseCharucoBoard(chs_crn, chs_ids, board_obj, cam_mat, cam_dis)
            #pick pose estimator
            if mrk_num == 0:   rvec = np.zeros((3, 1)); tvec = np.zeros((3, 1))
            elif chs_num < 6:  rvec = mrk_rvec.copy();  tvec = mrk_tvec.copy()
            else:              rvec = chs_rvec.copy();  tvec = chs_tvec.copy()
            #calculate plate odometry relative to drone
            odom_rel_pos = cam_rot.dot( tvec + np.matmul(cv2.Rodrigues(rvec)[0], board_cent) ) /1000.0
            odom_rel_mat = np.linalg.multi_dot([cam_rot, cv2.Rodrigues(rvec)[0], board_rot, cam_rot.transpose()])
            odom_rel_yaw = yaw_from_matrix(odom_rel_mat)
            if mrk_num == 0: odom_rel_pos = np.zeros((3,1)); odom_rel_yaw = 0.0
            #calculate drone odometry relative to world
            odom_drone_pos = array_from_xyz(odom_drone.position)
            odom_drone_quat = array_from_xyzw(odom_drone.orientation)
            odom_drone_yaw = yaw_from_quaternion(odom_drone_quat)
            #calculate drone odometry relative to world
            odom_plate.x = odom_drone_pos[0][0] + odom_rel_pos[0][0] + cam_offs_x
            odom_plate.y = odom_drone_pos[1][0] + odom_rel_pos[1][0] + cam_offs_y
            odom_plate.theta = odom_drone_yaw + odom_rel_yaw
            #throw warning if no markers were found
            if mrk_num == 0: rospy.logwarn("failed to find markers")


            #display detection result
            #print(odom_plate)
            #frame_orig = aruco.drawDetectedMarkers(frame_orig.copy(), mrk_crn, mrk_ids)
            #frame_marker = aruco.drawDetectedCornersCharuco(frame_orig.copy(), chs_crn, chs_ids)
            #plt.figure();plt.imshow(frame_marker);plt.show()

        #publish plate odometry
        pub_odom_plate.publish(odom_plate)

        rate.sleep()


#callback for drone odometry topic
odom_drone = Pose();
odom_drone.orientation.w = 1.0
def callback_odom_drone(odom_drone_new):
    global odom_drone; odom_drone = odom_drone_new.pose.pose

#callback for plate vicon odometry topic
odom_plate_vicon = Pose();
odom_plate_vicon.orientation.w = 1.0
def callback_odom_plate_vicon(odom_plate_vicon_new):
    global odom_plate_vicon; odom_plate_vicon = odom_plate_vicon_new.pose.pose

#callback for  node shutdown
def shutdown():
    cap.release()

################################################################################

#return array version particular objects
def array_from_xyz(obj): return np.array([[obj.x], [obj.y], [obj.z]])
def array_from_xyzw(obj): return [obj.x, obj.y, obj.z, obj.w]
#calculate yaw angle from quaternion
def yaw_from_quaternion(q): return math.atan2( 2.0 * (q[3]*q[2] + q[0]*q[1]) , 1.0 - 2.0*(q[1]*q[1] + q[2]*q[2]) )
#calculate yaw angle from matrix
def yaw_from_matrix(m): return math.atan2(m[1][0],m[0][0])

if __name__ == '__main__':
    main()
