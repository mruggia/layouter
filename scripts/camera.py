#!/usr/bin/env python3

#load custom opencv binaries with econ camera support (located in "lib/opencv+econ")
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
import matplotlib.pyplot as plt

import rospy
from nav_msgs.msg import Odometry
from marker.msg import StatePlate

################################################################################
# global variables

#camera offset relative to drone
cam_offs_x = 0.0; cam_offs_y = 0.0;
#camera projection matrix
cam_mat = np.array([
    [189.09167097,   0.0       , 303.24887518],
    [  0.0       , 189.09167097, 238.29592283],
    [  0.0       ,   0.0       ,   1.0       ]
])
#camera distorsion values
cam_dis = np.array([
    [ 2.60795135e+00],[ 3.90439304e+00],[-4.19072992e-04],[ 2.97129516e-05],[ 2.12766848e-01],[ 2.65389817e+00],[ 4.01250767e+00],[ 7.48218890e-01],[ 0.00000000e+00],[ 0.00000000e+00],[ 0.00000000e+00],[ 0.00000000e+00],[ 0.00000000e+00],[ 0.00000000e+00]
])
#camera rotation rel to drone frame
cam_rot = np.array([
    [  0.0,  1.0 ,  0.0 ],
    [ -1.0,  0.0 ,  0.0 ],
    [  0.0,  0.0 ,  1.0 ]
])
#board properties
board_dict = aruco.Dictionary_get(aruco.DICT_6X6_100)
board_obj = aruco.CharucoBoard_create(14, 14, 15, 12, board_dict)
board_offs_x = +0.8; board_offs_y = -3.2; board_offs_theta = -0.007 #offsets in board frame
#board properties (calculated)
board_cent = np.transpose([np.sum(board_obj.objPoints[45], 0)/4]) - np.array([[board_offs_x],[board_offs_y],[0.0]])
board_rot = np.array([
    [  1.0,  0.0 ,  0.0 ],
    [  0.0, -1.0 ,  0.0 ],
    [  0.0,  0.0 , -1.0 ]
]).dot( np.array([
    [  np.cos(board_offs_theta), -np.sin(board_offs_theta) ,  0.0 ],
    [  np.sin(board_offs_theta),  np.cos(board_offs_theta) ,  0.0 ],
    [  0.0,  0.0 , 1.0 ]
]) )

################################################################################

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
    global pub_odom_plate; pub_odom_plate = rospy.Publisher('odometry_plate', StatePlate, queue_size=256)

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
        #init variable used to calculate camera velocity
        odom_rel_pos_prev = np.zeros((3,1));  odom_rel_yaw_prev = 0.0
        odom_rel_dpos_prev = np.zeros((3,1)); odom_rel_dyaw_prev = 0.0
        odom_rel_alpha = 0.3 #weight of new data for vel smoothing

    #run plate tracker
    rate = rospy.Rate(param_rate)
    while not rospy.is_shutdown():
        odom_plate = StatePlate()
        odom_plate.header.stamp = rospy.Time.now()
        odom_plate.header.frame_id ='world'

        # "none" mode: set plate position as equal to drone position
        if param_mode == "none":
            #get drone yaw
            odom_drone_quat = array_from_xyzw(odom_drone.pose.pose.orientation)
            odom_drone_yaw = yaw_from_quaternion(odom_drone_quat)
            #calculate plate odometry
            odom_plate.pos.x = odom_drone.pose.pose.position.x
            odom_plate.pos.y = odom_drone.pose.pose.position.y
            odom_plate.pos.theta = odom_drone_yaw
            odom_plate.dz = float("nan")
            odom_plate.vel.x = odom_drone.twist.twist.linear.x
            odom_plate.vel.y = odom_drone.twist.twist.linear.y
            odom_plate.vel.theta = odom_drone.twist.twist.angular.z

        # "vicon" mode: use vicon odometry of plate directly
        elif param_mode == "vicon":
            #get plate yaw
            odom_plate_quat = array_from_xyzw(odom_plate_vicon.pose.pose.orientation)
            odom_plate_yaw = yaw_from_quaternion(odom_plate_quat)
            #calculate plate odometry
            odom_plate.pos.x = odom_plate_vicon.pose.pose.position.x
            odom_plate.pos.y = odom_plate_vicon.pose.pose.position.y
            odom_plate.pos.theta = odom_plate_yaw
            odom_plate.dz = float("nan")
            odom_plate.vel.x = odom_plate_vicon.twist.twist.linear.x
            odom_plate.vel.y = odom_plate_vicon.twist.twist.linear.y
            odom_plate.vel.theta = odom_plate_vicon.twist.twist.angular.z

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
            #calculate plate position relative to drone
            odom_rel_pos = cam_rot.dot( tvec + np.matmul(cv2.Rodrigues(rvec)[0], board_cent) ) /1000.0
            odom_rel_mat = np.linalg.multi_dot([cam_rot, cv2.Rodrigues(rvec)[0], board_rot, cam_rot.transpose()])
            odom_rel_yaw = yaw_from_matrix(odom_rel_mat)
            if mrk_num == 0: odom_rel_pos = np.zeros((3,1)); odom_rel_yaw = 0.0
            #calculate plate velocity relative to drone
            odom_rel_dpos = odom_rel_alpha*param_rate*(odom_rel_pos-odom_rel_pos_prev) + (1.0-odom_rel_alpha)*odom_rel_dpos_prev
            odom_rel_dyaw = odom_rel_alpha*param_rate*(odom_rel_yaw-odom_rel_yaw_prev) + (1.0-odom_rel_alpha)*odom_rel_dyaw_prev
            odom_rel_pos_prev = odom_rel_pos; odom_rel_dpos_prev = odom_rel_dpos
            odom_rel_yaw_prev = odom_rel_yaw; odom_rel_dyaw_prev = odom_rel_dyaw
            #calculate drone position relative to world
            odom_drone_pos = array_from_xyz(odom_drone.pose.pose.position)
            odom_drone_quat = array_from_xyzw(odom_drone.pose.pose.orientation)
            odom_drone_yaw = yaw_from_quaternion(odom_drone_quat)
            odom_drone_dpos = array_from_xyz(odom_drone.twist.twist.linear)
            odom_drone_dyaw = odom_drone.twist.twist.angular.z
            #calculate plate position relative to world
            odom_plate.pos.x = odom_drone_pos[0][0] + odom_rel_pos[0][0] + cam_offs_x
            odom_plate.pos.y = odom_drone_pos[1][0] + odom_rel_pos[1][0] + cam_offs_y
            odom_plate.pos.theta = odom_drone_yaw + odom_rel_yaw
            odom_plate.dz = odom_rel_pos[2][0]
            odom_plate.vel.x = odom_drone_dpos[0][0] + odom_rel_dpos[0][0]
            odom_plate.vel.y = odom_drone_dpos[1][0] + odom_rel_dpos[1][0]
            odom_plate.vel.theta = odom_drone_dyaw + odom_rel_dyaw

            #display detection result
            #frame_orig = aruco.drawDetectedMarkers(frame_orig.copy(), mrk_crn, mrk_ids)
            #frame_orig = aruco.drawDetectedCornersCharuco(frame_orig.copy(), chs_crn, chs_ids)
            #plt.figure();plt.imshow(frame_orig);plt.show()

        #publish plate odometry
        if param_mode == "camera" and mrk_num == 0:
            rospy.logwarn("failed to find markers. no plate odometry published")
        else:
            pub_odom_plate.publish(odom_plate)

        rate.sleep()

################################################################################
# callbacks

#callback for drone odometry topic
odom_drone = Odometry();
odom_drone.pose.pose.orientation.w = 1.0
def callback_odom_drone(odom_drone_new):
    global odom_drone; odom_drone = odom_drone_new

#callback for plate vicon odometry topic
odom_plate_vicon = Odometry();
odom_plate_vicon.pose.pose.orientation.w = 1.0
def callback_odom_plate_vicon(odom_plate_vicon_new):
    global odom_plate_vicon; odom_plate_vicon = odom_plate_vicon_new

#callback for  node shutdown
def shutdown():
    cap.release()

################################################################################
# utility

#return array version particular objects
def array_from_xyz(obj): return np.array([[obj.x], [obj.y], [obj.z]])
def array_from_xyzw(obj): return [obj.x, obj.y, obj.z, obj.w]
#calculate yaw angle from quaternion
def yaw_from_quaternion(q): return math.atan2( 2.0 * (q[3]*q[2] + q[0]*q[1]) , 1.0 - 2.0*(q[1]*q[1] + q[2]*q[2]) )
#calculate yaw angle from matrix
def yaw_from_matrix(m): return math.atan2(m[1][0],m[0][0])

################################################################################

if __name__ == '__main__':
    main()
