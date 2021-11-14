#!/usr/bin/env python3

import numpy as np
from copy import deepcopy
import ast

import rospy

from marker.msg import StatePlate
from std_msgs.msg import Header
from geometry_msgs.msg import Vector3, Transform, Twist
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint

from std_srvs.srv import Empty, EmptyResponse

################################################################################

def main():
    #initialize node
    rospy.init_node('preplanner')
    #load parameters
    global param_rate; param_rate = float(rospy.get_param("~rate"))
    global des_move; des_move = ast.literal_eval(rospy.get_param("~des_move"))
    global vel_move; vel_move = float(rospy.get_param("~vel_move"))
    global des_touch; des_touch = float(rospy.get_param("~des_touch"))
    global vel_touch; vel_touch = float(rospy.get_param("~vel_touch"))
    #initialize subscriptions
    rospy.Subscriber('odometry_plate', StatePlate, callback_odom_plate)
    rospy.Subscriber(rospy.get_param('setpoint_drone'), MultiDOFJointTrajectory, callback_setp_drone)
    #initialize publishers
    global pub_setpoint_drone; pub_setpoint_drone = rospy.Publisher(rospy.get_param('setpoint_drone'), MultiDOFJointTrajectory, queue_size=256)
    #initialize services
    rospy.Service('move', Empty, handle_move)
    rospy.Service('touch', Empty, handle_touch)
    #initialize drone setpoint
    global setp_drone;
    setp_drone = MultiDOFJointTrajectory(Header(0,rospy.Time.now(),'world'), ['base_link'], [MultiDOFJointTrajectoryPoint([Transform()], [Twist()], [Twist(),Twist()], rospy.Time(0))] )
    setp_drone.points[0].transforms[0].rotation.w = 1.0

    #run until node is shut down
    rospy.spin()

################################################################################

#callback for plate odometry topic
odom_plate = StatePlate()
def callback_odom_plate(odom_plate_new):
    global odom_plate; odom_plate = deepcopy(odom_plate_new)

#callback for drone setpoint
setp_drone = MultiDOFJointTrajectory()
def callback_setp_drone(setp_drone_new):
    global setp_drone; setp_drone = deepcopy(setp_drone_new)

################################################################################

#send setpoints to move drone to starting location
def handle_move(req):
    global ref_drone; ref_drone = deepcopy(setp_drone)
    curr_pos = np.array([ref_drone.points[0].transforms[0].translation.x, ref_drone.points[0].transforms[0].translation.y, ref_drone.points[0].transforms[0].translation.z])
    goal_pos = np.array(des_move)

    path_len = np.linalg.norm(goal_pos-curr_pos)
    step_len = vel_move/param_rate
    step_vec = (goal_pos-curr_pos)/path_len*step_len

    while True:
        path_len = np.linalg.norm(goal_pos-curr_pos)
        if path_len <= step_len: break
        curr_pos = curr_pos + step_vec
        publish_setpoint(curr_pos)
        rospy.sleep(1.0/param_rate)
    publish_setpoint(goal_pos)

    return EmptyResponse()

#send setpoints to move drone to touch ceiling
def handle_touch(req):
    global ref_drone; ref_drone = deepcopy(setp_drone)
    curr_pos = np.array([ref_drone.points[0].transforms[0].translation.x, ref_drone.points[0].transforms[0].translation.y, ref_drone.points[0].transforms[0].translation.z])

    step_len = vel_touch/param_rate

    while True:
        if odom_plate.dz <= des_touch: break
        curr_pos[2] = curr_pos[2] + step_len
        publish_setpoint(curr_pos)
        rospy.sleep(1.0/param_rate)

    return EmptyResponse()


#publish drone setpoint
def publish_setpoint(pos):
    ref_drone.header.stamp = rospy.Time.now()
    ref_drone.points[0].transforms[0].translation.x = pos[0]
    ref_drone.points[0].transforms[0].translation.y = pos[1]
    ref_drone.points[0].transforms[0].translation.z = pos[2]
    pub_setpoint_drone.publish(ref_drone)

################################################################################

if __name__ == '__main__':
    main()
