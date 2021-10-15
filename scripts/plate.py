#!/usr/bin/env python3

import math
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D, Pose

#program entry point
def main():

    #initialize node
    rospy.init_node('plate')
    #initialize subscriptions
    rospy.Subscriber('odometry_drone', Odometry, callback_odom_drone)
    #initialize publishers
    global pub_odom_plate; pub_odom_plate = rospy.Publisher('odometry_plate', Pose2D, queue_size=256)
    #load parameters
    global param_rate; param_rate = float(rospy.get_param("~rate"))

    #run plate tracker
    rate = rospy.Rate(param_rate)
    while not rospy.is_shutdown():

        #get drone yaw
        odom_drone_quat = odom_drone.orientation
        odom_drone_rpy = euler_from_quaternion(odom_drone_quat.x,odom_drone_quat.y,odom_drone_quat.z,odom_drone_quat.w)
        odom_drone_yaw = odom_drone_rpy[2]

        #calculate plate odometry
        odom_plate = Pose2D()
        odom_plate.x = odom_drone.position.x
        odom_plate.y = odom_drone.position.y
        odom_plate.theta = odom_drone_yaw

        #publish plate odometry
        pub_odom_plate.publish(odom_plate)

        rate.sleep()


#callback for drone odometry topic
odom_drone = Pose();
odom_drone.orientation.w = 1.0
def callback_odom_drone(odom_drone_new):
    global odom_drone; odom_drone = odom_drone_new.pose.pose

#calculate euler angles from quaternion
def euler_from_quaternion(x, y, z, w):

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z # in radians


if __name__ == '__main__':
    main()
