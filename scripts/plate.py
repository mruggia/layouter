#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D
from tf.transformations import euler_from_quaternion

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
        odom_drone_quat = odom_drone.pose.pose.orientation
        odom_drone_rpy = euler_from_quaternion([odom_drone_quat.x,odom_drone_quat.y,odom_drone_quat.z,odom_drone_quat.w])
        odom_drone_yaw = odom_drone_rpy[2]

        #calculate plate odometry
        odom_plate = Pose2D()
        odom_plate.x = odom_drone.pose.pose.position.x
        odom_plate.y = odom_drone.pose.pose.position.y
        odom_plate.theta = odom_drone_yaw

        #publish plate odometry
        pub_odom_plate.publish(odom_plate)

        rate.sleep()

    #run until node is shut down
    rospy.spin()

#callback for drone odometry topic
odom_drone = Odometry();
def callback_odom_drone(odom_drone_new):
    global odom_drone; odom_drone = odom_drone_new


if __name__ == '__main__':
    main()
