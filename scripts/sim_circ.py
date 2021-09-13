#!/usr/bin/env python

import rospy
from marker.msg import Setpoint, Draw

from math import sin, cos, pi


def sim_circ():

    pub_setpoint = rospy.Publisher('setpoint', Setpoint, queue_size=10)
    pub_draw = rospy.Publisher('draw', Draw, queue_size=10)
    rospy.init_node('sim_circ')
    rate = rospy.Rate(60)

    duration = 16000; # manouver duration [ms]
    radius = 90;    # manouver radius [mm]
    dashes = 12;

    time_start = rospy.Time.now()
    while not rospy.is_shutdown():

        time_now = rospy.Time.now()
        time_perc = (time_now-time_start).to_nsec()/1e6 / duration;
        if time_perc > 1.0: break

        setpoint = Setpoint()
        setpoint.header.stamp = time_now
        setpoint.pose.position.x = radius * cos(2*pi*time_perc) - radius
        setpoint.pose.position.y = radius * sin(2*pi*time_perc)

        draw = Draw()
        draw.header.stamp = time_now
        draw.state.data = bool(int(time_perc*dashes*2)%2)

        rospy.loginfo(rospy.get_caller_id() + ' sending setpoint %2.0f%%', time_perc*100)
        pub_setpoint.publish(setpoint)
        pub_draw.publish(draw)
        rate.sleep()

if __name__ == '__main__':
    try:
        sim_circ()
    except rospy.ROSInterruptException:
        pass
