#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool
import serial

#callback for draw topic
def draw_callback(draw):
    #forward draw command to servo
    ret = serial.write(str(int(draw.data)).encode())

#callback for drawer node shutdown
def shutdown():
    #return marker to default on
    serial.write(b'1')

#program entry point
def main():

    #initialize node and subscriptions
    rospy.init_node('marker')
    rospy.on_shutdown(shutdown)
    rospy.Subscriber('draw', Bool, draw_callback)
    #load parameters
    device = rospy.get_param("~device")

    #connect to marker
    global serial; serial = serial.Serial(device)
    #set marker to default off
    serial.write(b'0')

    #run until node is shut down
    rospy.spin()

if __name__ == '__main__':
    main()
