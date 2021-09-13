#!/usr/bin/env python

import rospy
from marker.msg import Setpoint, Draw
from math import sin, cos, pi
from dynamixel_sdk import PortHandler, PacketHandler, GroupSyncWrite
import serial

# dynamixel constants
DXL_DEVICE = '/dev/ttyUSB0'
DXL_COMM_SUCCESS = 0
DXL_ADDR_OPERATING_MODE   = 11
DXL_ADDR_TORQUE_ENABLE    = 64
DXL_ADDR_LED              = 65
DXL_ADDR_GOAL_POSITION    = 116
DXL_ADDR_PRESENT_POSITION = 132
DXL_PULSE_PER_MM          = 4095/(100*pi)
DXL_ID1 = 21; DXL_ID2 = 22; DXL_ID3 = 23
dxl_port = 0; dxl_pack = 0; dxl_goal_write = 0

#drawer constants
DRW_DEVICE = '/dev/ttyACM1'
drw_port = 0

#callback for setpoint topic
def setpoint_callback(setpoint):

    #helper function to split long into byte array
    def to_4ByteArray(long):
        return [ (long>>0)&0xFF, (long>>8)&0xFF, (long>>16)&0xFF, (long>>24)&0xFF]

    #trigonometric constants
    COS_150 =-0.866025403784;  SIN_150 = 0.5
    COS_270 = 0.0;             SIN_270 =-1.0
    COS_030 = 0.866025403784;  SIN_030 = 0.5

    #calculate motor goal positions
    goal1 = int(( COS_150*setpoint.pose.position.x + SIN_150*setpoint.pose.position.y )*DXL_PULSE_PER_MM)
    goal2 = int(( COS_270*setpoint.pose.position.x + SIN_270*setpoint.pose.position.y )*DXL_PULSE_PER_MM)
    goal3 = int(( COS_030*setpoint.pose.position.x + SIN_030*setpoint.pose.position.y )*DXL_PULSE_PER_MM)

    #send goal positions to motors
    dxl_goal_write.addParam(DXL_ID1, to_4ByteArray(goal1))
    dxl_goal_write.addParam(DXL_ID2, to_4ByteArray(goal2))
    dxl_goal_write.addParam(DXL_ID3, to_4ByteArray(goal3))
    dxl_goal_write.txPacket()
    dxl_goal_write.clearParam()


#callback for draw topic
def draw_callback(draw):

    #send draw command to servo
    ret = drw_port.write(chr(draw.state.data))


#callback for marker node shutdown
def shutdown():

    #turn of motor torque
    dxl_pack.write1ByteTxRx(dxl_port, DXL_ID1, DXL_ADDR_TORQUE_ENABLE, 0)
    dxl_pack.write1ByteTxRx(dxl_port, DXL_ID2, DXL_ADDR_TORQUE_ENABLE, 0)
    dxl_pack.write1ByteTxRx(dxl_port, DXL_ID3, DXL_ADDR_TORQUE_ENABLE, 0)
    #turn of motor led
    dxl_pack.write1ByteTxRx(dxl_port, DXL_ID1, DXL_ADDR_LED, 0)
    dxl_pack.write1ByteTxRx(dxl_port, DXL_ID2, DXL_ADDR_LED, 0)
    dxl_pack.write1ByteTxRx(dxl_port, DXL_ID3, DXL_ADDR_LED, 0)


#program entry point
def main():
    global dxl_port, dxl_pack, dxl_goal_write
    global drw_port

    #initialize node and subscriptions
    rospy.init_node('marker')
    rospy.on_shutdown(shutdown)
    rospy.Subscriber('setpoint', Setpoint, setpoint_callback)
    rospy.Subscriber('draw', Draw, draw_callback)

    #connect to dynamixel motors
    dxl_port = PortHandler(DXL_DEVICE)
    dxl_pack = PacketHandler(2.0)
    dxl_goal_write = GroupSyncWrite(dxl_port, dxl_pack, DXL_ADDR_GOAL_POSITION, 4)
    if not dxl_port.openPort():             rospy.logerr(rospy.get_caller_id() + ' failed to open dynamixel port'); return
    if not dxl_port.setBaudRate(57600):     rospy.logerr(rospy.get_caller_id() + ' failed to set dynamixel baud rate'); return

    #set dynamixel multi turn mode
    dxl_res, dxl_err = dxl_pack.write1ByteTxRx(dxl_port, DXL_ID1, DXL_ADDR_OPERATING_MODE, 4)
    if dxl_res != DXL_COMM_SUCCESS:  rospy.logerr(rospy.get_caller_id() + ' failed to find dynamixel motor 1'); return
    dxl_res, dxl_err = dxl_pack.write1ByteTxRx(dxl_port, DXL_ID2, DXL_ADDR_OPERATING_MODE, 4)
    if dxl_res != DXL_COMM_SUCCESS:  rospy.logerr(rospy.get_caller_id() + ' failed to find dynamixel motor 2'); return
    dxl_res, dxl_err = dxl_pack.write1ByteTxRx(dxl_port, DXL_ID3, DXL_ADDR_OPERATING_MODE, 4)
    if dxl_res != DXL_COMM_SUCCESS:  rospy.logerr(rospy.get_caller_id() + ' failed to find dynamixel motor 3'); return
    #enable motor torque
    dxl_res, dxl_err = dxl_pack.write1ByteTxRx(dxl_port, DXL_ID1, DXL_ADDR_TORQUE_ENABLE, 1)
    dxl_res, dxl_err = dxl_pack.write1ByteTxRx(dxl_port, DXL_ID2, DXL_ADDR_TORQUE_ENABLE, 1)
    dxl_res, dxl_err = dxl_pack.write1ByteTxRx(dxl_port, DXL_ID3, DXL_ADDR_TORQUE_ENABLE, 1)
    #enable dynamixel led
    dxl_pack.write1ByteTxRx(dxl_port, DXL_ID1, DXL_ADDR_LED, 1)
    dxl_pack.write1ByteTxRx(dxl_port, DXL_ID2, DXL_ADDR_LED, 1)
    dxl_pack.write1ByteTxRx(dxl_port, DXL_ID3, DXL_ADDR_LED, 1)

    #connect to drawer
    drw_port = serial.Serial(DRW_DEVICE)  # open serial port

    #run until node is shut down
    rospy.spin()

if __name__ == '__main__':
    main()
