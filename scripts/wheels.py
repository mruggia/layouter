#!/usr/bin/env python3

import math
from copy import deepcopy
from dynamixel_sdk import PortHandler, PacketHandler, GroupSyncWrite, GroupSyncRead

import rospy
from nav_msgs.msg import Odometry
from marker.msg import StatePlate
from marker.srv import Home,HomeResponse

################################################################################

#program entry point
def main():

    #initialize node and subscriptions
    rospy.init_node('wheels')
    rospy.on_shutdown(dxl_off)
    #initialize subscriptions
    rospy.Subscriber('setpoint_plate', StatePlate, setpoint_plate_callback)
    rospy.Subscriber('odometry_plate', StatePlate, odometry_plate_callback)
    #initialize services
    rospy.Service('home', Home, home_handle)
    #load parameters
    global param_rate; param_rate = float(rospy.get_param("~rate"))
    global DXL_DEV; DXL_DEV = rospy.get_param("~DXL_DEV")
    global DXL_ID1; DXL_ID1 = int(rospy.get_param("~DXL_ID1"))
    global DXL_ID2; DXL_ID2 = int(rospy.get_param("~DXL_ID2"))
    global DXL_ID3; DXL_ID3 = int(rospy.get_param("~DXL_ID3"))
    global DXL_PULSE_PER_M; DXL_PULSE_PER_M = 4095.0/( float(rospy.get_param("~wheel_dia"))/1000.0*math.pi )
    #connect to dynamixel motors
    global dxl_port; dxl_port = PortHandler(DXL_DEV)
    global dxl_pack; dxl_pack = PacketHandler(2.0)
    global dxl_goal_write; dxl_goal_write = GroupSyncWrite(dxl_port, dxl_pack, 116, 4)
    global dxl_pres_read; dxl_pres_read = GroupSyncRead(dxl_port, dxl_pack, 132, 4)
    dxl_pres_read.addParam(DXL_ID1); dxl_pres_read.addParam(DXL_ID2); dxl_pres_read.addParam(DXL_ID3)
    if not dxl_port.openPort():         rospy.logerr(rospy.get_caller_id() + ' failed to open dynamixel port'); return
    if not dxl_port.setBaudRate(57600): rospy.logerr(rospy.get_caller_id() + ' failed to set dynamixel baud rate'); return
    dxl_on()
    #trigonometric constants
    COS_240 =-0.5;  SIN_240 =-0.866025403784
    COS_000 = 1.0;  SIN_000 = 0.0
    COS_120 =-0.5;  SIN_120 = 0.866025403784

    #run wheel update
    rate = rospy.Rate(param_rate)
    while not rospy.is_shutdown():

        #calculate motor goal positions
        rot_x = math.cos(-setp_plate.pos.theta)*(setp_plate.pos.x-home_plate.pos.x) - math.sin(-setp_plate.pos.theta)*(setp_plate.pos.y-home_plate.pos.y)
        rot_y = math.sin(-setp_plate.pos.theta)*(setp_plate.pos.x-home_plate.pos.x) + math.cos(-setp_plate.pos.theta)*(setp_plate.pos.y-home_plate.pos.y)
        goal1 = -int(( COS_240*rot_x + SIN_240*rot_y )*DXL_PULSE_PER_M)
        goal2 = -int(( COS_000*rot_x + SIN_000*rot_y )*DXL_PULSE_PER_M)
        goal3 = -int(( COS_120*rot_x + SIN_120*rot_y )*DXL_PULSE_PER_M)

        #send goal positions to motors
        dxl_set(goal1, goal2, goal3)

        rate.sleep()


################################################################################

#handle for plate home service
home_plate = StatePlate()
def home_handle(req):
    global home_plate
    home_plate = deepcopy(req.home)
    dxl_home()
    return HomeResponse(True)

#callback for plate odometry topic
odom_plate = StatePlate()
def odometry_plate_callback(odom_plate_new):
    global odom_plate
    odom_plate = deepcopy(odom_plate_new)

#callback for setpoint topic
setp_plate = StatePlate()
def setpoint_plate_callback(setp_plate_new):
    global setp_plate
    setp_plate = deepcopy(setp_plate_new)


################################################################################

#turn on dynamixels
def dxl_on():
    #enable motor torque
    dxl_res, dxl_err = dxl_pack.write1ByteTxRx(dxl_port, DXL_ID1, 64, 1)
    if dxl_res != 0:  rospy.logerr(rospy.get_caller_id() + ' failed to find dynamixel motor 1'); return
    dxl_res, dxl_err = dxl_pack.write1ByteTxRx(dxl_port, DXL_ID2, 64, 1)
    if dxl_res != 0:  rospy.logerr(rospy.get_caller_id() + ' failed to find dynamixel motor 2'); return
    dxl_res, dxl_err = dxl_pack.write1ByteTxRx(dxl_port, DXL_ID3, 64, 1)
    if dxl_res != 0:  rospy.logerr(rospy.get_caller_id() + ' failed to find dynamixel motor 3'); return
    #enable dynamixel led
    dxl_pack.write1ByteTxRx(dxl_port, DXL_ID1, 65, 1)
    dxl_pack.write1ByteTxRx(dxl_port, DXL_ID2, 65, 1)
    dxl_pack.write1ByteTxRx(dxl_port, DXL_ID3, 65, 1)
    #set home position
    dxl_home()

#turn off dynamixels
def dxl_off():
    #turn of motor torque
    dxl_pack.write1ByteTxRx(dxl_port, DXL_ID1, 64, 0)
    dxl_pack.write1ByteTxRx(dxl_port, DXL_ID2, 64, 0)
    dxl_pack.write1ByteTxRx(dxl_port, DXL_ID3, 64, 0)
    #turn of motor led
    dxl_pack.write1ByteTxRx(dxl_port, DXL_ID1, 65, 0)
    dxl_pack.write1ByteTxRx(dxl_port, DXL_ID2, 65, 0)
    dxl_pack.write1ByteTxRx(dxl_port, DXL_ID3, 65, 0)

#reset dynamxiels home position
def dxl_home():

    #disable motor torque
    dxl_pack.write1ByteTxRx(dxl_port, DXL_ID1, 64, 0)
    dxl_pack.write1ByteTxRx(dxl_port, DXL_ID2, 64, 0)
    dxl_pack.write1ByteTxRx(dxl_port, DXL_ID3, 64, 0)
    #set operating mode to position control (resets present position to 0-360deg)
    dxl_pack.write1ByteTxRx(dxl_port, DXL_ID1, 11, 3)
    dxl_pack.write1ByteTxRx(dxl_port, DXL_ID2, 11, 3)
    dxl_pack.write1ByteTxRx(dxl_port, DXL_ID3, 11, 3)
    #enable motor torque
    dxl_pack.write1ByteTxRx(dxl_port, DXL_ID1, 64, 1)
    dxl_pack.write1ByteTxRx(dxl_port, DXL_ID2, 64, 1)
    dxl_pack.write1ByteTxRx(dxl_port, DXL_ID3, 64, 1)
    #disable motor torque
    dxl_pack.write1ByteTxRx(dxl_port, DXL_ID1, 64, 0)
    dxl_pack.write1ByteTxRx(dxl_port, DXL_ID2, 64, 0)
    dxl_pack.write1ByteTxRx(dxl_port, DXL_ID3, 64, 0)
    #revert back to multi turn mode
    dxl_pack.write1ByteTxRx(dxl_port, DXL_ID1, 11, 4)
    dxl_pack.write1ByteTxRx(dxl_port, DXL_ID2, 11, 4)
    dxl_pack.write1ByteTxRx(dxl_port, DXL_ID3, 11, 4)
    #set home position to 0
    dxl_pack.write4ByteTxRx(dxl_port, DXL_ID1, 20, 0)
    dxl_pack.write4ByteTxRx(dxl_port, DXL_ID2, 20, 0)
    dxl_pack.write4ByteTxRx(dxl_port, DXL_ID3, 20, 0)
    #read present position
    dxl_pos1,dxl_pos2,dxl_pos3 = dxl_get()
    #set present position as new home
    dxl_pack.write4ByteTxRx(dxl_port, DXL_ID1, 20, -dxl_pos1)
    dxl_pack.write4ByteTxRx(dxl_port, DXL_ID2, 20, -dxl_pos2)
    dxl_pack.write4ByteTxRx(dxl_port, DXL_ID3, 20, -dxl_pos3)
    #enable motor torque
    dxl_pack.write1ByteTxRx(dxl_port, DXL_ID1, 64, 1)
    dxl_pack.write1ByteTxRx(dxl_port, DXL_ID2, 64, 1)
    dxl_pack.write1ByteTxRx(dxl_port, DXL_ID3, 64, 1)

#set dynamixels goal position
def dxl_set(a,b,c):
    #set dynamixels goal position
    dxl_goal_write.addParam(DXL_ID1, to_4ByteArray(a))
    dxl_goal_write.addParam(DXL_ID2, to_4ByteArray(b))
    dxl_goal_write.addParam(DXL_ID3, to_4ByteArray(c))
    dxl_goal_write.txPacket()
    dxl_goal_write.clearParam()

#get dynamixels present position
def dxl_get():
    dxl_pres_read.txRxPacket()
    dxl_pos1 = dxl_pres_read.getData(DXL_ID1, 132, 4)
    dxl_pos2 = dxl_pres_read.getData(DXL_ID2, 132, 4)
    dxl_pos3 = dxl_pres_read.getData(DXL_ID3, 132, 4)
    return dxl_pos1, dxl_pos2, dxl_pos3

#helper function to split long into byte array
def to_4ByteArray(long):
    return [ (long>>0)&0xFF, (long>>8)&0xFF, (long>>16)&0xFF, (long>>24)&0xFF]

################################################################################

if __name__ == '__main__':
    main()
