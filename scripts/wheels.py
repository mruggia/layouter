#!/usr/bin/env python3

import math
from copy import deepcopy
from dynamixel_sdk import PortHandler, PacketHandler, GroupSyncWrite

import rospy
from marker.msg import StatePlate
from marker.msg import ControlPlate
from marker.srv import WheelsStart, WheelsStartResponse
from marker.srv import WheelsStop, WheelsStopResponse

##############################################################################

#program entry point
def main():

    #initialize node and subscriptions
    rospy.init_node('wheels')
    rospy.on_shutdown(dxl_off)
    #initialize subscriptions
    rospy.Subscriber('setpoint_plate', StatePlate, setpoint_plate_callback)
    rospy.Subscriber('odometry_plate', StatePlate, odometry_plate_callback)
    #initialize publishers
    global pub_control_plate; pub_control_plate = rospy.Publisher('control_plate', ControlPlate, queue_size=256)
    #initialize services
    rospy.Service('wheels_start', WheelsStart, wheels_start_handle)
    rospy.Service('wheels_stop', WheelsStop, wheels_stop_handle)
    #load parameters
    global param_rate; param_rate = float(rospy.get_param("~rate"))
    global param_controller; param_controller = rospy.get_param("~controller")
    global param_ctrl_pos_p; param_ctrl_pos_p = rospy.get_param("~ctrl_pos_p")
    global param_ctrl_ang_p; param_ctrl_ang_p = rospy.get_param("~ctrl_ang_p")
    global param_wheel_dist; param_wheel_dist = rospy.get_param("~wheel_dist")
    global DXL_DEV; DXL_DEV = rospy.get_param("device_wheels")
    global DXL_ID1; DXL_ID1 = int(rospy.get_param("~DXL_ID1"))
    global DXL_ID2; DXL_ID2 = int(rospy.get_param("~DXL_ID2"))
    global DXL_ID3; DXL_ID3 = int(rospy.get_param("~DXL_ID3"))
    global DXL_UNIT_PER_MPS; DXL_UNIT_PER_MPS = 60.0/0.229/( float(rospy.get_param("~wheel_dia"))/1000.0*math.pi )

    #connect to dynamixel motors
    global dxl_port; dxl_port = PortHandler(DXL_DEV)
    global dxl_pack; dxl_pack = PacketHandler(2.0)
    global dxl_goal_vel_writer; dxl_goal_vel_writer = GroupSyncWrite(dxl_port, dxl_pack, 104, 4)
    if not dxl_port.openPort():         rospy.logerr('failed to open dynamixel port'); return
    if not dxl_port.setBaudRate(57600): rospy.logerr('failed to set dynamixel baud rate'); return
    if dxl_pack.read1ByteTxRx(dxl_port, DXL_ID1, 6)[1] != 0: rospy.logerr('failed to find dynamixel motor 1'); return
    if dxl_pack.read1ByteTxRx(dxl_port, DXL_ID2, 6)[1] != 0: rospy.logerr('failed to find dynamixel motor 2'); return
    if dxl_pack.read1ByteTxRx(dxl_port, DXL_ID3, 6)[1] != 0: rospy.logerr('failed to find dynamixel motor 3'); return
    dxl_on()


    #run wheel update
    rate = rospy.Rate(param_rate)
    while not rospy.is_shutdown():

        if wheels_running:

            #feed-forward control
            if param_controller == "ff":
                theta = setp_plate.pos.theta
                vx = setp_plate.vel.x
                vy = setp_plate.vel.y
                vt = 0

            #proportional control
            if param_controller == "p":
                theta = (setp_plate.pos.theta+odom_plate.pos.theta)/2.0
                vx = setp_plate.vel.x + param_ctrl_pos_p * (setp_plate.pos.x-odom_plate.pos.x)
                vy = setp_plate.vel.y + param_ctrl_pos_p * (setp_plate.pos.y-odom_plate.pos.y)
                vt =                    param_ctrl_ang_p * (setp_plate.pos.theta-odom_plate.pos.theta)

            #send velocity to actuators
            wheels_goal = wheels_from_vec( vx, vy, theta )
            wheels_goal = [goal+vt for goal in wheels_goal]
            dxl_goal_vel_set(*vel_from_wheels(*wheels_goal))

            #publish control velocity
            control_plate = ControlPlate()
            control_plate.header.stamp = rospy.Time.now()
            control_plate.header.frame_id ='world'
            control_plate.vel.x = vx
            control_plate.vel.y = vy
            control_plate.vel.theta = vt/param_wheel_dist
            pub_control_plate.publish(control_plate)

        rate.sleep()


################################################################################
# callbalcks

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

#handle for start/stop service
home_plate = StatePlate()
wheels_running = False
def wheels_start_handle(req):
    global home_plate, setp_plate, wheels_running
    home_plate = deepcopy(req.home)
    setp_plate = deepcopy(req.home)
    wheels_running = True
    return WheelsStartResponse()
def wheels_stop_handle(req):
    global wheels_running
    wheels_running = False
    dxl_goal_vel_set(0,0,0)
    return WheelsStopResponse()

################################################################################
# utility

# convert 2d vector to wheels coordinate
def wheels_from_vec(x,y, theta):
    COS_240 =-0.5;  SIN_240 =-0.866025403784
    COS_000 = 1.0;  SIN_000 = 0.0
    COS_120 =-0.5;  SIN_120 = 0.866025403784
    rot_x = math.cos(-theta)*x - math.sin(-theta)*y
    rot_y = math.sin(-theta)*x + math.cos(-theta)*y
    p1 = COS_240*rot_x + SIN_240*rot_y
    p2 = COS_000*rot_x + SIN_000*rot_y
    p3 = COS_120*rot_x + SIN_120*rot_y
    return p1, p2, p3

#convert wheels velocity value [m/2] to dynamixel unit
def vel_from_wheels(p1, p2, p3):
    p1 = int( p1 * DXL_UNIT_PER_MPS)
    p2 = int( p2 * DXL_UNIT_PER_MPS)
    p3 = int( p3 * DXL_UNIT_PER_MPS)
    return p1, p2, p3

################################################################################
# dynamixel commands

#turn motors on in mode depending on controller
def dxl_on():
    dxl_write(64, 1, 0)     #disable motor torque
    dxl_write(11, 1, 1)     #set mode to velocity control

    dxl_write(44, 4, 1023)  #unlock velocity limit
    dxl_write(36, 2, 885)   #unlock pwm limit
    dxl_write(78, 2, 600)   #velocity P gain
    dxl_write(76, 2, 6000)  #velocity I gain

    dxl_write(64, 1, 1)     #enable motor torque
    dxl_write(65, 1, 1)     #enable motor red led
#turn motors off
def dxl_off():
    dxl_write(64, 1, 0)     #disable motor torque
    dxl_write(65, 1, 0)     #disable motor red led

#set goal vel with GroupSyncWrite (fast)
def dxl_goal_vel_set(p1,p2,p3):
    dxl_goal_vel_writer.addParam(DXL_ID1, to_4ByteArray(p1))
    dxl_goal_vel_writer.addParam(DXL_ID2, to_4ByteArray(p2))
    dxl_goal_vel_writer.addParam(DXL_ID3, to_4ByteArray(p3))
    dxl_goal_vel_writer.txPacket()
    dxl_goal_vel_writer.clearParam()
#utility function to split number into 4 bytes
def to_4ByteArray(long):
    return [ (long>>0)&0xFF, (long>>8)&0xFF, (long>>16)&0xFF, (long>>24)&0xFF]

#generalized write command
def dxl_write(address, length, data1,data2=None,data3=None):
    if data2 == None: data2 = data1; data3 = data1
    if length == 1:
        dxl_pack.write1ByteTxRx(dxl_port, DXL_ID1, address, data1)
        dxl_pack.write1ByteTxRx(dxl_port, DXL_ID2, address, data2)
        dxl_pack.write1ByteTxRx(dxl_port, DXL_ID3, address, data3)
    if length == 2:
        dxl_pack.write2ByteTxRx(dxl_port, DXL_ID1, address, data1)
        dxl_pack.write2ByteTxRx(dxl_port, DXL_ID2, address, data2)
        dxl_pack.write2ByteTxRx(dxl_port, DXL_ID3, address, data3)
    elif length == 4:
        dxl_pack.write4ByteTxRx(dxl_port, DXL_ID1, address, data1)
        dxl_pack.write4ByteTxRx(dxl_port, DXL_ID2, address, data2)
        dxl_pack.write4ByteTxRx(dxl_port, DXL_ID3, address, data3)
#generalized read command
def dxl_read(address, length):
    if length == 1:
        data1 = dxl_pack.read1ByteTxRx(dxl_port, DXL_ID1, address)[0]
        data2 = dxl_pack.read1ByteTxRx(dxl_port, DXL_ID2, address)[0]
        data3 = dxl_pack.read1ByteTxRx(dxl_port, DXL_ID3, address)[0]
    if length == 2:
        data1 = dxl_pack.read2ByteTxRx(dxl_port, DXL_ID1, address)[0]
        data2 = dxl_pack.read2ByteTxRx(dxl_port, DXL_ID2, address)[0]
        data3 = dxl_pack.read2ByteTxRx(dxl_port, DXL_ID3, address)[0]
    elif length == 4:
        data1 = dxl_pack.read4ByteTxRx(dxl_port, DXL_ID1, address)[0]
        data2 = dxl_pack.read4ByteTxRx(dxl_port, DXL_ID2, address)[0]
        data3 = dxl_pack.read4ByteTxRx(dxl_port, DXL_ID3, address)[0]
    return data1, data2, data3


################################################################################

if __name__ == '__main__':
    main()
