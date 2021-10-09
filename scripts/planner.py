#!/usr/bin/env python

import os
import time
import math
import numpy as np
from copy import deepcopy

import rospy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Transform, Twist, Pose2D
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from std_msgs.msg import Bool

from marker.srv import Execute,ExecuteResponse, Home,HomeResponse

################################################################################

# gcode commands:
#     G0,G1: linear moves (G0=G1)
#     G2,G3: CW and CCW arc moves
#     M3,M4,M5: draw on (M3=M4) and draw off (M5) commands
# gcode arguments:
#     X,Y: target position for G0,G1,G2,G3, in absolute coordinates (start at X=0,Y=0)
#     I,J: arc center position for G2,G3, in relative coord to arc start pos

################################################################################

def main():
    #initialize node
    rospy.init_node('planner')
    #initialize subscriptions
    rospy.Subscriber('odometry_drone', Odometry, callback_odom_drone)
    rospy.Subscriber('odometry_plate', Pose2D, callback_odom_plate)
    #initialize services
    rospy.Service('execute', Execute, handle_execute)
    #initialize publishers
    global pub_draw; pub_draw = rospy.Publisher('draw', Bool, queue_size=256)
    global pub_setpoint_drone; pub_setpoint_drone = rospy.Publisher('setpoint_drone', MultiDOFJointTrajectory, queue_size=256)
    global pub_setpoint_plate; pub_setpoint_plate = rospy.Publisher('setpoint_plate', Pose2D, queue_size=256)
    #load parameters
    global param_rate; param_rate = float(rospy.get_param("~rate"))
    global param_max_vel; param_max_vel = float(rospy.get_param("~max_vel"))
    global param_max_acc; param_max_acc = float(rospy.get_param("~max_acc"))
    global param_wait_draw; param_wait_draw = float(rospy.get_param("~wait_draw"))
    global param_wait_corner; param_wait_corner = float(rospy.get_param("~wait_corner"))
    global param_tresh_corner; param_tresh_corner = float(rospy.get_param("~tresh_corner"))
    #run until node is shut down
    rospy.spin()

#callback for drone odometry topic
odom_drone_curr = Odometry();odom_drone_home = Odometry();
def callback_odom_drone(odom_drone):
    global odom_drone_curr; odom_drone_curr = odom_drone

#callback for plate odometry topic
odom_plate_curr = Pose2D(); odom_plate_home = Pose2D();
def callback_odom_plate(odom_plate):
    global odom_plate_curr; odom_plate_curr = odom_plate

################################################################################

#handle for trajectory execution service
def handle_execute(req):

    #save current plate odometry as home
    global odom_plate_home; odom_plate_home = deepcopy(odom_plate_curr)
    #calculate drone odometry home
    global odom_drone_home; odom_drone_home = Odometry()
    odom_drone_home.pose.pose.position.x = odom_plate_home.x
    odom_drone_home.pose.pose.position.y = odom_plate_home.y
    odom_drone_home.pose.pose.position.z = odom_drone_curr.pose.pose.position.z
    odom_drone_home.pose.pose.orientation.z = math.sin(odom_plate_home.theta/2)
    odom_drone_home.pose.pose.orientation.w = math.cos(odom_plate_home.theta/2)
    #send homing command to wheels
    rospy.wait_for_service('home')
    srv_home = rospy.ServiceProxy('home', Home)
    srv_home(odom_plate_home)

    #read gcode file
    gcode = open(os.path.dirname(__file__)+"/../gcode/"+req.filename, 'r').read()

    #parse gcode text
    vertices, segments = gcode_parse(gcode)

    #print result
    #print("vrt: "+str(vertices[0]))
    #for id in range(len(segments)):
    #    print("seg: "+str(segments[id]))
    #    print("vrt: "+str(vertices[id+1]))

    #run gcode
    gcode_run(vertices, segments)

    return ExecuteResponse(True)

# parse gcode text into vertices and segments readable by "run_gcode"
def gcode_parse(text):

    #initialize commands list
    vertices = [ { 'pos': [0.0,0.0], 'vel': 0.0, 'dis': 0.0} ]
    segments = []
    pos_prev = [0.0, 0.0]

    #parse gcode lines
    lines = text.splitlines()   #split gcode text in lines
    lines.append("M2")          #append program end command
    id = 0
    for line in lines:
        words = line.split(" ")
        if len(words) == 0: continue

        #read words for next position "X","Y" and next arc center "I","J"
        pos_next = pos_prev[:]; pos_cnt = pos_prev[:]
        for word in words[1:]:
            val = float(word[1:])
            if   word[0] == "X": pos_next[0] = val;
            elif word[0] == "Y": pos_next[1] = val;
            elif word[0] == "I": pos_cnt[0] = pos_prev[0]+val;
            elif word[0] == "J": pos_cnt[1] = pos_prev[1]+val;

        #parse line
        if words[0] in ["G0","G1","G2","G3","M2","M3","M4","M5"]:

            #parse linear move
            if words[0] in ["G0","G1"]:
                vec = np.array(pos_next)-np.array(pos_prev)
                lgt = np.linalg.norm(vec)
                dir = vec/lgt
                ang = math.atan2(dir[1],dir[0])
                tan_beg = ang; tan_end = ang
                vel = param_max_vel
                aux = { 'dir': list(dir) }

            #parse arc move
            elif words[0] in ["G2","G3"]:
                vec_beg = np.array(pos_prev)-np.array(pos_cnt)
                vec_end = np.array(pos_next)-np.array(pos_cnt)
                dir_beg = vec_beg/np.linalg.norm(vec_beg)
                dir_end = vec_end/np.linalg.norm(vec_end)
                ang_beg = math.atan2(dir_beg[1],dir_beg[0])
                ang_end = math.atan2(dir_end[1],dir_end[0])
                rad = np.linalg.norm(vec_beg)
                if words[0]=="G2":
                    tan_beg = math.atan2(-dir_beg[0],dir_beg[1])
                    tan_end = math.atan2(-dir_end[0],dir_end[1])
                    if ang_end>ang_beg: ang_end=ang_end-2*math.pi
                else: #words[0]=="G3"
                    tan_beg = math.atan2(dir_beg[0],-dir_beg[1])
                    tan_end = math.atan2(dir_end[0],-dir_end[1])
                    if ang_end<ang_beg: ang_end=ang_end+2*math.pi
                lgt = abs(ang_beg-ang_end)*rad
                vel = min( param_max_vel, math.sqrt(param_max_acc*rad) )
                aux = { 'cnt': pos_cnt, 'rad': rad, 'ang_beg': ang_beg, 'ang_end': ang_end }

            #parse draw on/off
            elif words[0] in ["M2","M3","M4","M5"]:
                vel = 0.0
                lgt = 0.0
                tan_beg = float("nan")
                tan_end = float("nan")
                aux = {}

            #advance to next line
            vertices.append( { 'pos': pos_next[:], 'vel': float("inf"), 'dis': vertices[-1]['dis']+lgt } )
            segments.append( { 'typ': words[0], 'vel': vel, 'len': lgt, 'tan_beg': tan_beg, 'tan_end': tan_end, 'aux': aux } )
            pos_prev = pos_next

    #limit vertex velocity by segment velocity
    for id in range(len(segments)):
        vertices[id]['vel']   = min( vertices[id]['vel'],   segments[id]['vel'] )
        vertices[id+1]['vel'] = min( vertices[id+1]['vel'], segments[id]['vel'] )

    #enforce zero velocity at start and end vertex
    vertices[0]['vel'] = 0.0
    vertices[-1]['vel'] = 0.0

    #enforce zero velocity at sharp corners
    for id in range(1,len(vertices)-1):
        if abs(segments[id]['tan_beg']-segments[id-1]['tan_end']) > math.radians(param_tresh_corner):
            vertices[id]['vel'] = 0.0

    #remove tangent data from segments
    for segment in segments:
        segment.pop('tan_beg',None)
        segment.pop('tan_end',None)

    #return
    return vertices, segments


# run gcode after it has been parsed
def gcode_run(vertices, segments):

    id = -1     #current segment id
    tme = 0.0   #current time
    vel = 0.0   #current velocity
    dis = 0.0   #distance traveled
    pos_vec = [0.0, 0.0]        #position vector along trajectory
    vel_vec = [0.0, 0.0]        #velocity vector along trajectory
    acc_vec = [0.0, 0.0]        #acceleration vector along trajectory
    stop_cmd_queue = []         #queue of comands that require trajectory to stop
    timestep = 1.0/param_rate   #length of one timestep

    #loop until end of trajectory
    while True:

        #check for segment changes
        if dis >= vertices[id+1]['dis']:    #if current dist is greater than upcoming vertex dist
            id = id + 1                     #count up segment/vertex id

            #stop command for end of trajectory
            if segments[id]['typ']=="M2":
                pos_vec = vertices[id]['pos'] #hack to get end position in last setpoint, not physically correct
                stop_cmd_queue.append("exit")
            #stop command for draw on
            elif segments[id]['typ'] in ["M3","M4"]:
                stop_cmd_queue.append("draw_on")
                continue
            #stop command for draw off
            elif segments[id]['typ']=="M5":
                stop_cmd_queue.append("draw_off")
                continue
            #stop command for sharp corner
            elif (id >=2 and vertices[id]['vel'] == 0.0
                and segments[id]['typ'] in ["G0","G1","G2","G3"]
                and segments[id-1]['typ'] in ["G0","G1","G2","G3"]):
                stop_cmd_queue.append("corner")


        acc = 0.0
        #determine acceleration
        if vel < segments[id]['vel']:                                           #if acceleration is allowed
            t_acc_act = ( segments[id]['vel'] - vel + 1e-12 ) / param_max_acc   #time to reach target velocity
            t_acc_smp = math.ceil(t_acc_act/timestep)*timestep                  #rounded to timestep multiple
            acc = ( segments[id]['vel'] - vel + 1e-12 ) / t_acc_smp             #acceleration necessary to reach target velocity at timestep multiple
        #determine decelleration
        for idn in range(id+1, len(vertices)):                                                  #loop trough upcoming vertices
            if vertices[idn]['dis']-dis > 0.5*(param_max_vel**2)/param_max_acc: break           #break if worst case decelleration dist is passed
            req_pos_acc = 0.5*(vertices[idn]['vel']**2-vel**2 ) / (vertices[idn]['dis']-dis+1e-12)  #calculate necessary acc for target vel at target dist
            if req_pos_acc <= -param_max_acc:                                                       #if necessary acc is greater max acc
                if acc < req_pos_acc: continue                                                      #skip vertex if a previous vertex already requires more decel
                req_vel_acc = (vertices[idn]['vel']-vel)/timestep                                   #calculate decel necessary to reach target vel in 1 step
                acc = max(req_pos_acc,req_vel_acc)                                                  #prefer req_pos_acc until last step

        #calculate vector pos,vel,acc for linear moves
        if segments[id]['typ'] in ["G0","G1"]:
            perc = (dis-vertices[id]['dis'])/segments[id]['len']
            pos_vec = list( np.array(vertices[id]['pos'])*(1.0-perc) + np.array(vertices[id+1]['pos'])*perc )
            vel_vec = list( vel*np.array(segments[id]['aux']['dir']) )
            acc_vec = list( acc*np.array(segments[id]['aux']['dir']) )
        #calculate vector pos,vel,acc for arc moves
        elif segments[id]['typ'] in ["G2","G3"]:
            perc = (dis-vertices[id]['dis'])/segments[id]['len']
            ang = segments[id]['aux']['ang_beg'] + perc * (segments[id]['aux']['ang_end']-segments[id]['aux']['ang_beg'])
            rad_dir = np.array([math.cos(ang),math.sin(ang)])
            if segments[id]['typ']=="G2": tan_dir = np.array([math.sin(ang),-math.cos(ang)])
            if segments[id]['typ']=="G3": tan_dir = np.array([-math.sin(ang),math.cos(ang)])
            pos_vec = list( np.array(segments[id]['aux']['cnt']) + segments[id]['aux']['rad']*rad_dir )
            vel_vec = list( vel*tan_dir )
            acc_vec = list( acc*tan_dir - vel**2/segments[id]['aux']['rad']*rad_dir )

        #enforce stop if stop_cmd_queue contains stop commands
        if len(stop_cmd_queue)>0:
            vel = 0.0; vel_vec = [0.0, 0.0]
            acc = 0.0; acc_vec = [0.0, 0.0]

        #publish new setpoint
        gcode_publish(pos_vec, vel_vec, acc_vec)
        #print( str(tme)+";"+str(dis)+";"+str(vel)+";"+str(acc)+";"+str(pos_vec[0])+";"+str(pos_vec[1]) )
        rospy.sleep(timestep)

        #update dis, vel, tme for next iteration
        dis = dis + vel*timestep + 0.5*acc*timestep*timestep
        vel = vel + acc*timestep
        tme = tme + timestep

        #execute all commands in stop_cmd_queue
        while len(stop_cmd_queue)>0:
            stop_cmd = stop_cmd_queue.pop(0)
            if   stop_cmd == "exit":     return
            elif stop_cmd == "draw_on":  pub_draw.publish(True);  rospy.sleep(param_wait_draw); tme = tme + param_wait_draw
            elif stop_cmd == "draw_off": pub_draw.publish(False); rospy.sleep(param_wait_draw); tme = tme + param_wait_draw
            elif stop_cmd == "corner":   rospy.sleep(param_wait_corner); tme = tme + param_wait_corner


# publish setpoint given by gcode_run
def gcode_publish( pos, vel, acc):

    point_drone = MultiDOFJointTrajectoryPoint([Transform()], [Twist()], [Twist(),Twist()], rospy.Time(0))
    point_drone.transforms[0].translation.x = odom_drone_home.pose.pose.position.x + pos[0]/1000.0
    point_drone.transforms[0].translation.y = odom_drone_home.pose.pose.position.y + pos[1]/1000.0
    point_drone.transforms[0].translation.z = odom_drone_home.pose.pose.position.z
    point_drone.transforms[0].rotation = deepcopy(odom_drone_home.pose.pose.orientation)
    point_drone.velocities[0].linear.x = vel[0]/1000.0
    point_drone.velocities[0].linear.y = vel[1]/1000.0
    point_drone.accelerations[0].linear.x = acc[0]/1000.0
    point_drone.accelerations[0].linear.y = acc[1]/1000.0
    setpoint_drone = MultiDOFJointTrajectory()
    setpoint_drone.header.stamp = rospy.Time.now()
    setpoint_drone.header.frame_id ='world'
    setpoint_drone.joint_names = ['base_link']
    setpoint_drone.points.append(point_drone)
    pub_setpoint_drone.publish(setpoint_drone)

    setpoint_plate = Pose2D()
    setpoint_plate.x = odom_plate_home.x + pos[0]/1000.0
    setpoint_plate.y = odom_plate_home.y + pos[1]/1000.0
    setpoint_plate.theta = odom_plate_home.theta
    pub_setpoint_plate.publish(setpoint_plate)


################################################################################

if __name__ == '__main__':
    main()
