#!/bin/bash

DAY="$(date +"%Y-%m-%d")"
TIME="$(date +"%I-%M-%S")"

cd
mkdir -p "bags/aerial_layouting/aerial_layouting_$DAY"
cd "bags/aerial_layouting/aerial_layouting_$DAY"

rosparam dump |& tee "$DAY-$TIME.param"

rosbag record \
/ceiling/vrpn_client/estimated_odometry \
/plate/vrpn_client/estimated_odometry \
/marker/odometry_plate \
/marker/odometry_plate_abs \
/marker/odometry_plate_rel \
/marker/setpoint_plate \
/marker/control_plate \
/marker/draw \
/ouzel/command/trajectory \
/ouzel/command/current_reference \
/ouzel/wrench_target \
/ouzel/position_target \
/ouzel/attitude_target \
/ouzel/mavros/imu/data_raw \
/ouzel/mavros/imu/mag \
/ouzel/mavros/rc/in \
/ouzel/mavros/rc/out \
/ouzel/msf_core/odometry \
/ouzel/vrpn_client/estimated_odometry \
/ouzel/state_machine/state_info
