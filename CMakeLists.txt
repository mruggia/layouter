cmake_minimum_required(VERSION 3.0.2)
project(marker)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
  FILES
  Setpoint.msg
  Draw.msg
)

generate_messages(
 DEPENDENCIES
 std_msgs
 geometry_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  LIBRARIES marker
  CATKIN_DEPENDS
    rospy
    std_msgs
    geometry_msgs
    message_runtime
)

#############
## Install ##
#############

catkin_install_python(
  PROGRAMS
    scripts/sim_circ.py
    scripts/main.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
