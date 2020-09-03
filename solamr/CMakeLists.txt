 cmake_minimum_required(VERSION 2.8.3)
project(solamr)

### Find catkin macros and libraries ###
find_package(catkin REQUIRED COMPONENTS 
	roscpp
	rospy 
	geometry_msgs 
	joy
	tf
	std_msgs
	visualization_msgs
	move_base
	    )

add_service_files(
   FILES
   StringSrv.srv
)
generate_messages(DEPENDENCIES std_msgs geometry_msgs)


### Add support for c++11, supported in ROS Kinetic and later version ###
#add_definitions(-std=c+11)

include_directories(${catkin_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS roscpp geometry_msgs joy
  DEPENDS
)

###########
## Build ##
###########



###--=============--###
###--pp_controller--###
###--=============--###
#add_executable(pp_controller scripts/pp_controller/pp_controller.cpp 
#                             scripts/pp_controller/pp_controller.h)
#target_link_libraries(pp_controller ${catkin_LIBRARIES})
#add_dependencies(pp_controller drogo_onBoard_generate_messages_cpp)

## Uncomment if the package has a setup.py
#catkin_python_setup()

catkin_install_python(PROGRAMS
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
#install(PROGRAMS
#  scripts/jetsoncar_teleop_key
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

## Mark executables and/or libraries for installation
#install(TARGETS SOLabot_teleop_joy
# RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

## Mark all other useful stuff for installation
#install(DIRECTORY launch
# DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)
