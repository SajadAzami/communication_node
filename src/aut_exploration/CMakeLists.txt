cmake_minimum_required(VERSION 2.8.3)
project(aut_exploration)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  laser_geometry
  costmap_2d
  geometry_msgs
  message_generation
  cv_bridge
  image_transport
  actionlib_msgs
  actionlib
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   Block_msgs.msg
   Object.msg
   Detections.msg
	 Master_toAgent.msg
	 Agent_toMaster.msg
)

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   FarGoal.srv
   Agent_service.srv
   Master_service.srv
   Percentage_service.srv
 )

## Generate actions in the 'action' folder
#add_action_files(
#  FILES
#   Master_toAgent.action
#  Agent_toMaster.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  nav_msgs  sensor_msgs   std_msgs  geometry_msgs actionlib_msgs )


###################################
## catkin specific configuration ##
###################################
catkin_package(
   CATKIN_DEPENDS nav_msgs roscpp actionlib rospy sensor_msgs actionlib_msgs std_msgs tf laser_geometry costmap_2d geometry_msgs message_runtime  cv_bridge image_transport
#  INCLUDE_DIRS include
#  LIBRARIES aut_exploration
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
