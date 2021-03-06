cmake_minimum_required(VERSION 2.8.3)
project(setup)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  costmap_2d
  nav_msgs
  tf
  geometry_msgs
  sensor_msgs
  cv_bridge
  std_msgs
  image_transport
  gazebo_plugins
  gazebo_ros
)

include_directories(include ${catkin_INCLUDE_DIRS})

## Find gazebo
find_package(gazebo REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

## Find OGRE and OGRE-Paging 
include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(OGRE OGRE)
  pkg_check_modules(OGRE-Paging OGRE-Paging)
endif()
include_directories(${OGRE_INCLUDE_DIRS} ${OGRE-Paging_INCLUDE_DIRS})
link_directories(${OGRE_LIBRARY_DIRS} ${OGRE-Paging_LIBRARY_DIRS})

## Find Boost
find_package(Boost REQUIRED COMPONENTS thread)
include_directories(${Boost_INCLUDE_DIRS})


###################################
## catkin specific configuration ##
###################################

catkin_package(
 CATKIN_DEPENDS
 nav_msgs
 roscpp
 rospy
 tf
 sensor_msgs
 cv_bridge
 std_msgs
 image_transport
 geometry_msgs
 costmap_2d
 gazebo_plugins
 gazebo_ros
 INCLUDE_DIRS 
 LIBRARIES
)

###########
## Build ##
###########


