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
 INCLUDE_DIRS include
 LIBRARIES
)

###########
## Build ##
###########


add_library(factory src/factory.cc)
target_link_libraries(factory ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})


add_library(hello_world src/hello_world.cc)
target_link_libraries(hello_world ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(model_push src/model_push.cc)
target_link_libraries(model_push ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(system_gui src/system_gui.cc)
target_link_libraries(system_gui ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(world_edit src/world_edit.cc)
target_link_libraries(world_edit ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
