cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

set(ROS_BUILD_TYPE Debug)
# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/activityLayer.cpp src/activityMap.cpp src/activityMapComponent.cpp)
#include_directories(include ${layered_costmap_2d})
#target_link_libraries(${PROJECT_NAME} layered_costmap_2d)

rosbuild_add_library(activity_layer
  src/activity_layer.cpp
  src/pmac_cell.cpp
  src/probablistic_cell.cpp
  src/probabilistic_filter.cpp
  src/pmac_learner.cpp
 # src/activityMap.cpp
  #src/activityMapComponent.cpp
)
include_directories(include 
  ${layered_costmap_2d}
)
target_link_libraries(activity_layer 
  layered_costmap_2d 
)
