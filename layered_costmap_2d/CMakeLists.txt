cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE Release)

rosbuild_init()
rosbuild_genmsg()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


rosbuild_add_library(layers
  plugins/inflation_layer.cpp
  plugins/obstacle_layer.cpp
  plugins/static_layer.cpp
  # plugins/voxel_layer.cpp # todo fix conversion to old stile
  src/observation_buffer.cpp
)

rosbuild_add_library(layered_costmap_2d   
  src/array_parser.cpp
  src/costmap_2d.cpp
  src/observation_buffer.cpp
  src/layer.cpp
  src/layered_costmap.cpp
  src/costmap_2d_ros.cpp
  src/costmap_2d_publisher.cpp
  src/costmap_math.cpp
  src/footprint.cpp
  src/costmap_layer.cpp
)

target_link_libraries(layers layered_costmap_2d )
rosbuild_link_boost(layered_costmap_2d system)

# make dynamic reconfigure api available
rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()

find_package(Eigen)
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

rosbuild_add_executable(costmap_2d_markers src/costmap_2d_markers.cpp)
target_link_libraries(costmap_2d_markers layered_costmap_2d)
rosbuild_add_executable(costmap_2d_cloud src/costmap_2d_cloud.cpp)
target_link_libraries(costmap_2d_cloud layered_costmap_2d )
rosbuild_add_executable(costmap_2d_node src/costmap_2d_node.cpp)
target_link_libraries(costmap_2d_node layered_costmap_2d )


