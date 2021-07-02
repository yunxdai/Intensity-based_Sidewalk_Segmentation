cmake_minimum_required(VERSION 3.0.2)
project(lidar_road)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

set(
  PCL_PACKAGE_DEPENDENCIES
    pcl_conversions
    pcl_ros
)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  ${PCL_PACKAGE_DEPENDENCIES}
  # ${VELODYNE_PACKAGE_DEPENDENCIES}
)

## System dependencies are found with CMake's conventions
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)


catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES lidar_road
  CATKIN_DEPENDS
    roscpp
    sensor_msgs
    std_msgs
    ${PCL_PACKAGE_DEPENDENCIES}
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(
  ${PROJECT_NAME}_lib
  src/intensity_filter.cpp
)

add_executable(intensity_filter_node src/intensity_filter_node.cpp src/intensity_filter.cpp)


target_link_libraries(intensity_filter_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  # ${PROJECT_NAME}_lib
)

add_executable(ground_filter_node src/ground_filter_node.cpp src/ground_filter.cpp)


target_link_libraries(ground_filter_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  # ${PROJECT_NAME}_lib
)
