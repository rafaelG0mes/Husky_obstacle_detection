cmake_minimum_required(VERSION 2.8.3)
project(cloud_processing)

## Find catkin macros and libraries
find_package(
  catkin REQUIRED COMPONENTS
  roscpp rospy
  pcl_conversions pcl_ros
  std_msgs sensor_msgs
  tf)

find_package(PCL 1.2 REQUIRED)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR})

## Declare a C++ executable
add_executable(
  object_classification
  src/obstacle_detection/obstacledetection_node.cpp
  src/obstacle_detection/object_classification.cpp
  src/obstacle_detection/filters.cpp
  src/obstacle_detection/avoidance_path.cpp)
target_link_libraries(
  object_classification
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${PCL_LIBRARIES})

add_library(
  object_classification_lib
  src/obstacle_detection/object_classification.cpp
  src/obstacle_detection/filters.cpp
  src/obstacle_detection/avoidance_path.cpp
  include/cloud_processing/config.h)
target_link_libraries(
  object_classification_lib
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${PCL_LIBRARIES})

## Add cmake target dependencies of the executable
add_dependencies(
  object_classification
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})

link_directories(${PCL_LIBRARY_DIRS})
add_definitions (${PCL_DEFINITIONS})





