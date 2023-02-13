cmake_minimum_required(VERSION 3.0.2)
project(hpe_udp_ctl)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    hpe_ros_msgs
    std_msgs
    roscpp
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}_lib
  CATKIN_DEPENDS
    hpe_ros_msgs
    std_msgs
    roscpp
#  CATKIN_DEPENDS other_catkin_pkg
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
)


set(PROJECT_LIB_FILES 
    src/hpe2udp.cpp
)

## Declare a C++ library
add_library(
        ${PROJECT_NAME}_lib
        ${PROJECT_LIB_FILES}
)

add_executable(
        ${PROJECT_NAME}_node
        src/hpe2udp_node.cpp
)

target_link_libraries(
        ${PROJECT_NAME}_node
        ${PROJECT_NAME}_lib
        ${catkin_LIBRARIES}
)