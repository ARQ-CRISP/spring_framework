cmake_minimum_required(VERSION 2.8.3)
project(spring_framework)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  eigen_conversions
  kdl_conversions
  kdl_control_tools
  roscpp
  message_generation
  tf
)

## System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)
find_package(orocos_kdl REQUIRED)

# For YouCompleteMe
set( CMAKE_EXPORT_COMPILE_COMMANDS ON )

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
  FILES
  VirtualSpringMsg.msg
  SpringNetworkMsg.msg
)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

###################################
## catkin specific configuration ##
###################################

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES spring_framework
 CATKIN_DEPENDS
    eigen_conversions
    kdl_conversions
    kdl_control_tools
    roscpp
    message_runtime
    tf
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
  ${EIGEN3_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/spring_control.cpp
  src/spring_display.cpp
  src/virtual_spring.cpp
  src/spring_network.cpp
  src/spring_server.cpp
)

## executables
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
)
#############
## Install ##
#############



#############
## Testing ##
#############
