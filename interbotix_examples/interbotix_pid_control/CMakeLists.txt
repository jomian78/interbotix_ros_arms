cmake_minimum_required(VERSION 2.8.3)
project(interbotix_pid_control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  interbotix_sdk
  roscpp
  sensor_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
catkin_package(
 CATKIN_DEPENDS interbotix_sdk roscpp sensor_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
## Specify libraries to link a library or executable target against
## Add cmake target dependencies of the executable
add_executable(pid_control_node src/pid_control_node.cpp)
target_link_libraries(pid_control_node ${catkin_LIBRARIES})
add_dependencies(pid_control_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
