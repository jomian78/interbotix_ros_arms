cmake_minimum_required(VERSION 2.8.3)
project(testing_environment_moveit)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  message_filters
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

catkin_package()
# CATKIN_DEPENDS roscpp sensor_msgs message_filters

include_directories(${catkin_INCLUDE_DIRS})

## Declare a C++ executable
## Specify libraries to link a library or executable target against
## Add cmake target dependencies of the executable
add_executable(combine_arm_joints src/combine_arm_joints.cpp)
add_dependencies(combine_arm_joints ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})
target_link_libraries(combine_arm_joints ${catkin_LIBRARIES})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
