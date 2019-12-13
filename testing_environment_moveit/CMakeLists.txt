cmake_minimum_required(VERSION 2.8.3)
project(testing_environment_moveit)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  message_filters
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_visual_tools
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)

catkin_package()
# CATKIN_DEPENDS roscpp sensor_msgs message_filters

include_directories(${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

## Declare a C++ executable
## Specify libraries to link a library or executable target against
## Add cmake target dependencies of the executable
add_executable(combine_arm_joints src/combine_arm_joints.cpp)
add_dependencies(combine_arm_joints ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})
target_link_libraries(combine_arm_joints ${catkin_LIBRARIES})

# motion_plan_test executable
add_executable(motion_plan_test src/motion_plan_test.cpp)
add_dependencies(motion_plan_test ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})
target_link_libraries(motion_plan_test ${catkin_LIBRARIES})

# move_group_test executable
add_executable(move_group_test src/move_group_test.cpp)
add_dependencies(move_group_test ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})
target_link_libraries(move_group_test ${catkin_LIBRARIES})

# pseudo-random poses executable
add_executable(pseudo_random_arm_poses src/pseudo_random_arm_poses.cpp)
add_dependencies(pseudo_random_arm_poses ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})
target_link_libraries(pseudo_random_arm_poses ${catkin_LIBRARIES})

# zero_gravity executable
add_executable(zero_gravity src/zero_gravity.cpp)
add_dependencies(zero_gravity ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})
target_link_libraries(zero_gravity ${catkin_LIBRARIES})

# shape_waypoints executable
add_executable(shape_waypoints src/shape_waypoints.cpp)
add_dependencies(shape_waypoints ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})
target_link_libraries(shape_waypoints ${catkin_LIBRARIES})

# manual_poses executable
add_executable(manual_poses src/manual_poses.cpp)
add_dependencies(manual_poses ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})
target_link_libraries(manual_poses ${catkin_LIBRARIES})

# run_through_waypoints executable
add_executable(run_through_waypoints src/run_through_waypoints.cpp)
add_dependencies(run_through_waypoints ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})
target_link_libraries(run_through_waypoints ${catkin_LIBRARIES})

# waypoint_1 executable
add_executable(waypoint_1 src/waypoint_1.cpp)
add_dependencies(waypoint_1 ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})
target_link_libraries(waypoint_1 ${catkin_LIBRARIES})

# waypoint_2 executable
add_executable(waypoint_2 src/waypoint_2.cpp)
add_dependencies(waypoint_2 ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})
target_link_libraries(waypoint_2 ${catkin_LIBRARIES})

# waypoint_3 executable
add_executable(waypoint_3 src/waypoint_3.cpp)
add_dependencies(waypoint_3 ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})
target_link_libraries(waypoint_3 ${catkin_LIBRARIES})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
