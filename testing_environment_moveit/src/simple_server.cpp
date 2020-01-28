#include "ros/ros.h"
#include "config/send_to_home.h" // will contain request/response types for this service

// import statements from move_group_test.cpp
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

// from move_group_test.cpp
static const std::string PLANNING_GROUP = "testing_environment";
moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
const robot_state::JointModelGroup* joint_model_group =
  move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

// service function
bool send_to_home(send_to_home::Request  &req, send_to_home::Response &res){
  move_arms_by_angle(req.arm_a_angles, req.arm_b_angles);

  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  res.position_arm_a = ; // copy joint_group_positions 0-7
  res.position_arm_b = ; // copy joint_group_positions 8-14

  ROS_INFO("request: send arms to home position");
  ROS_INFO("final arm joint angles (should be all 0.0): position_arm_a=%f position_arm_b=%f", res.position_arm_a, res.position_arm_b);
  return true;
}

// service helper
void move_arms_by_angle(float64[] arm_a_angles, float64[] arm_b_angles){

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "simple_server");
  ros::NodeHandle nh;
  ros::ServiceServer service = nh.advertiseService("send_to_home", send_to_home);
  ROS_INFO("Ready to send both arms to home position.");

  ros::spin();
  return 0;
}
