
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_test");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  /********************* MoveGroupInterface test *****************************/

  static const std::string PLANNING_GROUP = "testing_environment";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  // for adding and removing collision objects
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  const robot_state::JointModelGroup* joint_model_group =
    move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  geometry_msgs::Pose test_pose1;
  test_pose1.orientation.w = 1.0;
  test_pose1.position.x = 0.5;
  test_pose1.position.y = 0.0;
  test_pose1.position.z = 0.5;
  move_group.setPoseTarget(test_pose1);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;


  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO("Visualizing test_pose 1 (pose goal) %s", success ? "" : "FAILED");
  /****************************************************************************/

  /********************* Second test ******************************************/
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
  ROS_WARN("Line 48");

  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
  ROS_WARN("Line 52");

  joint_group_positions[0] = -1.0;
  move_group.setJointValueTarget(joint_group_positions);
  ROS_WARN("Line 56");

  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO("Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");
  ROS_WARN("Line 60");

  // move_group.move();
  ROS_WARN("Line 63");

  /****************************************************************************/

  // keep the node running
  while(ros::ok()){
    ;
  }

  return 0;
}
