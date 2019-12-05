
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

  // static const std::string PLANNING_GROUP = "testing_environment";
  static const std::string PLANNING_GROUP = "wx200_arm_A/arm_controller";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);


  // for adding and removing collision objects
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  const robot_state::JointModelGroup* joint_model_group =
    move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  // print out end-effector link for this group
  ROS_INFO("End effector link: %s", move_group.getEndEffectorLink().c_str());

  geometry_msgs::Pose test_pose1;
  test_pose1.orientation.w = 1.0;
  test_pose1.position.x = 0.0;
  test_pose1.position.y = 0.0;
  test_pose1.position.z = 0.28;
  move_group.setPoseTarget(test_pose1);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO("Visualizing test_pose 1 (pose goal) %s", success ? "" : "FAILED");

  move_group.move();

  /********************* Second test ******************************************/
  // moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
  //
  // std::vector<double> joint_group_positions;
  // current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
  //
  // // new pose in joint space
  // joint_group_positions[0] = -1.0;
  //
  // move_group.setJointValueTarget(joint_group_positions);
  //
  // bool success;
  // success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  // ROS_INFO("Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");
  // //
  // move_group.move();

  /******* CollisionObject Test ***********************************************/
  // moveit_msgs::CollisionObject collision_object;
  // collision_object.header.frame_id = move_group.getPlanningFrame();
  //
  // collision_object.id = "box1";
  //
  // shape_msgs::SolidPrimitive primitive;
  // primitive.type = primitive.BOX;
  // primitive.dimensions.resize(3);
  // primitive.dimensions[0] = 0.1;
  // primitive.dimensions[1] = 0.3;
  // primitive.dimensions[2] = 0.1;
  //
  // geometry_msgs::Pose box_pose;
  // box_pose.orientation.w = 1.0;
  // box_pose.position.x = 0.3;
  // box_pose.position.y = 0.0;
  // box_pose.position.z = 0.15;
  //
  // collision_object.primitives.push_back(primitive);
  // collision_object.primitive_poses.push_back(box_pose);
  // collision_object.operation = collision_object.ADD;
  //
  // std::vector<moveit_msgs::CollisionObject> collision_objects;
  // collision_objects.push_back(collision_object);
  //
  // planning_scene_interface.addCollisionObjects(collision_objects);

  /************************** Wait till shutdown ******************************/
  while(ros::ok()){
    ;
  }

  return 0;
}
