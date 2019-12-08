
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
  test_pose1.position.z = 0.45;
  move_group.setPoseTarget(test_pose1);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO("Visualizing test_pose 1 (pose goal) %s", success ? "" : "FAILED");

  move_group.move();

  ros::Duration(3.0).sleep(); // wait for plan to finish

  /*********************** Path Constraint test *******************************/
  ROS_WARN("Starting Path Constraint test...");

  // create a path constraint
  moveit_msgs::JointConstraint jc_wrist_angle;
  jc_wrist_angle.joint_name = "wx200_arm_A_wrist_angle";
  jc_wrist_angle.position = 0.0;
  jc_wrist_angle.tolerance_above = 0.1;
  jc_wrist_angle.tolerance_below = 0.1;
  jc_wrist_angle.weight = 0.5; // denotes relative importance 0-1

  moveit_msgs::JointConstraint jc_waist;
  jc_waist.joint_name = "wx200_arm_A_waist";
  jc_waist.position = 0.0;
  jc_waist.tolerance_above = 0.1;
  jc_waist.tolerance_below = 0.1;
  jc_waist.weight = 0.4; // denotes relative importance 0-1

  // set the path constraint for the move group
  moveit_msgs::Constraints test_constraints;
  test_constraints.joint_constraints.push_back(jc_wrist_angle);
  test_constraints.joint_constraints.push_back(jc_waist);
  move_group.setPathConstraints(test_constraints);

  // set the start state to a new pose
  robot_state::RobotState start_state(*move_group.getCurrentState());
  geometry_msgs::Pose test_pose2;
  test_pose2.orientation.w = 1.0;
  test_pose2.position.x = 0.15;
  test_pose2.position.y = 0.0;
  test_pose2.position.z = 0.25;
  // start_state.setFromIK(joint_model_group, test_pose2);
  move_group.setStartState(start_state);

  move_group.setPoseTarget(test_pose2);

  move_group.setPlanningTime(10.0);
  //
  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO("Visualizing plan 2 (constraints) %s", success ? "" : "FAILED");

  move_group.move();

  // move_group.clearPathConstraints();
  ROS_WARN("Done!");


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



  /******* Cartesian Path Test ***********************************************/
  // move_group.setPlanningTime(10.0);
  //
  // std::vector<geometry_msgs::Pose> waypoints;
  // waypoints.push_back(test_pose1);
  //
  // geometry_msgs::Pose test_pose2 = test_pose1;
  // test_pose2.position.z += 0.2;
  // waypoints.push_back(test_pose2);
  //
  // test_pose2.position.y -= 0.2;
  // waypoints.push_back(test_pose2);
  //
  // test_pose2.position.x += 0.15;
  // test_pose2.position.y += -0.15;
  // test_pose2.position.z -= 0.1;
  // waypoints.push_back(test_pose2);
  //
  // move_group.setMaxVelocityScalingFactor(0.1);
  //
  // moveit_msgs::RobotTrajectory trajectory;
  // const double jump_threshold = 0.01;
  // const double eef_step = 0.01;
  // double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  // ROS_INFO("Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);
  //
  // ROS_WARN("Moving!");
  // move_group.move();


  /************************** Wait till shutdown ******************************/
  while(ros::ok()){
    ;
  }

  return 0;
}
