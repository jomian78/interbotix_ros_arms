
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

  // retrieve relative distances between bases
  float relative_x;
  float relative_y;
  float relative_z;
  float relative_roll;
  float relative_pitch;
  float relative_yaw;
  nh.param<float>("/testing_environment/pseudo_random_arm_poses/relative_x", relative_x, 0.0);
  nh.param<float>("/testing_environment/pseudo_random_arm_poses/relative_y", relative_y, 0.50);
  nh.param<float>("/testing_environment/pseudo_random_arm_poses/relative_z", relative_z, 0.0);
  nh.param<float>("/testing_environment/pseudo_random_arm_poses/relative_roll", relative_roll, 0.0);
  nh.param<float>("/testing_environment/pseudo_random_arm_poses/relative_pitch", relative_pitch, 0.0);
  nh.param<float>("/testing_environment/pseudo_random_arm_poses/relative_yaw", relative_yaw, 0.0);
  ROS_WARN("relative_y value: %f", relative_y);

  // wait for everything to initialize
  ros::Duration(3.0).sleep();

  // initialize our planning group
  static const std::string PLANNING_GROUP = "testing_environment";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  const robot_state::JointModelGroup* joint_model_group =
    move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  // moveit visual tools
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("link0");
  visual_tools.deleteAllMarkers();
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 0.7;
  visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  /****************************************************************************
  *                  Declaring Collision Objects                               *
  *****************************************************************************/

  /************** ground plane arm A ********************/
  moveit_msgs::CollisionObject wx200_arm_A_table;
  wx200_arm_A_table.header.frame_id = move_group.getPlanningFrame();

  wx200_arm_A_table.id = "wx200_arm_A_table";

  shape_msgs::SolidPrimitive table_A;
  table_A.type = table_A.BOX;
  table_A.dimensions.resize(3);
  table_A.dimensions[0] = 2.0;
  table_A.dimensions[1] = 2.0;
  table_A.dimensions[2] = 0.01;

  geometry_msgs::Pose table_arm_A_pose;
  table_arm_A_pose.orientation.w = 1.0;
  table_arm_A_pose.position.x = 0.0;
  table_arm_A_pose.position.y = 0.0;
  table_arm_A_pose.position.z = -0.005;

  wx200_arm_A_table.primitives.push_back(table_A);
  wx200_arm_A_table.primitive_poses.push_back(table_arm_A_pose);
  wx200_arm_A_table.operation = wx200_arm_A_table.ADD;

  /*********** ground plane arm B **********************************/
  moveit_msgs::CollisionObject wx200_arm_B_table;
  wx200_arm_B_table.header.frame_id = move_group.getPlanningFrame();

  wx200_arm_B_table.id = "wx200_arm_B_table";

  shape_msgs::SolidPrimitive table_B;
  table_B.type = table_B.BOX;
  table_B.dimensions.resize(3);
  table_B.dimensions[0] = 2.0;
  table_B.dimensions[1] = 2.0;
  table_B.dimensions[2] = 0.01;

  geometry_msgs::Pose table_arm_B_pose;
  table_arm_B_pose.orientation.w = 1.0;
  table_arm_B_pose.position.x = relative_x;
  table_arm_B_pose.position.y = relative_y;
  table_arm_B_pose.position.z = -0.005;

  wx200_arm_B_table.primitives.push_back(table_B);
  wx200_arm_B_table.primitive_poses.push_back(table_arm_B_pose);
  wx200_arm_B_table.operation = wx200_arm_B_table.ADD;

  /************** rodent perch *******************************/
  moveit_msgs::CollisionObject perch;
  perch.header.frame_id = move_group.getPlanningFrame();

  perch.id = "box1";

  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.1;
  primitive.dimensions[1] = 0.1;
  primitive.dimensions[2] = 0.65/2.0;

  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 0.3;
  box_pose.position.y = relative_y/2.0;
  box_pose.position.z = 0.65/4.0;

  perch.primitives.push_back(primitive);
  perch.primitive_poses.push_back(box_pose);
  perch.operation = perch.ADD;

  /******* Add these collision objects to the planning scene *****************/
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(wx200_arm_A_table);
  collision_objects.push_back(wx200_arm_B_table);
  collision_objects.push_back(perch);
  planning_scene_interface.addCollisionObjects(collision_objects);

  /****************************************************************************
  *                  Pseudo-random poses                                      *
  *****************************************************************************/
  // arm A pose
  geometry_msgs::Pose test_pose1;
  test_pose1.orientation.w = 1.0;
  test_pose1.position.x = 0.0;
  test_pose1.position.y = 0.0;
  test_pose1.position.z = 0.45;
  move_group.setPoseTarget(test_pose1, "wx200_arm_A/ee_arm_link");

  // arm B pose
  geometry_msgs::Pose test_pose2;
  test_pose2.orientation.w = 1.0;
  test_pose2.position.x = 0.0;
  test_pose2.position.y = 0.25;
  test_pose2.position.z = 0.35;
  move_group.setPoseTarget(test_pose2, "wx200_arm_B/ee_arm_link");

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO("Visualizing test_pose 1 and test_pose 2(pose goal) %s", success ? "" : "FAILED");

  visual_tools.publishAxisLabeled(test_pose1, "wx200_arm_A pose");
  visual_tools.publishAxisLabeled(test_pose2, "wx200_arm_B pose");
  visual_tools.publishText(text_pose, "Pose Goals", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();

  move_group.move();

  geometry_msgs::Pose test_pose3;
  test_pose3.orientation.w = 1.0;
  test_pose3.position.x = 0.0;
  test_pose3.position.y = 0.0;
  test_pose3.position.z = 0.35;
  move_group.setPoseTarget(test_pose3, "wx200_arm_A/ee_arm_link");

  geometry_msgs::Pose test_pose4;
  test_pose4.orientation.w = 1.0;
  test_pose4.position.x = 0.0;
  test_pose4.position.y = 0.25;
  test_pose4.position.z = 0.45;
  move_group.setPoseTarget(test_pose4, "wx200_arm_B/ee_arm_link");

  move_group.move();

  move_group.setPoseTarget(test_pose1, "wx200_arm_A/ee_arm_link");
  move_group.setPoseTarget(test_pose2, "wx200_arm_B/ee_arm_link");
  move_group.move();

  // wait for plan to finish
  ros::Duration(5.0).sleep();

  /*********************** Path Constraint test *******************************/
  // ROS_WARN("Starting Path Constraint test...");
  //
  // // create a path constraint
  // moveit_msgs::JointConstraint jc_wrist_angle;
  // jc_wrist_angle.joint_name = "wx200_arm_A_wrist_angle";
  // jc_wrist_angle.position = 0.0;
  // jc_wrist_angle.tolerance_above = 0.1;
  // jc_wrist_angle.tolerance_below = 0.1;
  // jc_wrist_angle.weight = 0.5; // denotes relative importance 0-1
  //
  // moveit_msgs::JointConstraint jc_waist;
  // jc_waist.joint_name = "wx200_arm_A_waist";
  // jc_waist.position = 0.0;
  // jc_waist.tolerance_above = 0.1;
  // jc_waist.tolerance_below = 0.1;
  // jc_waist.weight = 0.4; // denotes relative importance 0-1
  //
  // // set the path constraint for the move group
  // moveit_msgs::Constraints test_constraints;
  // test_constraints.joint_constraints.push_back(jc_wrist_angle);
  // test_constraints.joint_constraints.push_back(jc_waist);
  // move_group.setPathConstraints(test_constraints);
  //
  // // set the start state to a new pose
  // robot_state::RobotState start_state(*move_group.getCurrentState());
  // geometry_msgs::Pose test_pose3;
  // test_pose3.orientation.w = 1.0;
  // test_pose3.position.x = 0.15;
  // test_pose3.position.y = 0.0;
  // test_pose3.position.z = 0.45;
  // // start_state.setFromIK(joint_model_group, test_pose2);
  // move_group.setStartState(start_state);
  //
  // move_group.setPoseTarget(test_pose3, "wx200_arm_A/ee_arm_link");
  //
  // move_group.setPlanningTime(10.0);
  // //  // move_group.setStartState(start_state);

  // success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  // ROS_INFO("Visualizing plan 3 (constraints) %s", success ? "" : "FAILED");
  //
  // move_group.move();
  //
  // ROS_WARN("Done!");
  // ROS_WARN("Clearing path constraints...");
  // move_group.clearPathConstraints();

  /************************** Wait till shutdown ******************************/
  while(ros::ok()){
    ;
  }

  return 0;
}
