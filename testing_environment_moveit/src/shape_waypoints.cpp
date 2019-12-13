
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

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
  nh.param<float>("/testing_environment/pseudo_random_arm_poses/relative_y", relative_y, 0.24);
  nh.param<float>("/testing_environment/pseudo_random_arm_poses/relative_z", relative_z, 0.0);
  nh.param<float>("/testing_environment/pseudo_random_arm_poses/relative_roll", relative_roll, 0.0);
  nh.param<float>("/testing_environment/pseudo_random_arm_poses/relative_pitch", relative_pitch, 0.0);
  nh.param<float>("/testing_environment/pseudo_random_arm_poses/relative_yaw", relative_yaw, 0.0);
  ROS_WARN("relative_y value: %f", relative_y);

  // perch distance and yaw
  float perch_distance = 0.4;
  float perch_yaw = 3.1415926;
  float perch_height = 0.65/2.0;

  // perch sphere parameters
  float perch_max_x = 0.20;
  float perch_min_x = 0.05;

  float perch_max_y_A = 0.20;
  float perch_min_y_A = 0.05;
  float perch_max_y_B = -0.05;
  float perch_min_y_B = -0.20;

  float perch_max_z = 0.10;
  float perch_min_z = -0.10;

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
  box_pose.position.x = perch_distance;
  box_pose.position.y = relative_y/2.0;
  box_pose.position.z = 0.65/4.0;

  perch.primitives.push_back(primitive);
  perch.primitive_poses.push_back(box_pose);
  perch.operation = perch.ADD;

  /******* Add these collision objects to the planning scene *******/
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(wx200_arm_A_table);
  collision_objects.push_back(wx200_arm_B_table);
  collision_objects.push_back(perch);
  planning_scene_interface.addCollisionObjects(collision_objects);
  visual_tools.trigger();

  /**** Square waypoints plan **********************************/
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  // Point 1
  double object_A_position_x = 0.3;
  double object_A_position_y = 0.0;
  double object_A_position_z = 0.1;
  move_group.setPositionTarget(object_A_position_x, object_A_position_y, object_A_position_z, "wx200_arm_A/ee_arm_link");

  double object_B_position_x = 0.3;
  double object_B_position_y = 0.24;
  double object_B_position_z = 0.3;
  move_group.setPositionTarget(object_B_position_x, object_B_position_y, object_B_position_z, "wx200_arm_B/ee_arm_link");

  // bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  // ROS_INFO("Visualizing test_pose 1 and test_pose 2(pose goal) %s", success ? "" : "FAILED");

  // geometry_msgs::PoseStamped object_A_stamped_pose;
  // geometry_msgs::PoseStamped object_B_stamped_pose;
  // object_A_stamped_pose = move_group.getPoseTarget("wx200_arm_A/ee_arm_link");
  // object_B_stamped_pose = move_group.getPoseTarget("wx200_arm_B/ee_arm_link");

  // visual_tools.publishAxisLabeled(object_A_stamped_pose.pose, "object_A pose");
  // visual_tools.publishAxisLabeled(object_B_stamped_pose.pose, "object_B pose");
  // visual_tools.publishText(text_pose, "Pose Goals", rvt::WHITE, rvt::XLARGE);
  // visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  // visual_tools.trigger();

  move_group.move();

  // Point 2
  object_A_position_x = 0.3;
  object_A_position_y = 0.0;
  object_A_position_z = 0.3;
  move_group.setPositionTarget(object_A_position_x, object_A_position_y, object_A_position_z, "wx200_arm_A/ee_arm_link");

  object_B_position_x = 0.3;
  object_B_position_y = 0.24;
  object_B_position_z = 0.1;
  move_group.setPositionTarget(object_B_position_x, object_B_position_y, object_B_position_z, "wx200_arm_B/ee_arm_link");
  move_group.move();
  // Point 3
  object_A_position_x = 0.3;
  object_A_position_y = 0.24;
  object_A_position_z = 0.3;
  move_group.setPositionTarget(object_A_position_x, object_A_position_y, object_A_position_z, "wx200_arm_A/ee_arm_link");

  object_B_position_x = 0.3;
  object_B_position_y = 0.0;
  object_B_position_z = 0.1;
  move_group.setPositionTarget(object_B_position_x, object_B_position_y, object_B_position_z, "wx200_arm_B/ee_arm_link");
  move_group.move();
  // Point 4
  object_A_position_x = 0.3;
  object_A_position_y = 0.24;
  object_A_position_z = 0.1;
  move_group.setPositionTarget(object_A_position_x, object_A_position_y, object_A_position_z, "wx200_arm_A/ee_arm_link");

  object_B_position_x = 0.3;
  object_B_position_y = 0.0;
  object_B_position_z = 0.3;
  move_group.setPositionTarget(object_B_position_x, object_B_position_y, object_B_position_z, "wx200_arm_B/ee_arm_link");
  move_group.move();
  // wait for plan to finish
  ros::Duration(5.0).sleep();

  // clear path constraints
  ROS_WARN("Done!");

  /************************** Wait till shutdown ******************************/
  while(ros::ok()){
    ;
  }

  return 0;
}
