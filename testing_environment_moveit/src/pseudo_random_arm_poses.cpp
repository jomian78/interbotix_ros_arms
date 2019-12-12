
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

float random_float(float min, float max){
  float random = ((float) rand()) / (float) RAND_MAX;
  float diff = max - min;
  float r = random * diff;
  return min + r;
}

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

  /****************************************************************************
  *                  Coordinate frames to perch reference point               *
  *****************************************************************************/
  tf2::Vector3 wx200_arm_A_position(0.0,0.0,0.0);
  tf2::Quaternion wx200_arm_A_orientation(tf2::Vector3(0,0,1), 0.0);
  tf2::Transform global_to_wx200_arm_A(wx200_arm_A_orientation, wx200_arm_A_position);

  tf2::Vector3 wx200_arm_B_position(relative_x,relative_y,relative_z);
  tf2::Quaternion wx200_arm_B_orientation(tf2::Vector3(0,0,1), relative_yaw);
  tf2::Transform global_to_wx200_arm_B(wx200_arm_B_orientation, wx200_arm_B_position);

  tf2::Vector3 perch_position(perch_distance,relative_y/2.0,perch_height);
  tf2::Quaternion perch_orientation(tf2::Vector3(0,0,1), perch_yaw);
  tf2::Transform global_to_perch(perch_orientation, perch_position);

  /****************************************************************************
  *                  Generate pseudo_random points                            *
  *****************************************************************************/
  // seed the pseudo random number generator
  srand(time(0));

  // randomly generate a point for arm_A around the perch
  float wx200_arm_A_object_x = random_float(perch_min_x, perch_max_x);
  float wx200_arm_A_object_y = random_float(perch_min_y_A, perch_max_y_A);
  float wx200_arm_A_object_z = random_float(perch_min_z, perch_max_z);

  // randomly generate a point for arm_B around the perch
  float wx200_arm_B_object_x = random_float(perch_min_x, perch_max_x);
  float wx200_arm_B_object_y = random_float(perch_min_y_B, perch_max_y_B);
  float wx200_arm_B_object_z = random_float(perch_min_z, perch_max_z);

  // perch to object A
  tf2::Vector3 wx200_arm_A_object_position(wx200_arm_A_object_x,wx200_arm_A_object_y,wx200_arm_A_object_z);
  tf2::Quaternion wx200_arm_A_object_orientation(tf2::Vector3(0,0,1), 3.1415926);
  tf2::Transform perch_to_object_A(wx200_arm_A_object_orientation, wx200_arm_A_object_position);

  // perch to object B
  tf2::Vector3 wx200_arm_B_object_position(wx200_arm_B_object_x,wx200_arm_B_object_y,wx200_arm_B_object_z);
  tf2::Quaternion wx200_arm_B_object_orientation(tf2::Vector3(0,0,1), 3.1415926);
  tf2::Transform perch_to_object_B(wx200_arm_B_object_orientation, wx200_arm_B_object_position);

  // global transform to object A
  tf2::Transform global_to_object_A = global_to_perch*perch_to_object_A;

  // global transform to object B
  tf2::Transform global_to_object_B = global_to_perch*perch_to_object_B;

  // arm A position
  float object_A_position_x = global_to_object_A.getOrigin().x();
  float object_A_position_y = global_to_object_A.getOrigin().y();
  float object_A_position_z = global_to_object_A.getOrigin().z();
  move_group.setPositionTarget(object_A_position_x, object_A_position_y, object_A_position_z, "wx200_arm_A/ee_arm_link");
  // move_group.setJointValueTarget("wx200_arm_A_wrist_angle", 3.1415926/2.0);

  // arm B pose
  float object_B_position_x = global_to_object_B.getOrigin().x();
  float object_B_position_y = global_to_object_B.getOrigin().y();
  float object_B_position_z = global_to_object_B.getOrigin().z();
  move_group.setPositionTarget(object_B_position_x, object_B_position_y, object_B_position_z, "wx200_arm_B/ee_arm_link");
  // move_group.setJointValueTarget("wx200_arm_B_wrist_angle", 3.1415926/2.0);


  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO("Visualizing test_pose 1 and test_pose 2(pose goal) %s", success ? "" : "FAILED");

  geometry_msgs::PoseStamped object_A_stamped_pose;
  geometry_msgs::PoseStamped object_B_stamped_pose;
  object_A_stamped_pose = move_group.getPoseTarget("wx200_arm_A/ee_arm_link");
  object_B_stamped_pose = move_group.getPoseTarget("wx200_arm_B/ee_arm_link");

  visual_tools.publishAxisLabeled(object_A_stamped_pose.pose, "object_A pose");
  visual_tools.publishAxisLabeled(object_B_stamped_pose.pose, "object_B pose");
  visual_tools.publishText(text_pose, "Pose Goals", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();

  move_group.move();

  // adding more waypoints
  object_A_position_x = 0.3;
  object_A_position_y = -0.2;
  object_A_position_z = 0.2;
  move_group.setPositionTarget(object_A_position_x, object_A_position_y, object_A_position_z, "wx200_arm_A/ee_arm_link");

  object_B_position_x = 0.3;
  object_B_position_y = 0.2;
  object_B_position_z = 0.2;
  move_group.setPositionTarget(object_B_position_x, object_B_position_y, object_B_position_z, "wx200_arm_B/ee_arm_link");
  move_group.move();

  object_A_position_x = 0.15;
  object_A_position_y = 0.15;
  object_A_position_z = 0.2;
  move_group.setPositionTarget(object_A_position_x, object_A_position_y, object_A_position_z, "wx200_arm_A/ee_arm_link");

  object_B_position_x = 0.20;
  object_B_position_y = 0.05;
  object_B_position_z = 0.2;
  move_group.setPositionTarget(object_B_position_x, object_B_position_y, object_B_position_z, "wx200_arm_B/ee_arm_link");
  move_group.move();

  object_A_position_x = 0.20;
  object_A_position_y = -0.1;
  object_A_position_z = 0.35;
  move_group.setPositionTarget(object_A_position_x, object_A_position_y, object_A_position_z, "wx200_arm_A/ee_arm_link");

  object_B_position_x = 0.15;
  object_B_position_y = 0.3;
  object_B_position_z = 0.35;
  move_group.setPositionTarget(object_B_position_x, object_B_position_y, object_B_position_z, "wx200_arm_B/ee_arm_link");
  move_group.move();

  object_A_position_x = 0.25;
  object_A_position_y = 0.15;
  object_A_position_z = 0.25;
  move_group.setPositionTarget(object_A_position_x, object_A_position_y, object_A_position_z, "wx200_arm_A/ee_arm_link");

  object_B_position_x = 0.10;
  object_B_position_y = 0.02;
  object_B_position_z = 0.05;
  move_group.setPositionTarget(object_B_position_x, object_B_position_y, object_B_position_z, "wx200_arm_B/ee_arm_link");
  move_group.move();

  object_A_position_x = 0.20;
  object_A_position_y = -0.05;
  object_A_position_z = 0.15;
  move_group.setPositionTarget(object_A_position_x, object_A_position_y, object_A_position_z, "wx200_arm_A/ee_arm_link");

  object_B_position_x = 0.15;
  object_B_position_y = 0.05;
  object_B_position_z = 0.15;
  move_group.setPositionTarget(object_B_position_x, object_B_position_y, object_B_position_z, "wx200_arm_B/ee_arm_link");
  move_group.move();



  // wait for plan to finish
  ros::Duration(5.0).sleep();

  /*********************** Wrist Constraints *******************************/
  // moveit_msgs::JointConstraint arm_A_joint_constraint;
  // arm_A_joint_constraint.joint_name = "wx200_arm_A_wrist_angle";
  // arm_A_joint_constraint.position = 3.1415926/2.0;
  // arm_A_joint_constraint.tolerance_above = 0.01;
  // arm_A_joint_constraint.tolerance_below = 0.01;
  // arm_A_joint_constraint.weight = 0.5; // denotes relative importance 0-1
  //
  // moveit_msgs::JointConstraint arm_B_joint_constraint;
  // arm_B_joint_constraint.joint_name = "wx200_arm_B_wrist_angle";
  // arm_B_joint_constraint.position = 3.1415926/2.0;
  // arm_B_joint_constraint.tolerance_above = 0.01;
  // arm_B_joint_constraint.tolerance_below = 0.01;
  // arm_B_joint_constraint.weight = 0.5; // denotes relative importance 0-1
  //
  // moveit_msgs::Constraints arm_constraints;
  // arm_constraints.joint_constraints.push_back(arm_A_joint_constraint);
  // arm_constraints.joint_constraints.push_back(arm_B_joint_constraint);
  // move_group.setPathConstraints(arm_constraints);
  //
  // move_group.setPlanningTime(10.0);
  //
  // // clear path constraints
  // ROS_WARN("Done!");
  // ROS_WARN("Clearing path constraints...");
  // move_group.clearPathConstraints();

  /************************** Wait till shutdown ******************************/
  while(ros::ok()){
    ;
  }

  return 0;
}
