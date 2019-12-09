
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
  ros::Duration(3.0).sleep(); // wait for everything to initialize


  static const std::string PLANNING_GROUP = "testing_environment";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  // for adding and removing collision objects
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  //MoveIt Visual tools
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("link0");
  visual_tools.deleteAllMarkers();
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 0.7;
  visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  /************************ CollisionObject Test ******************************/
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

  // table A
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

  // table B
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
  table_arm_B_pose.position.x = 0.0;
  table_arm_B_pose.position.y = 0.22;
  table_arm_B_pose.position.z = -0.005;

  wx200_arm_B_table.primitives.push_back(table_B);
  wx200_arm_B_table.primitive_poses.push_back(table_arm_B_pose);
  wx200_arm_B_table.operation = wx200_arm_B_table.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(wx200_arm_A_table);
  collision_objects.push_back(wx200_arm_B_table);
  // collision_objects.push_back(collision_object);

  planning_scene_interface.addCollisionObjects(collision_objects);


  // /********************* MoveGroupInterface test *****************************/
  const robot_state::JointModelGroup* joint_model_group =
    move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  // print out end-effector link for this group
  // move_group.setEndEffectorLink("wx200_arm_A/ee_arm_link");
  // ROS_INFO("End effector link: %s", move_group.getEndEffectorLink().c_str());

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
  test_pose2.position.y = 0.22;
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
  test_pose4.position.y = 0.22;
  test_pose4.position.z = 0.45;
  move_group.setPoseTarget(test_pose4, "wx200_arm_B/ee_arm_link");

  move_group.move();

  move_group.setPoseTarget(test_pose1, "wx200_arm_A/ee_arm_link");
  move_group.setPoseTarget(test_pose2, "wx200_arm_B/ee_arm_link");
  move_group.move();

  // int cntr = 1;
  // while(ros::ok()){
  //   if (cntr % 2 == 1){
  //     move_group.setPoseTarget(test_pose1, "wx200_arm_A/ee_arm_link");
  //     move_group.setPoseTarget(test_pose2, "wx200_arm_B/ee_arm_link");
  //   }
  //   else{
  //     move_group.setPoseTarget(test_pose3, "wx200_arm_A/ee_arm_link");
  //     move_group.setPoseTarget(test_pose4, "wx200_arm_B/ee_arm_link");
  //   }
  //   move_group.move();
  //   cntr++;
  // }


  ros::Duration(5.0).sleep(); // wait for plan to finish

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

  /********************* Joint space test ******************************************/
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();

  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  // new pose in joint space
  // joint_group_positions[1] = 1.0;
  joint_group_positions[7] = 1.0;

  move_group.setJointValueTarget(joint_group_positions);

  // bool success;
  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO("Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");

  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();

  move_group.move();

  /******* Cartesian Path Test ***********************************************/
  // move_group.setPlanningTime(10.0);
  // ROS_WARN("Starting waypoints test!");
  //
  // robot_state::RobotState start_state(*move_group.getCurrentState());
  // move_group.setStartState(start_state);
  //
  // std::vector<geometry_msgs::Pose> waypoints;
  // geometry_msgs::Pose test_pose3;
  // test_pose3.orientation.w = 1.0;
  // test_pose3.position.x = 0.15;
  // test_pose3.position.y = 0.22;
  // test_pose3.position.z = 0.45;
  // waypoints.push_back(test_pose3);
  //
  // geometry_msgs::Pose test_pose4;
  // test_pose4.orientation.w = 1.0;
  // test_pose4.position.x = 0.15;
  // test_pose4.position.y = 0.32;
  // test_pose4.position.z = 0.45;
  // waypoints.push_back(test_pose4);
  //
  // geometry_msgs::Pose test_pose5;
  // test_pose5.orientation.w = 1.0;
  // test_pose5.position.x = 0.15;
  // test_pose5.position.y = 0.32;
  // test_pose5.position.z = 0.35;
  // waypoints.push_back(test_pose5);
  //
  // move_group.setMaxVelocityScalingFactor(0.1);
  //
  // moveit_msgs::RobotTrajectory trajectory;
  // const double jump_threshold = 0.0;
  // const double eef_step = 0.01;
  // double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  // ROS_INFO("Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);
  //
  // ROS_WARN("Moving!");
  //
  // // arm A
  // static const std::string PLANNING_GROUP_2 = "wx200_arm_A/arm_controller";
  // moveit::planning_interface::MoveGroupInterface move_group_2(PLANNING_GROUP_2);
  // geometry_msgs::Pose first_A;
  // first_A.orientation.w = 1.0;
  // first_A.position.x = 0.0;
  // first_A.position.y = 0.0;
  // first_A.position.z = 0.35;
  // move_group_2.setPoseTarget(first_A, "wx200_arm_A/ee_arm_link");
  // move_group_2.move();
  //
  // move_group_2.setPlanningTime(10.0);
  // robot_state::RobotState start_state_2(*move_group_2.getCurrentState());
  // move_group_2.setStartState(start_state_2);
  //
  // std::vector<geometry_msgs::Pose> waypoints2;
  // geometry_msgs::Pose test_pose6;
  // test_pose6.orientation.w = 1.0;
  // test_pose6.position.x = 0.15;
  // test_pose6.position.y = 0.0;
  // test_pose6.position.z = 0.45;
  //
  // geometry_msgs::Pose test_pose7;
  // test_pose6.orientation.w = 1.0;
  // test_pose6.position.x = 0.15;
  // test_pose6.position.y = -0.12;
  // test_pose6.position.z = 0.45;
  //
  // geometry_msgs::Pose test_pose8;
  // test_pose6.orientation.w = 1.0;
  // test_pose6.position.x = 0.15;
  // test_pose6.position.y = -0.12;
  // test_pose6.position.z = 0.35;
  //
  // waypoints2.push_back(test_pose6);
  // waypoints2.push_back(test_pose7);
  // waypoints2.push_back(test_pose8);
  //
  // moveit::planning_interface::MoveGroupInterface::Plan my_plan_2;
  //
  // move_group_2.setMaxVelocityScalingFactor(0.1);
  // moveit_msgs::RobotTrajectory trajectory2;
  // double fraction2 = move_group_2.computeCartesianPath(waypoints2, eef_step, jump_threshold, trajectory2);
  // ROS_INFO("Visualizing plan 5 (Cartesian path) (%.2f%% acheived)", fraction2 * 100.0);
  //
  // my_plan.trajectory_ = trajectory;
  // move_group.execute(my_plan);
  //
  // my_plan_2.trajectory_ = trajectory2;
  // move_group_2.execute(my_plan_2);


  /************************** Wait till shutdown ******************************/
  while(ros::ok()){
    ;
  }

  return 0;
}
