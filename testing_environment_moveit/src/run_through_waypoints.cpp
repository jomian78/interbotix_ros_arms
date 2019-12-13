// ROS imports
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <std_srvs/Empty.h>

// MoveIt imports
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

// joint_states for each waypoints
sensor_msgs::JointState waypoint_1;
sensor_msgs::JointState waypoint_2;
sensor_msgs::JointState waypoint_3;

std::vector<double> joint_positions_1;
std::vector<double> joint_positions_2;
std::vector<double> joint_positions_3;

void first_joint_state_cb(const sensor_msgs::JointState &msg)
{
  waypoint_1 = msg;
  joint_positions_1 = waypoint_1.position;
}
void second_joint_state_cb(const sensor_msgs::JointState &msg)
{
  waypoint_2 = msg;
  joint_positions_2 = waypoint_2.position;
}
void third_joint_state_cb(const sensor_msgs::JointState &msg)
{
  waypoint_3 = msg;
  joint_positions_3 = waypoint_3.position;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "run_through_waypoints");
  ros::NodeHandle n;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ROS_WARN("line 51");

  // retrieve relative distances between bases
  float relative_x;
  float relative_y;
  float relative_z;
  float relative_roll;
  float relative_pitch;
  float relative_yaw;
  n.param<float>("/testing_environment/pseudo_random_arm_poses/relative_x", relative_x, 0.0);
  n.param<float>("/testing_environment/pseudo_random_arm_poses/relative_y", relative_y, 0.50);
  n.param<float>("/testing_environment/pseudo_random_arm_poses/relative_z", relative_z, 0.0);
  n.param<float>("/testing_environment/pseudo_random_arm_poses/relative_roll", relative_roll, 0.0);
  n.param<float>("/testing_environment/pseudo_random_arm_poses/relative_pitch", relative_pitch, 0.0);
  n.param<float>("/testing_environment/pseudo_random_arm_poses/relative_yaw", relative_yaw, 0.0);
  ROS_WARN("relative_y value: %f", relative_y);


  // Subscribe to joint states
  ros::Subscriber sub_waypoint_1 = n.subscribe("/testing_environment/waypoint_1", 50, first_joint_state_cb);
  ros::Subscriber sub_waypoint_2 = n.subscribe("/testing_environment/waypoint_2", 50, second_joint_state_cb);
  ros::Subscriber sub_waypoint_3 = n.subscribe("/testing_environment/waypoint_3", 50, third_joint_state_cb);

  ros::Rate loop_rate(50);

  // Wait for the arm node to finish initializing
  while ((waypoint_3.position.size() < 1) && ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  ROS_WARN("giving user time to prepare for motion sequence");
  ros::Duration(15.0).sleep();

  static const std::string PLANNING_GROUP = "testing_environment";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  const robot_state::JointModelGroup* joint_model_group =
    move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);


  ros::spinOnce(); // grab the waypoints
  ros::spinOnce(); // grab the waypoints
  ros::spinOnce(); // grab the waypoints

  // move to reset position
  move_group.setJointValueTarget("wx200_arm_A_waist", 0.0);
  move_group.setJointValueTarget("wx200_arm_A_shoulder", -1.88);
  move_group.setJointValueTarget("wx200_arm_A_elbow", -1.55);
  move_group.setJointValueTarget("wx200_arm_A_wrist_angle", -0.8);
  move_group.setJointValueTarget("wx200_arm_A_wrist_rotate", 0.0);
  move_group.setJointValueTarget("wx200_arm_A_gripper", 0.0);
  move_group.setJointValueTarget("wx200_arm_A_left_finger", 0.0);
  move_group.setJointValueTarget("wx200_arm_A_right_finger", 0.0);

  move_group.setJointValueTarget("wx200_arm_B_waist", 0.0);
  move_group.setJointValueTarget("wx200_arm_B_shoulder", -1.88);
  move_group.setJointValueTarget("wx200_arm_B_elbow", -1.55);
  move_group.setJointValueTarget("wx200_arm_B_wrist_angle", 0.0);
  move_group.setJointValueTarget("wx200_arm_B_wrist_rotate", 0.0);
  move_group.setJointValueTarget("wx200_arm_B_gripper", 0.0);
  move_group.setJointValueTarget("wx200_arm_B_left_finger", 0.0);
  move_group.setJointValueTarget("wx200_arm_B_right_finger", 0.0);

  ROS_WARN("Moving to reset position!");
  move_group.move();
  ros::Duration(3.0).sleep();

  // waypoint 1
  move_group.setJointValueTarget("wx200_arm_A_waist", joint_positions_1[0]);
  move_group.setJointValueTarget("wx200_arm_A_shoulder", joint_positions_1[1]);
  move_group.setJointValueTarget("wx200_arm_A_elbow", joint_positions_1[2]);
  move_group.setJointValueTarget("wx200_arm_A_wrist_angle", joint_positions_1[3]);
  move_group.setJointValueTarget("wx200_arm_A_wrist_rotate", joint_positions_1[4]);
  move_group.setJointValueTarget("wx200_arm_A_gripper", joint_positions_1[5]);
  move_group.setJointValueTarget("wx200_arm_A_left_finger", joint_positions_1[6]);
  move_group.setJointValueTarget("wx200_arm_A_right_finger", joint_positions_1[7]);

  move_group.setJointValueTarget("wx200_arm_B_waist", joint_positions_1[8]);
  move_group.setJointValueTarget("wx200_arm_B_shoulder", joint_positions_1[9]);
  move_group.setJointValueTarget("wx200_arm_B_elbow", joint_positions_1[10]);
  move_group.setJointValueTarget("wx200_arm_B_wrist_angle", joint_positions_1[11]);
  move_group.setJointValueTarget("wx200_arm_B_wrist_rotate", joint_positions_1[12]);
  move_group.setJointValueTarget("wx200_arm_B_gripper", joint_positions_1[13]);
  move_group.setJointValueTarget("wx200_arm_B_left_finger", joint_positions_1[14]);
  move_group.setJointValueTarget("wx200_arm_B_right_finger", joint_positions_1[15]);

  ROS_WARN("Moving to waypoint 1!");
  move_group.move();
  ros::Duration(3.0).sleep();

  // waypoint 2
  move_group.setJointValueTarget("wx200_arm_A_waist", joint_positions_2[0]);
  move_group.setJointValueTarget("wx200_arm_A_shoulder", joint_positions_2[1]);
  move_group.setJointValueTarget("wx200_arm_A_elbow", joint_positions_2[2]);
  move_group.setJointValueTarget("wx200_arm_A_wrist_angle", joint_positions_2[3]);
  move_group.setJointValueTarget("wx200_arm_A_wrist_rotate", joint_positions_2[4]);
  move_group.setJointValueTarget("wx200_arm_A_gripper", joint_positions_2[5]);
  move_group.setJointValueTarget("wx200_arm_A_left_finger", joint_positions_2[6]);
  move_group.setJointValueTarget("wx200_arm_A_right_finger", joint_positions_2[7]);

  move_group.setJointValueTarget("wx200_arm_B_waist", joint_positions_2[8]);
  move_group.setJointValueTarget("wx200_arm_B_shoulder", joint_positions_2[9]);
  move_group.setJointValueTarget("wx200_arm_B_elbow", joint_positions_2[10]);
  move_group.setJointValueTarget("wx200_arm_B_wrist_angle", joint_positions_2[11]);
  move_group.setJointValueTarget("wx200_arm_B_wrist_rotate", joint_positions_2[12]);
  move_group.setJointValueTarget("wx200_arm_B_gripper", joint_positions_2[13]);
  move_group.setJointValueTarget("wx200_arm_B_left_finger", joint_positions_2[14]);
  move_group.setJointValueTarget("wx200_arm_B_right_finger", joint_positions_2[15]);
  ROS_WARN("Moving to waypoint 2!");
  move_group.move();
  ros::Duration(3.0).sleep();

  // waypoint 3
  move_group.setJointValueTarget("wx200_arm_A_waist", joint_positions_3[0]);
  move_group.setJointValueTarget("wx200_arm_A_shoulder", joint_positions_3[1]);
  move_group.setJointValueTarget("wx200_arm_A_elbow", joint_positions_3[2]);
  move_group.setJointValueTarget("wx200_arm_A_wrist_angle", joint_positions_3[3]);
  move_group.setJointValueTarget("wx200_arm_A_wrist_rotate", joint_positions_3[4]);
  move_group.setJointValueTarget("wx200_arm_A_gripper", joint_positions_3[5]);
  move_group.setJointValueTarget("wx200_arm_A_left_finger", joint_positions_3[6]);
  move_group.setJointValueTarget("wx200_arm_A_right_finger", joint_positions_3[7]);

  move_group.setJointValueTarget("wx200_arm_B_waist", joint_positions_3[8]);
  move_group.setJointValueTarget("wx200_arm_B_shoulder", joint_positions_3[9]);
  move_group.setJointValueTarget("wx200_arm_B_elbow", joint_positions_3[10]);
  move_group.setJointValueTarget("wx200_arm_B_wrist_angle", joint_positions_3[11]);
  move_group.setJointValueTarget("wx200_arm_B_wrist_rotate", joint_positions_3[12]);
  move_group.setJointValueTarget("wx200_arm_B_gripper", joint_positions_3[13]);
  move_group.setJointValueTarget("wx200_arm_B_left_finger", joint_positions_3[14]);
  move_group.setJointValueTarget("wx200_arm_B_right_finger", joint_positions_3[15]);
  ROS_WARN("Moving to waypoint 3!");
  move_group.move();
  ros::Duration(3.0).sleep();

  ROS_WARN("Done with waypoint sequence!");
  while (ros::ok())
  {
    ;
  }
  return 0;
}
