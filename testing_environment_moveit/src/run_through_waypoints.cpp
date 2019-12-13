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


  // Subscribe to joint states
  ros::Subscriber sub_waypoint_1 = n.subscribe("/testing_environment/waypoint_1", 100, first_joint_state_cb);
  ros::Subscriber sub_waypoint_2 = n.subscribe("/testing_environment/waypoint_2", 100, second_joint_state_cb);
  ros::Subscriber sub_waypoint_3 = n.subscribe("/testing_environment/waypoint_3", 100, third_joint_state_cb);

  ros::Rate loop_rate(50);

  static const std::string PLANNING_GROUP = "testing_environment";
  ROS_WARN("line 62");
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  const robot_state::JointModelGroup* joint_model_group =
    move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  ROS_WARN("line 67");

  ROS_WARN("wait for model info to load");
  ros::Duration(15.0).sleep();

  // Wait for the arm node to finish initializing
  while ((waypoint_1.position.size() < 1) && ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  ros::spinOnce(); // grab the waypoints

  std::vector<double> move_1;
  std::vector<double> move_2;
  std::vector<double> move_3;

  move_1 = joint_positions_1;
  move_2 = joint_positions_2;
  move_3 = joint_positions_3;

  // waypoint 1
  move_group.setJointValueTarget(move_1);
  move_group.move();

  // waypoint 2
  move_group.setJointValueTarget(move_2);
  move_group.move();

  // waypoint 3
  move_group.setJointValueTarget(move_3);
  move_group.move();

  ROS_WARN("Done with waypoint sequence!");
  while (ros::ok())
  {
    ;
  }
  return 0;
}
