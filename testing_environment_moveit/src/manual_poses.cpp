// ROS imports
#include <ros/ros.h>
#include "interbotix_sdk/RobotInfo.h"
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
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// testing_environment joints
sensor_msgs::JointState joint_states;

/// @brief Joint state callback function
/// @param msg - updated joint states
void joint_state_cb(const sensor_msgs::JointState &msg)
{
  joint_states = msg;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "manual_poses");
  ros::NodeHandle n;

  // Subscribe to joint states
  ros::Subscriber sub_positions = n.subscribe("/testing_environment/joint_states", 100, joint_state_cb);

  ros::ServiceClient wx200_arm_A_srv_torque_on = n.serviceClient<std_srvs::Empty>("/testing_environment/wx200_arm_A/torque_arm_on");
  ros::ServiceClient wx200_arm_B_srv_torque_on = n.serviceClient<std_srvs::Empty>("/testing_environment/wx200_arm_B/torque_arm_on");

  ros::ServiceClient wx200_arm_A_srv_torque_off = n.serviceClient<std_srvs::Empty>("/testing_environment/wx200_arm_A/torque_arm_off");
  ros::ServiceClient wx200_arm_B_srv_torque_off = n.serviceClient<std_srvs::Empty>("/testing_environment/wx200_arm_B/torque_arm_off");

  bool wx200_arm_A_success;
  bool wx200_arm_B_success;

  // Torque off the first robot arm so that the user can easily manipulate it
  std_srvs::Empty wx200_arm_A_e_srv;
  std_srvs::Empty wx200_arm_B_e_srv;

  wx200_arm_A_success = wx200_arm_A_srv_torque_off.call(wx200_arm_A_e_srv);
  if (!wx200_arm_A_success)
  {
    ROS_ERROR("Could not torque off wx200_arm_A arm.");
    return 1;
  }
  else{
    ROS_WARN("wx200_arm_A torqued off!")
  }

  wx200_arm_B_success = wx200_arm_B_srv_torque_off.call(wx200_arm_B_e_srv);
  if (!wx200_arm_B_success)
  {
    ROS_ERROR("Could not torque off wx200_arm_B arm.");
    return 1;
  }
  else{
    ROS_WARN("wx200_arm_B torqued off!")
  }

  size_t pose_cntr = 0;
  ros::Rate loop_rate(50);
  while (ros::ok())
  {
    if (pose_cntr == 3){
      ROS_WARN("Executing pose...stand clear!");
      ros::Duration.sleep(10.0); // waiting for user to stand clear
      // execute the sequence of waypoints
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
