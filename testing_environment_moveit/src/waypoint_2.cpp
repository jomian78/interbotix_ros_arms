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
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// joint_state 2
sensor_msgs::JointState curr_joint_state;
sensor_msgs::JointState waypoint_2;

void joint_state_cb(const sensor_msgs::JointState &msg)
{
  curr_joint_state = msg;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "waypoint_2");
  ros::NodeHandle n;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Subscribe/Publish
  ros::Subscriber joint_states = n.subscribe("/testing_environment/joint_states", 50, joint_state_cb);
  ros::Publisher pub_waypoint_2 = n.advertise<sensor_msgs::JointState>("/testing_environment/waypoint_2", 50);

  ros::Rate loop_rate(50);

  // Wait for the arm node to finish initializing
  while ((curr_joint_state.position.size() < 1) && ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  ros::spinOnce(); // get second waypoint
  waypoint_2 = curr_joint_state;
  ROS_WARN("Publishing second waypoint!");
  while (ros::ok())
  {
    pub_waypoint_2.publish(waypoint_2);
    loop_rate.sleep();
  }
  return 0;
}
