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

// joint_state 1
sensor_msgs::JointState waypoint_3;

void joint_state_cb(const sensor_msgs::JointState &msg)
{
  waypoint_3 = msg;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "waypoint_3");
  ros::NodeHandle n;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Subscribe/Publish
  ros::Subscriber joint_states = n.subscribe("/testing_environment/joint_states", 100, joint_state_cb);
  ros::Publisher pub_waypoint_3 = n.advertise<sensor_msgs::JointState>("/testing_environment/waypoint_3", 100);

  ros::Rate loop_rate(50);

  // Wait for the arm node to finish initializing
  while ((waypoint_3.position.size() < 1) && ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  ros::spinOnce(); // get first waypoint
  ROS_WARN("Publishing third waypoint!");
  while (ros::ok())
  {
    pub_waypoint_3.publish(waypoint_3);
    loop_rate.sleep();
  }
  return 0;
}
