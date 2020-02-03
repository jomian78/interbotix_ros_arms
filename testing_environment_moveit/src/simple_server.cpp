#include "ros/ros.h"

// import statements from move_group_test.cpp
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

// std_srvs
#include <std_srvs/Empty.h>

class SimpleServer{
  public:
    SimpleServer(std::string planning_group):move_group(planning_group){
      joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
      init_services();
    };
    void init_services();
    void send_to_home();
    void move_arms_to_home();

  private:
    ros::NodeHandle nh;
    static const std::string PLANNING_GROUP;
    moveit::planning_interface::MoveGroupInterface move_group;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const robot_state::JointModelGroup* joint_model_group;

};

void SimpleServer::init_services(){
  srv_send_to_home = nh.advertiseService("send_to_home", &SimpleServer::send_to_home, this);
}

/**
* Send_to_home function
*/
bool SimpleServer::send_to_home(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
  move_arms_to_home();
  ROS_INFO("request: send arms to home position");
  return true;
}

/**
* Home position helper
*/
void SimpleServer::move_arms_to_home(){
  std::vector<double> joint_group_positions;

  joint_group_positions[0] = 0.0;
  joint_group_positions[1] = 0.0;
  joint_group_positions[2] = 0.0;
  joint_group_positions[3] = 0.0;
  joint_group_positions[4] = 0.0;
  joint_group_positions[5] = 0.0;
  joint_group_positions[6] = 0.0;
  joint_group_positions[7] = 0.0;
  joint_group_positions[8] = 0.0;
  joint_group_positions[9] = 0.0;
  joint_group_positions[10] = 0.0;
  joint_group_positions[11] = 0.0;
  joint_group_positions[12] = 0.0;
  joint_group_positions[13] = 0.0;
  joint_group_positions[14] = 0.0;
  joint_group_positions[15] = 0.0;

  move_group.setJointValueTarget(joint_group_positions);
  ROS_WARN("Moving both arms!");
  move_group.move();
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "simple_server");
  ROS_INFO("Ready to send commands to both arms.");

  SimpleServer SimpleServer1("testing_environment");
  ros::Rate loop_rate(50);
  while(ros::ok()){
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
