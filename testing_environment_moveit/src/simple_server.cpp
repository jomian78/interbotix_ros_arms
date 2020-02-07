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

// custom srvs
#include <testing_environment_moveit/CustomAngle.h>

class SimpleServer{
  public:
    SimpleServer(std::string planning_group):move_group(planning_group){ // initialize move_group with planning_group name
      joint_model_group = move_group.getCurrentState()->getJointModelGroup(planning_group);
      PLANNING_GROUP = planning_group; // set our private planning_group variable for future use with this object
      init_services();
    };

  private:
    ros::NodeHandle nh;
    std::string PLANNING_GROUP;
    moveit::planning_interface::MoveGroupInterface move_group;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const robot_state::JointModelGroup* joint_model_group;

    // initialize all services (to be called with the python client)
    void init_services();

    // send_to_home service
    ros::ServiceServer srv_send_to_home;
    bool send_to_home(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    void move_arms_to_home();

    // send_to_custom_angles service
    ros::ServiceServer srv_send_to_custom_angles;
    bool send_to_custom_angles(testing_environment_moveit::CustomAngle::Request &req, testing_environment_moveit::CustomAngle::Response &res);
    void move_arms_to_custom_angles(double arm_a_angles[], double arm_b_angles[]);
};

void SimpleServer::init_services(){
  ROS_WARN("INITIALIZING SERVICES!");
  srv_send_to_home = nh.advertiseService("send_to_home", &SimpleServer::send_to_home, this);
  srv_send_to_custom_angles = nh.advertiseService("send_to_custom_angles", &SimpleServer::send_to_custom_angles, this);
}

/**
* Send_to_home function
*/
bool SimpleServer::send_to_home(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
  ROS_WARN("REQUEST: SEND ARMS TO HOME POSITION");
  move_arms_to_home();
  return true;
}

/**
* Home position helper
*/
void SimpleServer::move_arms_to_home(){
  ROS_WARN("MOVING BOTH ARMS!");
  std::vector<double> joint_group_positions;

  // Set all 8 joints on each arm (16 in total) to 0.0 for home position
  joint_group_positions.push_back(0.0);
  joint_group_positions.push_back(0.0);
  joint_group_positions.push_back(0.0);
  joint_group_positions.push_back(0.0);
  joint_group_positions.push_back(0.0);
  joint_group_positions.push_back(0.0);
  joint_group_positions.push_back(0.0);
  joint_group_positions.push_back(0.0);
  joint_group_positions.push_back(0.0);
  joint_group_positions.push_back(0.0);
  joint_group_positions.push_back(0.0);
  joint_group_positions.push_back(0.0);
  joint_group_positions.push_back(0.0);
  joint_group_positions.push_back(0.0);
  joint_group_positions.push_back(0.0);
  joint_group_positions.push_back(0.0);

  move_group.setJointValueTarget(joint_group_positions);
  move_group.move();
  ROS_WARN("LINE 89");
}

bool SimpleServer::send_to_custom_angles(testing_environment_moveit::CustomAngle::Request &req, testing_environment_moveit::CustomAngle::Response &res){
  ROS_WARN("REQUEST: SEND ARMS TO CUSTOM ANGLES!");
  // ROS_WARN("first value for arm_a in custom request: %f", req.arm_a_angles[0]);
  // move_arms_to_custom_angles(req.arm_a_angles, req.arm_b_angles);
  double a[8];
  double b[8];
  for (int i = 0; i < 8; i++){
    a[i] = req.arm_a_angles[i];
  }
  for (int i = 0; i < 8; i++){
    b[i] = req.arm_b_angles[i];
  }
  move_arms_to_custom_angles(a, b);
  return true;
}

void SimpleServer::move_arms_to_custom_angles(double arm_a_angles[], double arm_b_angles[]){
  ROS_WARN("MOVING BOTH ARMS!");
  std::vector<double> joint_group_positions;

  // Set all 8 joints on each arm (16 in total) to the user-requested angles
  joint_group_positions.push_back(arm_a_angles[0]);
  joint_group_positions.push_back(arm_a_angles[1]);
  joint_group_positions.push_back(arm_a_angles[2]);
  joint_group_positions.push_back(arm_a_angles[3]);
  joint_group_positions.push_back(arm_a_angles[4]);
  joint_group_positions.push_back(arm_a_angles[5]);
  joint_group_positions.push_back(arm_a_angles[6]);
  joint_group_positions.push_back(arm_a_angles[7]);

  joint_group_positions.push_back(arm_b_angles[0]);
  joint_group_positions.push_back(arm_b_angles[1]);
  joint_group_positions.push_back(arm_b_angles[2]);
  joint_group_positions.push_back(arm_b_angles[3]);
  joint_group_positions.push_back(arm_b_angles[4]);
  joint_group_positions.push_back(arm_b_angles[5]);
  joint_group_positions.push_back(arm_b_angles[6]);
  joint_group_positions.push_back(arm_b_angles[7]);

  move_group.setJointValueTarget(joint_group_positions);
  move_group.move();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "simple_server");
  ROS_INFO("Ready to send commands to both arms.");

  ros::AsyncSpinner spinner(1);
  spinner.start();

  SimpleServer SimpleServer1("testing_environment");
  while(ros::ok()){
    ;
  }
  return 0;
}
