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
#include <testing_environment_moveit/CustomPosition.h>

class SimpleServer{
  public:
    SimpleServer(std::string planning_group):move_group(planning_group){ // initialize move_group with planning_group name
      joint_model_group = move_group.getCurrentState()->getJointModelGroup(planning_group);
      PLANNING_GROUP = planning_group; // set our private planning_group variable for future use with this object
      current_state = move_group.getCurrentState();
      init_services();
      init_params();
    };

  private:
    ros::NodeHandle nh;
    std::string PLANNING_GROUP;
    moveit::planning_interface::MoveGroupInterface move_group;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const robot_state::JointModelGroup* joint_model_group;
    moveit::core::RobotStatePtr current_state;

    // Arm B's base position and orientation relative to arm A's base
    float relative_x;
    float relative_y;
    float relative_z;
    float relative_roll;
    float relative_pitch;
    float relative_yaw;

    // Perch x,y position and height relative to arm A's base
    // orientation will always be facing the arms
    // assume 0.3cm x 0.3cm x 0.3cm operating volume
    float perch_x;
    float perch_y;
    float perch_z;

    // initialize all services (to be called with the python client)
    void init_services();

    // initialize all parameters (set via testing_environment.launch args)
    void init_params();

    // send_to_home service
    ros::ServiceServer srv_send_to_home;
    bool send_to_home(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    void move_arms_to_home();

    // send_to_custom_angles service
    ros::ServiceServer srv_send_to_custom_angles;
    bool send_to_custom_angles(testing_environment_moveit::CustomAngle::Request &req, testing_environment_moveit::CustomAngle::Response &res);
    void move_arms_to_custom_angles(double arm_a_angles[], double arm_b_angles[]);

    // send_to_custom_positions service
    ros::ServiceServer srv_send_to_custom_positions;
    bool send_to_custom_positions(testing_environment_moveit::CustomPosition::Request &req, testing_environment_moveit::CustomPosition::Response &res);
    void move_arms_to_custom_positions(double arm_a_x_pos, double arm_a_y_pos, double arm_a_z_pos, double arm_b_x_pos, double arm_b_y_pos, double arm_b_z_pos);
};

void SimpleServer::init_services(){
  ROS_WARN("INITIALIZING SERVICES!");
  srv_send_to_home = nh.advertiseService("send_to_home", &SimpleServer::send_to_home, this);
  srv_send_to_custom_angles = nh.advertiseService("send_to_custom_angles", &SimpleServer::send_to_custom_angles, this);
  srv_send_to_custom_positions = nh.advertiseService("send_to_custom_positions", &SimpleServer::send_to_custom_positions, this);
}

void SimpleServer::init_params(){
  nh.param<float>("/testing_environment/simple_server/relative_x", relative_x, 0.0);
  nh.param<float>("/testing_environment/simple_server/relative_y", relative_y, 0.22);
  nh.param<float>("/testing_environment/simple_server/relative_z", relative_z, 0.0);
  nh.param<float>("/testing_environment/simple_server/relative_roll", relative_roll, 0.0);
  nh.param<float>("/testing_environment/simple_server/relative_pitch", relative_pitch, 0.0);
  nh.param<float>("/testing_environment/simple_server/relative_yaw", relative_yaw, 0.0);

  nh.param<float>("/testing_environment/simple_server/perch_x", perch_x, 0.3);
  nh.param<float>("/testing_environment/simple_server/perch_y", perch_y, 0.11); // halfway between the two arms
  nh.param<float>("/testing_environment/simple_server/perch_z", perch_z, 0.15);
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
  std::vector<double> joint_group_angles;

  // Set all 8 joints on each arm (16 in total) to 0.0 for home position
  joint_group_angles.push_back(0.0);
  joint_group_angles.push_back(0.0);
  joint_group_angles.push_back(0.0);
  joint_group_angles.push_back(0.0);
  joint_group_angles.push_back(0.0);
  joint_group_angles.push_back(0.0);
  joint_group_angles.push_back(0.0);
  joint_group_angles.push_back(0.0);
  joint_group_angles.push_back(0.0);
  joint_group_angles.push_back(0.0);
  joint_group_angles.push_back(0.0);
  joint_group_angles.push_back(0.0);
  joint_group_angles.push_back(0.0);
  joint_group_angles.push_back(0.0);
  joint_group_angles.push_back(0.0);
  joint_group_angles.push_back(0.0);

  move_group.setJointValueTarget(joint_group_angles);
  move_group.move();
}

bool SimpleServer::send_to_custom_angles(testing_environment_moveit::CustomAngle::Request &req, testing_environment_moveit::CustomAngle::Response &res){
  ROS_WARN("REQUEST: SEND ARMS TO CUSTOM ANGLES!");
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
  ROS_WARN("MOVING BOTH ARMS TO CUSTOM ANGLES!");
  std::vector<double> joint_group_angles;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_angles);

  // Set all 5 joints on each arm (10 in total) to the user-requested angles
  joint_group_angles[0] = arm_a_angles[0];
  joint_group_angles[1] = arm_a_angles[1];
  joint_group_angles[2] = arm_a_angles[2];
  joint_group_angles[3] = arm_a_angles[3];
  joint_group_angles[4] = arm_a_angles[4];
  joint_group_angles[5] = arm_b_angles[0];
  joint_group_angles[6] = arm_b_angles[1];
  joint_group_angles[7] = arm_b_angles[2];
  joint_group_angles[8] = arm_b_angles[3];
  joint_group_angles[9] = arm_b_angles[4];

  move_group.setJointValueTarget(joint_group_angles);
  move_group.move();
}

bool SimpleServer::send_to_custom_positions(testing_environment_moveit::CustomPosition::Request &req, testing_environment_moveit::CustomPosition::Response &res){
  ROS_WARN("REQUEST: SEND ARMS TO CUSTOM POSITIONS!");
  double arm_a_x_pos = req.arm_a_x_pos;
  double arm_a_y_pos = req.arm_a_y_pos;
  double arm_a_z_pos = req.arm_a_z_pos;

  double arm_b_x_pos = req.arm_b_x_pos;
  double arm_b_y_pos = req.arm_b_y_pos;
  double arm_b_z_pos = req.arm_b_z_pos;

  move_arms_to_custom_positions(arm_a_x_pos, arm_a_y_pos, arm_a_z_pos, arm_b_x_pos, arm_b_y_pos, arm_b_z_pos);
  return true;

}

void SimpleServer::move_arms_to_custom_positions(double arm_a_x_pos, double arm_a_y_pos, double arm_a_z_pos, double arm_b_x_pos, double arm_b_y_pos, double arm_b_z_pos){
  ROS_WARN("MOVING BOTH ARMS TO CUSTOM POSITIONS!");
  move_group.setPositionTarget(arm_a_x_pos,arm_a_y_pos,arm_a_z_pos,"wx200_arm_A/ee_arm_link");
  move_group.setPositionTarget(arm_b_x_pos,arm_b_y_pos,arm_b_z_pos,"wx200_arm_B/ee_arm_link");

  moveit_msgs::JointConstraint arm_A_wrist_angle_constraint;
  arm_A_wrist_angle_constraint.joint_name = "wx200_arm_A_wrist_angle";
  arm_A_wrist_angle_constraint.position = 0.0;
  arm_A_wrist_angle_constraint.tolerance_above = 0.05;
  arm_A_wrist_angle_constraint.tolerance_below = 0.05;
  arm_A_wrist_angle_constraint.weight = 1.0; // denotes relative importance 0-1

  moveit_msgs::JointConstraint arm_A_wrist_rotate_constraint;
  arm_A_wrist_rotate_constraint.joint_name = "wx200_arm_A_wrist_rotate";
  arm_A_wrist_rotate_constraint.position = 0.0;
  arm_A_wrist_rotate_constraint.tolerance_above = 0.05;
  arm_A_wrist_rotate_constraint.tolerance_below = 0.05;
  arm_A_wrist_rotate_constraint.weight = 0.9; // denotes relative importance 0-1

  moveit_msgs::JointConstraint arm_B_wrist_angle_constraint;
  arm_B_wrist_angle_constraint.joint_name = "wx200_arm_B_wrist_angle";
  arm_B_wrist_angle_constraint.position = 0.0;
  arm_B_wrist_angle_constraint.tolerance_above = 0.05;
  arm_B_wrist_angle_constraint.tolerance_below = 0.05;
  arm_B_wrist_angle_constraint.weight = 1.0; // denotes relative importance 0-1

  moveit_msgs::JointConstraint arm_B_wrist_rotate_constraint;
  arm_B_wrist_rotate_constraint.joint_name = "wx200_arm_B_wrist_rotate";
  arm_B_wrist_rotate_constraint.position = 0.0;
  arm_B_wrist_rotate_constraint.tolerance_above = 0.05;
  arm_B_wrist_rotate_constraint.tolerance_below = 0.05;
  arm_B_wrist_rotate_constraint.weight = 0.9; // denotes relative importance 0-1

  moveit_msgs::Constraints arm_constraints;
  arm_constraints.joint_constraints.push_back(arm_A_wrist_angle_constraint);
  arm_constraints.joint_constraints.push_back(arm_A_wrist_rotate_constraint);
  arm_constraints.joint_constraints.push_back(arm_B_wrist_angle_constraint);
  arm_constraints.joint_constraints.push_back(arm_B_wrist_rotate_constraint);
  move_group.setPathConstraints(arm_constraints);

  move_group.setPlanningTime(10.0);
  move_group.move();

  // clear path constraints
  ROS_WARN("Done!");
  ROS_WARN("Clearing path constraints...");
  move_group.clearPathConstraints();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "simple_server");
  ROS_INFO("Ready to send commands to both arms.");

  ros::AsyncSpinner spinner(3);
  spinner.start();

  SimpleServer SimpleServer1("testing_environment");
  while(ros::ok()){
    ;
  }
  return 0;
}
