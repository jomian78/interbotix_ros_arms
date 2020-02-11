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

// tf2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class SimpleServer{
  public:
    SimpleServer(std::string planning_group):move_group(planning_group){ // initialize move_group with planning_group name
      joint_model_group = move_group.getCurrentState()->getJointModelGroup(planning_group);
      PLANNING_GROUP = planning_group; // set our private planning_group variable for future use with this object
      current_state = move_group.getCurrentState();
      init_services();
      init_params();
      init_collision_objects();
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

    // initialize perch and collision objects
    void init_collision_objects();

    // takes in Tperch_ee; returns Tglobal_ee
    tf2::Transform global_to_ee(tf2::Transform perch_to_ee);

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

  nh.param<float>("/testing_environment/simple_server/perch_x", perch_x, 0.5);
  nh.param<float>("/testing_environment/simple_server/perch_y", perch_y, 0.11); // halfway between the two arms
  nh.param<float>("/testing_environment/simple_server/perch_z", perch_z, 0.3);
}

void SimpleServer::init_collision_objects(){
  // arm A ground plane
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

  // arm B ground plane
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

  // perch object
  moveit_msgs::CollisionObject perch;
  perch.header.frame_id = move_group.getPlanningFrame();
  perch.id = "perch";

  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.05;
  primitive.dimensions[1] = 0.05;
  primitive.dimensions[2] = perch_z;

  geometry_msgs::Pose box_pose;
  box_pose.orientation.z = -1.0;
  box_pose.position.x = perch_x;
  box_pose.position.y = perch_y;
  box_pose.position.z = perch_z/2.0;

  perch.primitives.push_back(primitive);
  perch.primitive_poses.push_back(box_pose);
  perch.operation = perch.ADD;

  // Add objects to planning_scene_interface
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(perch);
  collision_objects.push_back(wx200_arm_A_table);
  collision_objects.push_back(wx200_arm_B_table);
  planning_scene_interface.addCollisionObjects(collision_objects);
}

tf2::Transform SimpleServer::global_to_ee(tf2::Transform perch_to_ee){
  // global to perch
  tf2::Vector3 perch_position(perch_x,perch_y,perch_z);
  tf2::Quaternion perch_orientation(tf2::Vector3(0,0,1), 3.1415926);
  tf2::Transform global_to_perch(perch_orientation, perch_position);

  tf2::Transform global_to_ee = global_to_perch*perch_to_ee;
  return global_to_ee;
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

  tf2::Vector3 perch_to_a_pos(arm_a_x_pos, arm_a_y_pos, arm_a_z_pos);
  tf2::Quaternion perch_to_a_orientation(tf2::Vector3(0,0,1), 0.0);
  tf2::Transform perch_to_a_transform(perch_to_a_orientation, perch_to_a_pos);
  tf2::Transform global_to_a_transform = global_to_ee(perch_to_a_transform);

  tf2::Vector3 perch_to_b_pos(arm_b_x_pos, arm_b_y_pos, arm_b_z_pos);
  tf2::Quaternion perch_to_b_orientation(tf2::Vector3(0,0,1), 0.0);
  tf2::Transform perch_to_b_transform(perch_to_b_orientation, perch_to_b_pos);
  tf2::Transform global_to_b_transform = global_to_ee(perch_to_b_transform);

  double new_a_x = global_to_a_transform.getOrigin().x();
  double new_a_y = global_to_a_transform.getOrigin().y();
  double new_a_z = global_to_a_transform.getOrigin().z();

  double new_b_x = global_to_b_transform.getOrigin().x();
  double new_b_y = global_to_b_transform.getOrigin().y();
  double new_b_z = global_to_b_transform.getOrigin().z();

  move_arms_to_custom_positions(new_a_x, new_a_y, new_a_z, new_b_x, new_b_y, new_b_z);
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
