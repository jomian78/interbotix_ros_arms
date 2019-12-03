#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

// most recent testing_environment joint_state
sensor_msgs::JointState curr_joint_state;

void callback(const sensor_msgs::JointState& js){
  curr_joint_state = js;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "motion_plan_test");
  ros::NodeHandle nh;

  // wait for rviz to Initialize
  ros::Duration(3.0).sleep();

  // testing arm_A planning group
  // static const std::string PLANNING_GROUP = "wx200_arm_A/arm_controller";

  // setup for controlling and planning for arm A
  // moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  // setup for adding and removing collision objects in the world scene
  // moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // using pointers for improved performance
  // const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  // test pose
  // geometry_msgs::Pose target_pose1;
  // target_pose1.orientation.w = 1.0;
  // target_pose1.position.x = 0.1;
  // target_pose1.position.y = 0.0;
  // target_pose1.position.z = 0.1;
  // move_group.setPoseTarget(target_pose1);

  // compute and visualize plan
  // moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  // execute
  // move_group.move();

  ros::Subscriber curr_joint_sub = nh.subscribe("/testing_environment/joint_states", 1, callback);

  robot_model_loader::RobotModelLoader robot_model_loader("/testing_environment/robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
  // kinematic_state->setToDefaultValues();
  // kinematic_state->setToRandomPositions();
  ros::Duration(3.0).sleep();
  kinematic_state->setVariableValues(curr_joint_state);


  const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("wx200_arm_A/arm_controller");
  // const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("testing_environment");


  const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

  std::vector<double> joint_values;
  kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
  for (std::size_t i = 0; i < joint_names.size(); ++i)
  {
    ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
  }



  ros::shutdown();
  return 0;
}
