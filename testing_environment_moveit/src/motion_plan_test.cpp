#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

// global var: most recent testing_environment joint_state
sensor_msgs::JointState curr_joint_state;
std::vector<std::string> curr_joint_names;
std::vector<double> curr_joint_positions;

// combined joint_state callback
void callback(const sensor_msgs::JointState& js){
  curr_joint_state = js;
  ROS_WARN("curr_joint_state[0]: %f", curr_joint_state.position[0]);

  std::vector<std::string> temp_string(std::begin(js.name), std::end(js.name));
  std::vector<double> temp_positions(std::begin(js.position), std::end(js.position));
  curr_joint_names = temp_string;
  curr_joint_positions = temp_positions;

  ROS_WARN("curr_joint_names[0]: %s", curr_joint_names[0].c_str());
  ROS_WARN("curr_joint_positions[1]: %f", curr_joint_positions[1]);

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "motion_plan_test");
  ros::NodeHandle nh;

  // wait for rviz/arms to Initialize
  ros::Duration(3.0).sleep();

  // subscriber to combined joint_states
  ros::Subscriber curr_joint_sub = nh.subscribe("/testing_environment/joint_states", 1, callback);

  // Look up robot description and construct robot model to use
  robot_model_loader::RobotModelLoader robot_model_loader("/testing_environment/robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

  // construct a robot state that maintains the configuration of the robot
  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));

  // main loop
  ros::Rate loop_rate(50.0);
  while (ros::ok()){
    ros::spinOnce();

    // kinematic_state->setVariablePosition("wx200_arm_B_waist", 10.0);
    kinematic_state->setVariablePositions(curr_joint_names, curr_joint_positions);
    const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("testing_environment");
    const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

    // retrieve the current set of joint values tored in the state for the testing_environment
    std::vector<double> joint_values;
    kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
    for (std::size_t i = 0; i < joint_names.size(); ++i)
    {
      ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    }

    loop_rate.sleep();
  }

  // ros::spin();
  // ros::shutdown();
  return 0;
}
