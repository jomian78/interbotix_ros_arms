
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_test");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  /********************* MoveGroupInterface test *****************************/

  static const std::string PLANNING_GROUP = "testing_environment";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  // for adding and removing collision objects
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  const robot_state::JointModelGroup* joint_model_group =
    move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  // geometry_msgs::Pose test_pose1;
  // test_pose1.orientation.w = 1.0;
  // test_pose1.position.x = 0.5;
  // test_pose1.position.y = 0.0;
  // test_pose1.position.z = 0.5;
  // move_group.setPoseTarget(test_pose1);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  // bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  //
  // ROS_INFO("Visualizing test_pose 1 (pose goal) %s", success ? "" : "FAILED");
  //
  // move_group.move();
  //
  // ROS_WARN("Waiting after trajectory 1...");
  // ros::Duration(3.0).sleep();
  /****************************************************************************/

  /********************* Second test ******************************************/
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();

  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  ROS_WARN("joint_model_position[0]: %f", joint_group_positions[0]);
  // ROS_WARN("joint_model_position[7]: %f", joint_group_positions[7]);

  joint_group_positions[0] = -1.0;
  // joint_group_positions[7] = -1.0;

  move_group.setJointValueTarget(joint_group_positions);

  // success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  // ROS_INFO("Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");

  move_group.move();

  /****************************************************************************/
  // int cntr = 0;
  while(ros::ok()){
    // current_state = move_group.getCurrentState();
    // current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    // ROS_WARN("joint_group_positions[0]: %f", joint_group_positions[0]);
    // ROS_WARN("cntr= %d", cntr);
    //
    // if (cntr == 200){
    //   std::vector<double> plan2;
    //   current_state->copyJointGroupPositions(joint_model_group, plan2);
    //   plan2[0] = 0.1;
    //   plan2[7] = -0.1;
    //   move_group.setJointValueTarget(plan2);
    //   ROS_INFO("LINE 85");
    // }
    // cntr++;

    // ROS_WARN("joint_group_positions[0]: %f", joint_group_positions[0]);
    // ROS_WARN("joint_group_positions[1]: %f", joint_group_positions[1]);
    // ROS_WARN("joint_group_positions[2]: %f", joint_group_positions[2]);
    // ROS_WARN("joint_group_positions[3]: %f", joint_group_positions[3]);
    // ROS_WARN("joint_group_positions[4]: %f", joint_group_positions[4]);
    // ROS_WARN("joint_group_positions[5]: %f", joint_group_positions[5]);
    // ROS_WARN("joint_group_positions[6]: %f", joint_group_positions[6]);
    // ROS_WARN("joint_group_positions[7]: %f", joint_group_positions[7]);
    // ;
  }

  return 0;
}
