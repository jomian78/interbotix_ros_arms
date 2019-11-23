#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <message_filters/subscriber.h>
// #include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

ros::Publisher combined_js_pub;

void callback(const sensor_msgs::JointState& js_a, const sensor_msgs::JointState& js_b){
    sensor_msgs::JointState curr_combined_js;

    // Header
    curr_combined_js.header.stamp = ros::Time::now();

    // Add names for Arm A and B joints
    curr_combined_js.name.push_back("wx200_arm_A_waist");
    curr_combined_js.name.push_back("wx200_arm_A_shoulder");
    curr_combined_js.name.push_back("wx200_arm_A_elbow");
    curr_combined_js.name.push_back("wx200_arm_A_wrist_angle");
    curr_combined_js.name.push_back("wx200_arm_A_wrist_rotate");
    curr_combined_js.name.push_back("wx200_arm_A_gripper");
    curr_combined_js.name.push_back("wx200_arm_A_left_finger");
    curr_combined_js.name.push_back("wx200_arm_A_right_finger");

    curr_combined_js.name.push_back("wx200_arm_B_waist");
    curr_combined_js.name.push_back("wx200_arm_B_shoulder");
    curr_combined_js.name.push_back("wx200_arm_B_elbow");
    curr_combined_js.name.push_back("wx200_arm_B_wrist_angle");
    curr_combined_js.name.push_back("wx200_arm_B_wrist_rotate");
    curr_combined_js.name.push_back("wx200_arm_B_gripper");
    curr_combined_js.name.push_back("wx200_arm_B_left_finger");
    curr_combined_js.name.push_back("wx200_arm_B_right_finger");

    // copying position values
    for (int i = 0; i < js_a.position.size(); i++){
      curr_combined_js.position.push_back(js_a.position[i]);
    }
    for (int i = 0; i < js_b.position.size(); i++){
      curr_combined_js.position.push_back(js_b.position[i]);
    }

    // copying velocity values
    for (int i = 0; i < js_a.velocity.size(); i++){
      curr_combined_js.velocity.push_back(js_a.velocity[i]);
    }
    for (int i = 0; i < js_b.velocity.size(); i++){
      curr_combined_js.velocity.push_back(js_b.velocity[i]);
    }

    // copying effort values
    for (int i = 0; i < js_a.effort.size(); i++){
      curr_combined_js.effort.push_back(js_a.effort[i]);
    }
    for (int i = 0; i < js_b.effort.size(); i++){
      curr_combined_js.effort.push_back(js_b.effort[i]);
    }

    // publish the combined joint_state
    combined_js_pub.publish(curr_combined_js);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "combine_arm_joints");

  ros::NodeHandle nh;

  combined_js_pub = nh.advertise<sensor_msgs::JointState>("/testing_environment/joint_states", 100);
  message_filters::Subscriber<sensor_msgs::JointState> js_a_sub(nh, "/testing_environment/wx200_arm_A/joint_states", 1);
  message_filters::Subscriber<sensor_msgs::JointState> js_b_sub(nh, "/testing_environment/wx200_arm_B/joint_states", 1);
  // message_filters::TimeSynchronizer<sensor_msgs::JointState, sensor_msgs::JointState> sync(js_a_sub, js_b_sub, 10);

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::JointState, sensor_msgs::JointState> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), js_a_sub, js_b_sub);
  sync.registerCallback(callback);

  ros::spin();

  return 0;
}


// class JointCombiner{
//   public:
//     JointCombiner():sync(wx200_arm_A_sub, wx200_arm_B_sub, 10){
//       // combined joint_state publisher
//       pub_curr_combined_js = nh.advertise<sensor_msgs::JointState>("joint_states", 100);
//
//       // set message_filters
//       wx200_arm_A_sub.subscribe(nh, "/testing_environment/wx200_arm_A/joint_states", 1);
//       wx200_arm_B_sub.subscribe(nh, "/testing_environment/wx200_arm_B/joint_states", 1);
//
//       sync.registerCallback(&JointCombiner::callback, this);
//     };
//
//     void combine_arm_joints();
//
//   private:
//     ros::NodeHandle nh;
//
//     // most recent joint_states variables from each arm
//     sensor_msgs::JointState curr_js_a;
//     sensor_msgs::JointState curr_js_b;
//
//     // message_filters
//     message_filters::Subscriber<sensor_msgs::JointState> wx200_arm_A_sub;
//     message_filters::Subscriber<sensor_msgs::JointState> wx200_arm_B_sub;
//
//     // TimeSynchronizer
//     message_filters::TimeSynchronizer<sensor_msgs::JointState, sensor_msgs::JointState> sync;
//
//     // Publisher for combined joint_states
//     ros::Publisher pub_curr_combined_js;
//
//     // TimeSynchronizer callback
//     void callback(const sensor_msgs::JointState& js_a, const sensor_msgs::JointState& js_b);
//
// };
//
// // TimeSynchronizer callback
// void JointCombiner::callback(const sensor_msgs::JointState& js_a, const sensor_msgs::JointState& js_b){
//   curr_js_a = js_a;
//   curr_js_b = js_b;
// }
//
// // class function for combining the joints from different arms
// void JointCombiner::combine_arm_joints(){
//
//   sensor_msgs::JointState curr_combined_js;
//
//   // Header
//   curr_combined_js.header.stamp = ros::Time::now();
//
//   // Add names for Arm A and B joints
//   curr_combined_js.name.push_back("wx200_arm_A_waist");
//   curr_combined_js.name.push_back("wx200_arm_A_shoulder");
//   curr_combined_js.name.push_back("wx200_arm_A_elbow");
//   curr_combined_js.name.push_back("wx200_arm_A_wrist_angle");
//   curr_combined_js.name.push_back("wx200_arm_A_wrist_rotate");
//   curr_combined_js.name.push_back("wx200_arm_A_gripper");
//   curr_combined_js.name.push_back("wx200_arm_A_left_finger");
//   curr_combined_js.name.push_back("wx200_arm_A_right_finger");
//
//   curr_combined_js.name.push_back("wx200_arm_B_waist");
//   curr_combined_js.name.push_back("wx200_arm_B_shoulder");
//   curr_combined_js.name.push_back("wx200_arm_B_elbow");
//   curr_combined_js.name.push_back("wx200_arm_B_wrist_angle");
//   curr_combined_js.name.push_back("wx200_arm_B_wrist_rotate");
//   curr_combined_js.name.push_back("wx200_arm_B_gripper");
//   curr_combined_js.name.push_back("wx200_arm_B_left_finger");
//   curr_combined_js.name.push_back("wx200_arm_B_right_finger");
//
//   curr_combined_js.name.push_back("wx200_arm_A_to_wx200_arm_B");
//
//   // copying position values
//   // curr_combined_js.position.push_back(2.0);
//   // curr_combined_js.position.push_back(2.0);
//   // curr_combined_js.position.push_back(2.0);
//   // curr_combined_js.position.push_back(2.0);
//   // curr_combined_js.position.push_back(2.0);
//   // curr_combined_js.position.push_back(2.0);
//   // curr_combined_js.position.push_back(2.0);
//   // curr_combined_js.position.push_back(2.0);
//   // curr_combined_js.position.push_back(2.0);
//   // curr_combined_js.position.push_back(2.0);
//   // curr_combined_js.position.push_back(2.0);
//   // curr_combined_js.position.push_back(2.0);
//   // curr_combined_js.position.push_back(2.0);
//   // curr_combined_js.position.push_back(2.0);
//   // curr_combined_js.position.push_back(2.0);
//   // curr_combined_js.position.push_back(2.0);
//   // curr_combined_js.position.push_back(2.0);
//
//
//   // copying position values
//   for (int i = 0; i < 8; i++){
//     curr_combined_js.position.push_back(curr_js_a.position[i]);
//   }
//   for (int i = 0; i < 8; i++){
//     curr_combined_js.position.push_back(curr_js_b.position[i]);
//   }
//   curr_combined_js.position.push_back(0.5);
//
//   // copying velocity values
//   // std::vector<int> combined_velocities;
//   // combined_velocities.reserve(js_a.velocity.size() + js_b.velocity.size());
//   // combined_velocities.insert(combined_velocities.end(), js_a.velocity.begin(), js_a.velocity.end());
//   // combined_velocities.insert(combined_velocities.end(), js_b.velocity.begin(), js_b.velocity.end());
//   // combined_velocities.push_back(0.0); // for the prismatic joint
//   // for (int i = 0; i < combined_velocities.size(); i++){
//   //   curr_combined_js.velocity.push_back(combined_velocities[i]);
//   // }
//
//   // copying effort values
//   // std::vector<int> combined_effort;
//   // combined_effort.reserve(js_a.effort.size() + js_b.effort.size());
//   // combined_effort.insert(combined_effort.end(), js_a.effort.begin(), js_a.effort.end());
//   // combined_effort.insert(combined_effort.end(), js_b.effort.begin(), js_b.effort.end());
//   // combined_effort.push_back(0.0); // for the prismatic joint
//   // for (int i = 0; i < combined_effort.size(); i++){
//   //   curr_combined_js.effort.push_back(combined_effort[i]);
//   // }
//
//   // publish the combined joint_state
//   pub_curr_combined_js.publish(curr_combined_js);
// }
//
// int main( int argc, char** argv )
// {
//   ros::init(argc, argv, "combining_arm_joints_node");
//
//   JointCombiner JointCombiner1;
//
//   // main loop
//   ros::Rate loop_rate(50);
//   while(ros::ok()){
//     ros::spinOnce();
//     JointCombiner1.combine_arm_joints();
//     loop_rate.sleep();
//   }
//
//   return 0;
// }
