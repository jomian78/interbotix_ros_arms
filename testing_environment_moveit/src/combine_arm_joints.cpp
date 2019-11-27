#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <message_filters/subscriber.h>
// #include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

class JointCombiner{
  public:
    // JointCombiner():sync(wx200_arm_A_sub, wx200_arm_B_sub, 10){
    JointCombiner():sync(MySyncPolicy(10), wx200_arm_A_sub, wx200_arm_B_sub){

      // combined joint_state publisher
      pub_curr_combined_js = nh.advertise<sensor_msgs::JointState>("joint_states", 100);

      // set message_filters
      wx200_arm_A_sub.subscribe(nh, "/testing_environment/wx200_arm_A/joint_states", 1);
      wx200_arm_B_sub.subscribe(nh, "/testing_environment/wx200_arm_B/joint_states", 1);

      sync.registerCallback(&JointCombiner::callback, this);
    };

    void combine_arm_joints();

  private:
    ros::NodeHandle nh;

    // most recent joint_states variables from each arm
    sensor_msgs::JointState curr_js_a;
    sensor_msgs::JointState curr_js_b;

    // message_filters
    message_filters::Subscriber<sensor_msgs::JointState> wx200_arm_A_sub;
    message_filters::Subscriber<sensor_msgs::JointState> wx200_arm_B_sub;

    // TimeSynchronizer
    // message_filters::TimeSynchronizer<sensor_msgs::JointState, sensor_msgs::JointState> sync;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::JointState, sensor_msgs::JointState> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync;

    // Publisher for combined joint_states
    ros::Publisher pub_curr_combined_js;

    // TimeSynchronizer callback
    void callback(const sensor_msgs::JointState& js_a, const sensor_msgs::JointState& js_b);

};

// TimeSynchronizer callback
void JointCombiner::callback(const sensor_msgs::JointState& js_a, const sensor_msgs::JointState& js_b){
  curr_js_a = js_a;
  curr_js_b = js_b;
}

// class function for combining the joints from different arms
void JointCombiner::combine_arm_joints(){

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
  for (int i = 0; i < curr_js_a.position.size(); i++){
    curr_combined_js.position.push_back(curr_js_a.position[i]);
  }
  for (int i = 0; i < curr_js_b.position.size(); i++){
    curr_combined_js.position.push_back(curr_js_b.position[i]);
  }

  // copying velocity values
  for (int i = 0; i < curr_js_a.velocity.size(); i++){
    curr_combined_js.velocity.push_back(curr_js_a.velocity[i]);
  }
  for (int i = 0; i < curr_js_b.velocity.size(); i++){
    curr_combined_js.velocity.push_back(curr_js_b.velocity[i]);
  }

  // copying position values
  for (int i = 0; i < curr_js_a.effort.size(); i++){
    curr_combined_js.effort.push_back(curr_js_a.effort[i]);
  }
  for (int i = 0; i < curr_js_b.effort.size(); i++){
    curr_combined_js.effort.push_back(curr_js_b.effort[i]);
  }

  // publish the combined joint_state
  pub_curr_combined_js.publish(curr_combined_js);
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "combining_arm_joints_node");

  JointCombiner JointCombiner1;

  // main loop
  ros::Rate loop_rate(50);
  while(ros::ok()){
    ros::spinOnce();
    JointCombiner1.combine_arm_joints();
    loop_rate.sleep();
  }

  return 0;
}
