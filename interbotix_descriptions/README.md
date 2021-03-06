# interbotix_descriptions

## Overview
This package contains the URDFs and meshes for the many X-Series Interbotix arms and turrets. The STL files for each robot are located in a unique folder inside the [meshes](meshes/) directory. Also in the 'meshes' directory is the [interbotix_black.png](meshes/interbotix_black.png) picture. The appearance and texture of the robots come from this picture. Next, the URDFs for the robot are located in the [urdf](urdf/) directory. They are written in 'xacro' format so that users have the ability to customize what parts of the URDF get loaded to the parameter server (see the 'Usage' section below for details). Note that all the other ROS packages in the repo reference this package to launch the robot model.

## Structure
![interbotix_descriptions_flowchart](images/interbotix_descriptions_flowchart.png)
This package contains the [description.launch](launch/description.launch) file responsible for loading parts or all of the robot model. It launches up to four nodes as described below:
- **joint_state_publisher** - responsible for parsing the 'robot_description' parameter to find all non-fixed joints and publish a JointState message with those joints defined.
- **robot_state_publisher** - uses the URDF specified by the parameter robot_description and the joint positions from the joint_states topic to calculate the forward kinematics of the robot and publish the results via tf.
- **static_transform_publisher** - publishes a static transform from the 'world' frame in Rviz to the 'base_link' frame of a robot.
- **rviz** - displays the virtual robot model using the transforms in the 'tf' topic.

## Usage
To run this package, type the line below in a terminal. Note that the `robot_name` argument must be specified as the first five characters of one of the URDF files located in the [urdf](/urdf) directory. For example, to launch the ReactorX 150 arm, type:
```
$ roslaunch interbotix_descriptions description.launch robot_name:=rx150
```
This is the bare minimum needed to get up and running. Take a look at the table below to see how to further customize with other launch file arguments.

| Argument | Description | Default Value |
| -------- | ----------- | :-----------: |
| robot_name | the first five characters of one of the files in the [urdf](urdf/) directory | "" |
| robot_model | only used when launching multiple robots; if that's the case, this should be set to the first five characters of one of the files in the [urdf](urdf/) directory; 'robot_name' should then be set to a unique name followed by '$(arg robot_model)' | '$(arg robot_name)' |
| use_joint_pub | launches the joint_state_publisher node | true |
| jnt_pub_gui | launches the joint_state_publisher GUI | true |
| use_default_rviz | launches the rviz and static_transform_publisher nodes | true |
| use_default_gripper_bar | if true, the gripper_bar link is also loaded to the 'robot_description' parameter; if false, the gripper_bar link and any other link past it in the kinematic chain is not loaded to the parameter server. Set to 'false' if you have a custom gripper attachment | true |
| use_default_gripper_fingers | if true, the gripper fingers are also loaded to the 'robot_description' parameter; if false, the gripper fingers and any other link past it in the kinematic chain is not loaded to the parameter server. Set to 'false' if you have custom gripper fingers | true |
| use_external_gripper_urdf | if you have a URDF of a custom gripper attachment, set this to 'true' | false |
| external_gripper_urdf_loc | set the file path to where your custom gripper attachment URDF is located | "" |
| load_gazebo_material | set this to 'true' if Gazebo is being used; it makes sure to also load Gazebo related configs to the 'robot_description' parameter so that the robot models show up black in Gazebo | false |
| load_moveit_world_frame | set this to 'true' if MoveIt is being used; it makes sure to also load a 'world' frame to the 'robot_description' parameter which is located exactly at the 'base_link' frame of the robot arm | false |
| rvizconfig | file path to the config file Rviz should load | refer to [description.launch](launch/description.launch) |
| model | file path to the robot-specific URDF including arguments to be passed in | refer to [description.launch](launch/description.launch) |
