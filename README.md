# Dynamic Animal Testing Environment: ROS Melodic

## Overview
This repository contains code and documentation for the dynamic animal testing environment. A high-level description of the project can be found on my [portfolio](https://jomian78.github.io/).

## Quick start
1) Navigate to your `/home` directory (or your directory of choice)
2) Clone this repository:
```
git clone https://github.com/jomian78/interbotix_ros_arms
```
3) Place the bases of the arms at your desired positions, measure the relative x,y,z positions and r,p,y orientations with respect to your refernce point

4) To start the environment, run:
```
roslaunch testing_environment_moveit testing_environment.launch relative_x:=0.0 relative_y:=0.24 relative_z:=0.0 relative_roll:=0,0 relative_pitch:=0.0 relative_yaw:=0.0
```

5) To send the arms through a quick series of end-effector way-points, run:
```
rosrun testing_environment_moveit move_group_test
```

## Manual Poses
1) To start the environment, run:
```
roslaunch testing_environment_moveit testing_environment.launch relative_x:=0.0 relative_y:=0.24 relative_z:=0.0 relative_roll:=0,0 relative_pitch:=0.0 relative_yaw:=0.0
```

2) Make sure the `manual_poses` and `run_through_waypoints` nodes are uncommented in your
`multi_arm.launch` file

3) To torque the motors off on both arms and begin moving them to manual positions, run:
```
rosrun testing_environment_moveit manual_poses
```
4) When you have moved both of the arms to a desired pose, run:
```
rosrun testing_environment_moveit waypoint_1
```
to save this pose as a waypoint in your motion plan.

5) Repeat this step for two additional waypoints in the sequence
6) Run:
```
rosrun testing_environment_moveit run_through_waypoints
```
to move the arms through all the waypoints in your sequence. The arms will move to a reset position before commencing the sequence.

## Generating experimental trials
1) To start the environment, run:
```
roslaunch testing_environment_moveit testing_environment.launch relative_x:=0.0 relative_y:=0.24 relative_z:=0.0 relative_roll:=0,0 relative_pitch:=0.0 relative_yaw:=0.0
```
2) Run:
```
rosrun testing_environment_moveit pseudo_random_arm_poses
```
to generate a set of pseudo_random end effector positions within a specified bounding box in front of the animal test subject.

## udev rules

#### Verification
1) Copy the `10-interbotix-udev.rules` file from the `/hardware_config_files` directory to the `/etc/udev/rules.d` directory on your system

2) Enter the commands:

```
$ sudo udevadm control --reload-rules
$ sudo udevadm trigger
```

3) Then, enter the command:
```
ls /dev/widow*
```

4) Verify that symbolic links for
```
/dev/widow_200_arm_A
/dev/widow_200_arm_B
```
appear in your terminal

#### Adding new robot arms to the setup
- U2D2 serial numbers are used to identify each unique arm in the testing environment.

1) To identify the serial number of a particular U2D2 converter, connect the device to your PC's USB port

2) Run `ls /dev/tty*` to identify which port it is connected to

3) If the connected port is `/dev/ttyUSB0` (for example), enter the command:
```
udevadm info /dev/ttyUSB0 | grep ID_SERIAL
```
to get the device serial number.

4) Add the line:

```
ACTION=="add", ATTRS{serial}=="", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6014", ENV{ID_MM_DEVICE_IGNORE}="1", ATTR{device/latency_timer}="1", SYMLINK+=""

```
to the 10-interbotix-udev.rules file, with your ID number in the `ATTRS{serial}==`
field, and a symlink for the arm in the form `robotmodel_arm_a-z` in the `SYMLINK+=` FIELD
