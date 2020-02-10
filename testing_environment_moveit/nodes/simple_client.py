#!/usr/bin/env python
import sys
import rospy
import tf
import numpy as np
import geometry_msgs.msg
from std_msgs.msg import String
from std_msgs.msg import Bool
from std_srvs.srv import Empty
from testing_environment_moveit.srv import *

'''
Send the arms to their home position at the edge of the workspace
'''
def send_to_home_client():
    print "Requesting send_to_home service"
    rospy.wait_for_service('/testing_environment/send_to_home')
    try:
        send_to_home = rospy.ServiceProxy('/testing_environment/send_to_home', Empty, persistent=True)
        resp1 = send_to_home()
        return resp1
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

'''
Send the arms to custom positions using joint angles
'''
def send_to_custom_angles_client(arm_a_angles, arm_b_angles):
    print "Requesting send_to_custom_angles service"
    rospy.wait_for_service('/testing_environment/send_to_custom_angles')
    try:
        send_to_custom_angles = rospy.ServiceProxy('/testing_environment/send_to_custom_angles', CustomAngle, persistent=True)
        resp1 = send_to_custom_angles(arm_a_angles, arm_b_angles)
        return resp1
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


'''
Send the arms to custom end-effector positions/orientations (w.r.t to our perch reference point)
'''
def send_to_custom_positions_client(arm_a_x,arm_a_y, arm_a_z, arm_b_x, arm_b_y, arm_b_z):
    print "Requesting send_to_custom_positions service"
    rospy.wait_for_service('/testing_environment/send_to_custom_positions')
    try:
        send_to_custom_positions = rospy.ServiceProxy('/testing_environment/send_to_custom_positions', CustomPosition, persistent=True)
        resp1 = send_to_custom_positions(arm_a_x,arm_a_y, arm_a_z, arm_b_x, arm_b_y, arm_b_z)
        return resp1
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


'''
        Create a motion sequence for the arms
        by calling the desired ROS services
        in order in the main function
'''
if __name__ == "__main__":
    # # call services here
    # print "Running send_to_home client"
    # # Home position angles: 0.0 for all joints
    # send_to_home_client()
    #
    # print "Running send_to_custom_angles_client client"
    # # CW = +, CCW = -
    # a = np.array([0.0, 0.0, 0.0, 0.2, 0.0]) # waist,shoulder,elbow,wrist angle, wrist rotate
    # b = np.array([0.0, 0.0, 0.0, -0.2, 0.0]) # waist,shoulder,elbow,wrist angle, wrist rotate
    # send_to_custom_angles_client(a, b)
    #
    # print "Running send_to_home client a second time"
    # # Home position angles: 0.0 for all joints
    # send_to_home_client()
    #
    # print "Running send_to_custom_angles_client a second time"
    # # CW = +, CCW = -
    # a = np.array([-1.0, 0.0, 0.0, 0.4, 0.0]) # waist,shoulder,elbow,wrist angle, wrist rotate
    # b = np.array([-1.0, 0.0, 0.0, -0.4, 0.0]) # waist,shoulder,elbow,wrist angle, wrist rotate
    # send_to_custom_angles_client(a, b)

    print "Running send_to_custom_positions_client"
    # test case 1
    # arm_a_x = 0.3
    # arm_a_y = -0.2
    # arm_a_z = 0.3
    # arm_a_yaw = -0.05
    #
    # arm_b_x = 0.3
    # arm_b_y = 0.42
    # arm_b_z = 0.3
    # arm_b_yaw = 0.05

    # test case 2
    arm_a_x = 0.2
    arm_a_y = 0.05
    arm_a_z = 0.3
    arm_a_yaw = -0.05

    arm_b_x = 0.2
    arm_b_y = 0.17
    arm_b_z = 0.3
    arm_b_yaw = 0.05
    send_to_custom_positions_client(arm_a_x,arm_a_y, arm_a_z, arm_b_x, arm_b_y, arm_b_z)
