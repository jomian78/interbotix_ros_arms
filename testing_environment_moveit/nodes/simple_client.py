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
        send_to_home = rospy.ServiceProxy('/testing_environment/send_to_home', Empty)
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
        send_to_custom_angles = rospy.ServiceProxy('/testing_environment/send_to_custom_angles', CustomAngle)
        resp1 = send_to_custom_angles(arm_a_angles, arm_b_angles)
        return resp1
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


'''
Standard positions for arms A and B to present objects to the rats
'''
# def two_options_client(p1,p2):
#     print "Requesting two options"
#     rospy.wait_for_service('two_options')
#     try:
#         two_options = rospy.ServiceProxy('two_options', two_options)
#         resp1 = two_options(p1,p2)
#         return resp1.confirm
#     except rospy.ServiceException, e:
#         print "Service call failed: %s"%e

'''
Generate positions within a sphere
of radius r (with no collisions)
for both arms
'''
# def new_positions_client(r):
#     print "Requesting new positions"
#     rospy.wait_for_service('new_positions')
#     try:
#         new_positions = rospy.ServiceProxy('new_positions', new_positions)
#         resp1 = new_positions(r)
#         return resp1.confirm
#     except rospy.ServiceException, e:
#         print "Service call failed: %s"%e

'''
Send the arms
to a sequence of waypoints
specified by the user
'''
# def waypoints_client(set1, set2):
#     print "Requesting arms move to waypoints"
#     rospy.wait_for_service('waypoints')
#     try:
#         waypoints = rospy.ServiceProxy('waypoints', waypoints)
#         resp1 = waypoints(set1, set2)
#         return resp1.confirm
#     except rospy.ServiceException, e:
#         print "Service call failed: %s"%e

'''
Swap the object on arm A
after the arm has been moved to
the home position (specify CW or CCW?)
'''
# def swap_A_client():
#     print "swapping object A"
#     rospy.wait_for_service('swap_A')
#     try:
#         swap_A = rospy.ServiceProxy('swap_A', swap_A)
#         resp1 = swap_A()
#         return resp1.confirm
#     except rospy.ServiceException, e:
#         print "Service call failed: %s"%e

'''
Swap the object on arm B
after the arm has been moved to
the home position (specify CW or CCW?)
'''
# def swap_B_client():
#     print "swapping object B"
#     rospy.wait_for_service('swap_B')
#     try:
#         swap_B = rospy.ServiceProxy('swap_B', swap_B)
#         resp1 = swap_B()
#         return resp1.confirm
#     except rospy.ServiceException, e:
#         print "Service call failed: %s"%e


'''
        Create a motion sequence for the arms
        by calling the desired ROS services
        in order in the main function
'''
if __name__ == "__main__":
    # print "Running send_to_home client"

    # call services here
    # send_to_home_client()

    print "Running send_to_custom_angles_client client"
    # CW = +, CCW = -
    a = np.array([0.0, 0.0, 0.0, 0.2, 0.0]) # waist,shoulder,elbow,wrist angle, wrist rotate
    b = np.array([0.0, 0.0, 0.0, -0.2, 0.0]) # waist,shoulder,elbow,wrist angle, wrist rotate
    send_to_custom_angles_client(a, b)
