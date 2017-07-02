#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
from __future__ import print_function
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
#from mpmath import *
#from sympy import *
import numpy as np
from math import atan2, cos, sin, asin, pi

# Joint angle symbols
#q0, q1, q2, q3, q4, q5, q6 = symbols('q0:7') # theta angle
#d0, d1, d2, d3, d4, d5, d6 = symbols('d0:7') # distance between frames (x) along z
#a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7') # distance between frames (z) along x
# angle change between zi-1 axis and zi about xi-1
#alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')

# DH Parameters
d_base = 0.75
a_1 = 0.35
a_2 = 1.25
a_3 = -0.054
d_3 = 1.5
d_6 = 0.303

"""
s = {alpha0:     0, a0:   0, d0: d_base, q0:       0,
     alpha1: -pi/2, a1: a_1, d1:      0,
     alpha2:     0, a2: a_2, q2:      0, q2: q2-pi/2,
     alpha3: -pi/2, a3: a_3, d3:    d_3,
     alpha4:  pi/2, a4:   0, d4:      0,
     alpha5: -pi/2, a5:   0, q5:      0,
     alpha6:     0, a6:   0, d6:    d_6}"""
# Create individual transformation matrices
def rot_x(q):
    R_x = np.array([[1, 0, 0],
                  [0, cos(q), -sin(q)],
                  [0, sin(q), cos(q)]])
    return R_x

def rot_y(q):
    R_y = np.array([[cos(q), 0, sin(q)],
                  [0, 1, 0],
                  [-sin(q), 0, cos(q)]])
    return R_y

def rot_z(q):
    R_z = np.array([[cos(q), -sin(q), 0],
                  [sin(q), cos(q), 0],
                  [0, 0, 1]])
    return R_z

def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print("No valid poses received")
        return -1
    else:
        # Initialize service response
        joint_trajectory_list = []

        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

            # Modified DH params
            """T0_1 = Matrix([[cos(q0), -sin(q0), 0, a0],
                           [sin(q0) * cos(alpha0), cos(q0) * cos(alpha0), -sin(alpha0), -sin(alpha0) * d0],
                           [sin(q0) * sin(alpha0), cos(q0) * sin(alpha0), cos(alpha0), cos(alpha0) * d0],
                           [0, 0, 0, 1]])
            T0_1 = T0_1.subs(s)

            T1_2 = Matrix([[cos(q1), -sin(q1), 0, a1],
                           [sin(q1) * cos(alpha1), cos(q1) * cos(alpha1), -sin(alpha1), -sin(alpha1) * d1],
                           [sin(q1) * sin(alpha1), cos(q1) * sin(alpha1), cos(alpha1), cos(alpha1) * d1],
                           [0, 0, 0, 1]])
            T1_2 = T1_2.subs(s)

            T2_3 = Matrix([[cos(q2), -sin(q2), 0, a2],
                           [sin(q2) * cos(alpha2), cos(q2) * cos(alpha2), -sin(alpha2), -sin(alpha2) * d2],
                           [sin(q2) * sin(alpha2), cos(q2) * sin(alpha2), cos(alpha2), cos(alpha2) * d2],
                           [0, 0, 0, 1]])
            T2_3 = T2_3.subs(s)

            T3_4 = Matrix([[cos(q3), -sin(q3), 0, a3],
                           [sin(q3) * cos(alpha3), cos(q3) * cos(alpha3), -sin(alpha3), -sin(alpha3) * d3],
                           [sin(q3) * sin(alpha3), cos(q3) * sin(alpha3), cos(alpha3), cos(alpha3) * d3],
                           [0, 0, 0, 1]])
            T3_4 = T3_4.subs(s)

            T4_5 = Matrix([[cos(q4), -sin(q4), 0, a4],
                           [sin(q4) * cos(alpha4), cos(q4) * cos(alpha4), -sin(alpha4), -sin(alpha4) * d4],
                           [sin(q4) * sin(alpha4), cos(q4) * sin(alpha4), cos(alpha4), cos(alpha4) * d4],
                           [0, 0, 0, 1]])
            T4_5 = T4_5.subs(s)

            T5_6 = Matrix([[cos(q5), -sin(q5), 0, a5],
                           [sin(q5) * cos(alpha5), cos(q5) * cos(alpha5), -sin(alpha5), -sin(alpha5) * d5],
                           [sin(q5) * sin(alpha3), cos(q3) * sin(alpha5), cos(alpha5), cos(alpha5) * d5],
                           [0, 0, 0, 1]])
            T5_6 = T5_6.subs(s)

            T6_7 = Matrix([[cos(q6), -sin(q6), 0, a6],
                           [sin(q6) * cos(alpha6), cos(q6) * cos(alpha6), -sin(alpha6), -sin(alpha6) * d6],
                           [sin(q6) * sin(alpha6), cos(q6) * sin(alpha6), cos(alpha6), cos(alpha6) * d6],
                           [0, 0, 0, 1]])
            T6_7 = T6_7.subs(s)

            # Define Modified DH Transformation matrix
            T0_3 = simplify(T0_1 * T1_2 * T2_3 * T3_4) # arm section
            T0_6 = simplify(T0_3 * T4_5 * T5_6 * T6_7)  # Spherical wrist section

            print("T0_6")
            print(T0_6)"""

            '''R3_6 =
            ([[-sin(q4) * sin(q6) + cos(q4) * cos(q5) * cos(q6), -sin(q4) * cos(q6) - sin(q6) * cos(q4) * cos(q5),
               -sin(q5) * cos(q4)],
              [sin(q5) * cos(q6), -sin(q5) * sin(q6), cos(q5)],
              [-sin(q4) * cos(q5) * cos(q6) - sin(q6) * cos(q4), sin(q4) * sin(q6) * cos(q5) - cos(q4) * cos(q6),
               sin(q4) * sin(q5)]])'''


            #grip_corr = simplify(R_z * R_y) # ROS gripper correction matrix
            #T_gripper = simplify(T0_6 * grip_corr)

            """R0_3 = T0_3[:3,:3]
            R_gripper = T_gripper[:3,:3]
            R4_6 = R0_3.T * R_gripper"""

            # Extract end-effector position and orientation from request
	        # px,py,pz = end-effector position
	        # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            #print("px ", px, " py ", py, " pz ", pz)


            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])

            # yaw, pitch, roll, R correction between frames
            Rrpy_cor = rot_z(yaw) * rot_y(pitch) * rot_x(roll) * rot_z(pi) * rot_y(-pi/2)
            #Rrpy_cor = rot_x(roll) * rot_y(pitch) * rot_x(yaw)

            # end coordinates of the arm and start coordinates of the wrist
            #c_px = px + d_6 #* float(Rrpy_cor[0, 2])
            #c_py = py + d_6 #* float(Rrpy_cor[1, 2])
            #c_pz = pz + d_6 #* float(Rrpy_cor[2, 2])
            #print("px ", type(c_px), " py ", c_py, " pz ", c_pz)
     
            # Calculate joint angles using Geometric IK method
            theta1 = atan2(py, px)

            # remove base offsets from the wrist coords for theta 2 and 3
            a_1_x = a_1 * cos(theta1) # link a_1 offset in x direction
            a_1_y = a_1 * sin(theta1)
            d_6_x = d_6 * cos(theta1) # link d_6 offset in x direction
            d_6_y = d_6 * sin(theta1)
            # get the x, y, z coordinates
            x_d = px - d_6_x - a_1_x
            y_d = py - d_6_y - a_1_y
            z_d = pz - d_base

            # x y plane arm projections
            r_xy = np.sqrt(x_d**2 + y_d**2)
            # x, y, z plane arm length
            r_xyz = np.sqrt(x_d**2 + y_d**2 + z_d**2)

            # calculate link 3 offsets and distance to arm end frame
            link_3 = np.sqrt(a_3**2 + d_3**2)
            link_3_theta = atan2(a_3, d_3)

            ''' theta 2 '''
            beta2 = atan2(r_xy, z_d) # link 2 angle from z axis / vertical (pi/4)

            # law of cosine rule
            D_theta2 = (a_2**2 + r_xyz**2 - link_3**2) / (2 * a_2 * r_xyz)
            alpha2 = atan2(np.sqrt(np.abs(1 - D_theta2**2)), D_theta2)
            theta2 = beta2 - alpha2

            ''' theta 3 '''
            # law of cosine rule
            D_theta3 = (a_2**2 + link_3**2 - r_xyz**2) / (2 * a_2 * link_3)
            alpha3 = atan2(np.sqrt(np.abs(1 - D_theta3**2)), D_theta3) + link_3_theta
            theta3 = (alpha3 - pi/2) * -1.0 # angle perpendicular wrt link 1 but alpha is from link 1

            if theta3 > pi * (155 - 91) / 180:
                theta3 = pi * (155 - 91) / 180
            if theta3 < -pi:
                theta3 = -pi

            # Using law of cosines to calculate theta 2 and theta 3
            #D = x_d**2 + y_d**2 + z_d**2 - a_2**2 - r_3**2
            #D = x_d**2 + z_d**2 - a_2**2 - r_3**2
            #D = D / (2 * a_2 * r_3)
            #D_theta2 = (a_2**2 + rxyz**2 - r_3**2) / (2 * a_2**2 * np.sqrt(h_2_3))

            #omega2 = atan2(np.sqrt(abs(1 - D_theta2**2)), D_theta2)
            #beta2 = asin(z_d / np.sqrt(h_2_3))
            #theta2 = (beta2 - pi/4) * -1. #+ beta2 - pi/4

            ''''#D_theta3 = (a_2 ** 2 - h_2_3 + r_3**2) / (2 * a_2**2 * r_3)
            #print("D ", type(tan_D), "   ", sqrt(tan_D))
            theta3 = (atan2(D_theta3, np.sqrt(abs(1 - D_theta3**2))) + r_3_theta) #* -1.0# * 10/180
            if theta3 > pi*(155-91)/180:
                theta3 = pi*(155-91)/180
            if theta3 < -pi:
                theta3 = -pi'''

            """
            #tan_S1 = symbols('tan_S1', real=True)
            tan_S1 = x_d**2 + y_d**2
            theta2 = atan2(np.sqrt(abs(tan_S1)), z_d**2)
            theta2 -= atan2(a_2 + r_3 * cos(-theta3), r_3 * sin(-theta3)) #- pi/4#*95/180
            theta2 = theta2 #+ pi * 10/180"""
            #print("theta2  ", theta2, "   alpha2 ", alpha2, "   beta2 ", beta2)

            theta4 = 0
            theta5 = 0
            theta6 = 0

            #theta2 = 0# -pi/4
            #theta1 = 0
            #theta3 = 0
            print("theta1: {:.3f} theta2: {:.3f} theta3: {:.3f} theta4: {:.3f}".
                  format(theta1*180/pi, theta2*180/pi, theta3*180/pi, alpha3*180/pi))

            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
            joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
            joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print("Ready to receive an IK request")
    rospy.spin()

if __name__ == "__main__":
    IK_server()
