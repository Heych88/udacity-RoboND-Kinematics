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
d_4 = 1.5
d_6 = 0.303
'''
Link    a       alpha   d       theta
1       a1      90      d1      *
2       a2      0       0       *
3       -a3     90      0       *
4       0       -90     d4      *
5       0       90      0       *
6       0       0       d6      *
'''

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

def link_transform(a, alpha, d, theta):
    return np.array([[cos(theta), -sin(theta) * cos(alpha), sin(theta) * sin(alpha), a * cos(theta)],
                     [sin(theta), cos(theta) * cos(alpha), -cos(theta) * sin(alpha), a * cos(theta)],
                     [0, sin(alpha), cos(alpha), d],
                     [0, 0, 0, 1]])

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

            '''R3_6 =
            ([[-sin(q4) * sin(q6) + cos(q4) * cos(q5) * cos(q6), -sin(q4) * cos(q6) - sin(q6) * cos(q4) * cos(q5),
               -sin(q5) * cos(q4)],
              [sin(q5) * cos(q6), -sin(q5) * sin(q6), cos(q5)],
              [-sin(q4) * cos(q5) * cos(q6) - sin(q6) * cos(q4), sin(q4) * sin(q6) * cos(q5) - cos(q4) * cos(q6),
               sin(q4) * sin(q5)]])'''

            # Extract end-effector position and orientation from request
	        # px,py,pz = end-effector position
	        # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            ''' theta 1 '''
            # Calculate joint angles using Geometric IK method
            theta1 = atan2(py, px)

            # remove base offsets from the wrist coords for theta 2 and 3
            a_1_x = a_1 * cos(theta1)  # link a_1 offset in x direction
            a_1_y = a_1 * sin(theta1)
            d_6_x = d_6 * cos(theta1)  # link d_6 offset in x direction
            d_6_y = d_6 * sin(theta1)
            # get the x, y, z coordinates
            x_d = px - d_6_x - a_1_x
            y_d = py - d_6_y - a_1_y
            z_d = pz - d_base

            # x y plane arm projections
            r_xy = np.sqrt(x_d ** 2 + y_d ** 2)
            # x, y, z plane arm length
            r_xyz = np.sqrt(x_d ** 2 + y_d ** 2 + z_d ** 2)

            # calculate link 3 offsets and distance to arm end frame
            link_3 = np.sqrt(a_3 ** 2 + d_4 ** 2)
            link_3_theta = atan2(a_3, d_4)

            ''' theta 2 '''
            beta2 = atan2(r_xy, z_d)  # link 2 angle from z axis / vertical (pi/4)

            # law of cosine rule
            D_theta2 = (a_2 ** 2 + r_xyz ** 2 - link_3 ** 2) / (2 * a_2 * r_xyz)
            alpha2 = atan2(np.sqrt(np.abs(1 - D_theta2 ** 2)), D_theta2)
            theta2 = beta2 - alpha2

            ''' theta 3 '''
            # law of cosine rule
            D_theta3 = (a_2 ** 2 + link_3 ** 2 - r_xyz ** 2) / (2 * a_2 * link_3)
            alpha3 = atan2(np.sqrt(np.abs(1 - D_theta3 ** 2)), D_theta3) + link_3_theta
            theta3 = (alpha3 - pi / 2) * -1.0  # angle perpendicular wrt link 1 but alpha is from link 1

            if theta3 > pi * (155 - 91) / 180:
                theta3 = pi * (155 - 91) / 180
            if theta3 < -pi:
                theta3 = -pi


            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])

            # yaw, pitch, roll, R correction between frames
            Rrpy_cor = rot_z(yaw) * rot_y(pitch) * rot_x(roll) #* rot_y(pi/2) * rot_z(pi)
            #Rrpy_cor = rot_x(roll) * rot_y(pitch) * rot_x(yaw)

            # end coordinates of the arm and start coordinates of the wrist
            #c_px = px + d_6 #* float(Rrpy_cor[0, 2])
            #c_py = py + d_6 #* float(Rrpy_cor[1, 2])
            #c_pz = pz + d_6 #* float(Rrpy_cor[2, 2])
            #print("px ", type(c_px), " py ", c_py, " pz ", c_pz)

            print(type(roll), "   ", roll)

            # Spherical wrist
            # R3_6  [c4c5c6-s4s6, -c4c5s6-s4c6  , c4s5  ]
            #       [s4c5c6+c4s6, -s4c5s6+c4c6  , s4s5  ]
            #       [-s5c6      , s5s6          , c5    ]
            #print('Rrpy_cor')
            #print(Rrpy_cor)
            r13 = Rrpy_cor[0, 2]
            r23 = Rrpy_cor[1, 2]
            r33 = Rrpy_cor[2, 2]
            r11 = Rrpy_cor[0, 0]
            r21 = Rrpy_cor[1, 0]
            r12 = Rrpy_cor[0, 1]
            r22 = Rrpy_cor[1, 1]

            ''' theta 5 '''
            #D_theta5 = sin(theta1) * r13 - cos(theta1) * r23
            #theta5 = atan2(D_theta5, np.sqrt(np.abs(1 - D_theta5**2)))
            alpha5 = pi - alpha2 - alpha3
            beta5 = pi/2 - beta2
            theta5 = (alpha5 - beta5) * (-1.0 - sin(abs(theta1)))

            ''' theta 4 '''
            #D_y = cos(theta1)*cos(theta1)
            #theta4 = atan2(r13, r23)
            theta4 = pi/2 * sin(theta1)

            ''' theta 6 '''
            #D_y = cos(theta1) * r21 - sin(theta1) * r11
            #D_x = sin(theta1) * r12 + cos(theta1) * r22
            #theta6 = atan2(D_y, D_x)
            theta6 = (pi/2 - (alpha5 - beta5)) * -sin(theta1)

            #theta4 = 0
            #theta5 = -pi/4
            #theta6 = 0

            print("t1: {:.3f}   t2: {:.3f}   t3: {:.3f}   t4: {:.3f}   t5: {:.3f}   t6: {:.3f}".
                  format(theta1*180/pi, theta2*180/pi, theta3*180/pi, theta4*180/pi, theta5*180/pi, theta6*180/pi))

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
