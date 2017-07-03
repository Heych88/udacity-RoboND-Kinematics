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
from math import atan2, cos, sin, acos, pi

# Joint angle symbols
#q0, q1, q2, q3, q4, q5, q6 = symbols('q0:7') # theta angle
#d0, d1, d2, d3, d4, d5, d6 = symbols('d0:7') # distance between frames (x) along z
#a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7') # distance between frames (z) along x
# angle change between zi-1 axis and zi about xi-1
#alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')

# DH Parameters
'''
DH Table
Link    a       alpha   d       theta
1       a1      90      d1      *
2       a2      0       0       *
3       -a3     90      d3      *
4       0       -90     0       *
5       0       90      0       *
6       0       0       d6      *
'''
d_base = 0.75
a_1 = 0.35
a_2 = 1.25
a_3 = -0.054
d_3 = 1.5
d_6 = 0.303

# Create individual transformation matrices
def rot_x(angle):
    '''
    Rotation about x axis
    :param angle: amount of rotation about x
    :return: rotation matrix
    '''
    R_x = np.array([[1, 0, 0],
                    [0, cos(angle), -sin(angle)],
                    [0, sin(angle), cos(angle)]])
    return R_x

def rot_y(angle):
    '''
    Rotation about y axis
    :param angle: amount of rotation about y
    :return: rotation matrix
    '''
    R_y = np.array([[cos(angle), 0, sin(angle)],
                    [0, 1, 0],
                    [-sin(angle), 0, cos(angle)]])
    return R_y

def rot_z(angle):
    '''
    Rotation about z axis
    :param angle: amount of rotation about z
    :return: rotation matrix
    '''
    R_z = np.array([[cos(angle), -sin(angle), 0],
                  [sin(angle), cos(angle), 0],
                  [0, 0, 1]])
    return R_z

def rpy_matrix(roll, pitch, yaw):
    '''
    The rotation matrix with Roll, Pitch and Yaw
    :param roll: angle about z axis
    :param pitch: angle about y axis
    :param yaw: angle about x axis
    :return: rotation matrix
    '''
    # roll : about z axis
    # pitch : about y axis
    # yaw : about x axis
    r11 = cos(roll)*cos(pitch)
    r12 = -sin(roll)*cos(yaw) + cos(roll)*sin(pitch)*sin(yaw)
    r13 = sin(roll)*sin(yaw) + cos(roll)*sin(pitch)*cos(yaw)
    r21 = sin(roll)*cos(pitch)
    r22 = cos(roll)*cos(yaw) + sin(roll)*sin(pitch)*sin(yaw)
    r23 = -cos(roll)*sin(yaw) + sin(roll)*sin(pitch)*cos(yaw)
    r31 = -sin(pitch)
    r32 = cos(pitch)*sin(yaw)
    r33 = cos(pitch)*cos(yaw)

    return np.array([[r11, r12, r13],
                     [r21, r22, r23],
                     [r31, r32, r33]])

def link_transform(a, alpha, d, theta):
    '''
    original DH transformation matrix creator
    :param a: distance from Zi-1 to Zi, measured along Xi
    :param alpha: angle from Zi-1 to Zi, measured about Xi
    :param d:  distance from Xi-1 to Xi, measured along Zi-1
    :param theta: angle from Xi-1 to Xi, measured about Zi-1
    :return: link transformation matrix
    '''
    return np.array([[cos(theta), -sin(theta) * cos(alpha), sin(theta) * sin(alpha), a * cos(theta)],
                     [sin(theta), cos(theta) * cos(alpha), -cos(theta) * sin(alpha), a * cos(theta)],
                     [0, sin(alpha), cos(alpha), d],
                     [0, 0, 0, 1]])

def limit_angle(theta, min, max):
    if theta > pi * max / 180:
        return pi * max / 180
    elif theta < pi * min / 180:
        return pi * min / 180
    else:
        return theta

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

            # Extract end-effector position and orientation from request
	        # px,py,pz = end-effector position
	        # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            ''' theta 1 '''
            #   ^ y
            #   |
            #   |
            #   |
            #   o ----------> x
            #
            # Calculate joint angles using Geometric IK method
            theta1 = limit_angle(atan2(py, px), -185, 185)

            # remove base offsets from the wrist coords for theta 2 and 3
            a_1_x = a_1 * cos(theta1)  # link a_1 offset in x direction
            a_1_y = a_1 * sin(theta1)
            d_6_x = d_6 * cos(theta1)  # link d_6 offset in x direction
            d_6_y = d_6 * sin(theta1)
            # get the desired end arm x, y, z coordinates
            x_d = px - d_6_x - a_1_x
            y_d = py - a_1_y #- d_6_y
            z_d = pz - d_base + d_6/3

            # x y plane arm projections
            r_xy = np.sqrt(x_d ** 2 + y_d ** 2)
            # x, y, z 3D plane arm length
            r_xyz = np.sqrt(x_d ** 2 + y_d ** 2 + z_d ** 2)

            # calculate link 3 shoulder angle and distance to wrist center
            #         |----- d_3 ------| _
            #         O *               a_3
            #       *   * * * * * * * *  -
            #     *
            #   *
            link_3 = np.sqrt(a_3 ** 2 + d_3 ** 2)
            link_3_theta = atan2(a_3, d_3)

            ''' theta 2 '''
            #   ^ z
            #   |
            #   | a-b   o
            #   |---- *
            #   |   *
            #   | *
            #   o ----------> x
            #
            # link 1 to wrist center angle from z axis / vertical (pi/2)
            beta2 = atan2(r_xy, z_d)

            # law of cosine rule
            D_theta2 = (a_2 ** 2 + r_xyz ** 2 - link_3 ** 2) / (2 * a_2 * r_xyz)
            alpha2 = atan2(np.sqrt(np.abs(1 - D_theta2 ** 2)), D_theta2)
            # zero angle is along z axis
            theta2 = limit_angle(beta2 - alpha2, -45, 85)

            ''' theta 3 '''
            # law of cosine rule
            D_theta3 = (a_2 ** 2 + link_3 ** 2 - r_xyz ** 2) / (2 * a_2 * link_3)
            alpha3 = atan2(np.sqrt(np.abs(1 - D_theta3 ** 2)), D_theta3) + link_3_theta
            # angle perpendicular wrt link 1 but alpha is from link 1
            theta3 = limit_angle((alpha3 - pi / 2) * -1.0, -210, 155-90)

            # Spherical wrist
            # R3_6  [c4c5c6-s4s6, -c4c5s6-s4c6  , c4s5  ]
            #       [s4c5c6+c4s6, -s4c5s6+c4c6  , s4s5  ]
            #       [-s5c6      , s5s6          , c5    ]
            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                 req.poses[x].orientation.z, req.poses[x].orientation.w])
            # yaw, pitch, roll, R correction between frames
            Rrpy_cor = rpy_matrix(yaw, pitch, roll)  # * rot_y(pi/2) #* rot_z(pi)
            # print(Rrpy_cor)

            '''
            DH Table
            Link    a       alpha   d       theta
            1       a1      90      d1      *
            2       a2      0       0       *
            3       -a3     90      d3      *
            '''
            T0_3 = link_transform(a_1, pi/2, d_base, theta1)
            T0_3 = T0_3 * link_transform(a_2, 0, 0, theta2)
            T0_3 = T0_3 * link_transform(-a_3, pi / 2, d_3, theta3)

            R3_6 = np.transpose(T0_3[:3, :3]) * Rrpy_cor
            #print(R3_6)

            r11 = R3_6[0, 0]
            r12 = R3_6[0, 1]
            r13 = R3_6[0, 2]
            r21 = R3_6[1, 0]
            r22 = R3_6[1, 1]
            r23 = R3_6[1, 2]
            r31 = R3_6[2, 0]
            r32 = R3_6[2, 1]
            r33 = R3_6[2, 2]

            ''' theta 4 '''
            # rotate wrist to keep theta 5 along x axis
            #D_y = cos(theta1) * cos(r23) * r13 + sin(theta1) * cos(r23) * r23 + sin(r23) * r33
            #D_x = -cos(theta1) * sin(r23) * r13 - sin(theta1) * sin(r23) * r23 + cos(r23) * r33
            # theta4 = -(pi/2 * -sin(theta1)) #atan2(D_y, D_x)
            # theta4 = pi/2 * sin(theta1) + pi/2

            # create a plane onto the shelf with origin in the center and inverted axes
            # rotate wrist around origin so axis 5 is the tangent of the circle from the origin
            #           |                   |
            # ----------+-------------------+------------
            #           |               y   |
            #           |        o------->  |
            #           |        |          |
            #           |        |z         |
            # ----------+-------------------+------------
            #           |                   |
            z_o = (pz - (d_base + a_2)) * -1.  # origin of target shelf
            y_o = -py
            theta4 = -pi / 2 + atan2(z_o, y_o)
            theta4 = limit_angle(theta4, -350, 350)

            ''' theta 5 '''
            # keeps wrist level using geometric association laws
            #D_theta5 = sin(theta1) * r13 - cos(theta1) * r23
            #theta5 = atan2(D_theta5, np.sqrt(np.abs(1 - D_theta5**2)))
            alpha5 = pi - alpha2 - alpha3
            beta5 = pi/2 - beta2
            theta5 = -(alpha5 - beta5)
            if z_o < 0:
                theta5 = -theta5
            if sin(theta1) > 0.55:
                theta5 -= pi/2 * sin(theta1)
            #theta5 = acos((px*x_d + py*y_d + pz*z_d) / (r_xyz * np.sqrt(px**2 + py**2 + pz**2)))
            #theta5 = theta1 - atan2(py - y_d, px - x_d) - atan2(pz - z_d, px - x_d) + (alpha5 - beta5)
            theta5 = limit_angle(theta5, -125, 125)

            ''' theta 6 '''
            # rotate gripper keeping level wrt to the ground plane
            D_y = cos(theta1) * r21 - sin(theta1) * r11
            D_x = sin(theta1) * r12 - cos(theta1) * r22
            theta6 = -theta4 #atan2(D_y, D_x)
            #theta6 = (pi/2 - (alpha5 - beta5)) * -sin(theta1)
            theta6 = limit_angle(theta6, -350, 350)

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
