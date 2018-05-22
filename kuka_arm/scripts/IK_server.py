#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright (C) 2017 Udacity Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *
from time import time

# Make Homegeneous Matrix from DH parameters 
def matrix_from(alpha, a, q, d):
    return Matrix([[ cos(q),            -sin(q),           0,           a              ],
                   [ sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha) * d],
                   [ sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha) * d],
                   [ 0,                 0,                 0,           1              ]
                  ])

# Make Rotation Matrix
# x-axis
def rot_x(q):
    return Matrix([[ 1,             0,       0],
                   [ 0,        cos(q), -sin(q)],
                   [ 0,        sin(q),  cos(q)]])
# y-axis    
def rot_y(q):
    return Matrix([[ cos(q),        0,  sin(q)],
                   [      0,        1,       0],
                   [-sin(q),        0,  cos(q)]])
# z-axis
def rot_z(q):    
    return Matrix([[ cos(q),   -sin(q),        0],
                   [ sin(q),    cos(q),        0],
                   [ 0,              0,        1]])


# Obtain euler angles from rotation matrix
# ex. rot = t4 * t5 * t6
def euler_angles_from_rotation_matrix(rot):
    # simplify(t4 * t5 * t6 * t7)[0:3,0:3]
    # = [[*,              *,               -sin(q5)cos(q4)],
    #    [sin(q5)cos(q6), -sin(q5)sin(q6), cos(q5)        ],
    #    [*,              *,               sin(q4)sin(q5) ]]
    
    # atan2(sin(q4)sin(q5), cos(q4)sin(q5))
    alpha = atan2(rot[2,2], -rot[0,2])
    # atan2(sin(q5), cos(q5))
    beta  = atan2(sqrt(rot[0,2]**2 + rot[2,2]**2), rot[1,2])
    # atan2(sin(q5)sin(q6), sin(q5)cos(q6))
    gamma = atan2(-rot[1,1], rot[1,0])
    
    return (alpha, beta, gamma)


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print ("No valid poses received")
        return -1
    else:
        start_time = time()

        ### Your FK code here
        # Create symbols
        #
        #   α_{i-1} for Rx
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
        #   a_{i-1} for Dx
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
        #   θi for Rz, **Joint Angles**
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
        #   di for Dz
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')

        #
        # Create Modified DH parameters
        #
        # Define dh parameters
        dh_params = {
            alpha0: 0,     a0: 0,      q1: q1,      d1: 0.33+0.42,
            alpha1: -pi/2, a1: 0.35,   q2: q2-pi/2, d2: 0,
            alpha2: 0,     a2: 1.25,   q3: q3,      d3: 0,
            alpha3: -pi/2, a3: -0.054, q4: q4,      d4: 0.96+0.54,
            alpha4: pi/2,  a4: 0,      q5: q5,      d5: 0,
            alpha5: -pi/2, a5: 0,      q6: q6,      d6: 0,
            alpha6: 0,     a6: 0,      q7: 0,       d7: 0.193+0.11,
        }

        #
        # Define Modified DH Transformation matrix
        #
        t1 = matrix_from(alpha0, a0, q1, d1).subs(dh_params)
        t2 = matrix_from(alpha1, a1, q2, d2).subs(dh_params)
        t3 = matrix_from(alpha2, a2, q3, d3).subs(dh_params)
        t4 = matrix_from(alpha3, a3, q4, d4).subs(dh_params)
        t5 = matrix_from(alpha4, a4, q5, d5).subs(dh_params)
        t6 = matrix_from(alpha5, a5, q6, d6).subs(dh_params)
        t7 = matrix_from(alpha6, a6, q7, d7).subs(dh_params)
        #t_ee = t1 * t2 * t3 * t4 * t5 * t6 * t7


        #
        # Create individual transformation matrices
        #
        r, p, y = symbols('roll pitch yaw')
        # Generate rotation matrix
        rot_fix = rot_z(pi) * rot_y(-pi/2)  # Fix for URDF <-> DH-convertion
        _rot_ee = rot_z(y) * rot_y(p) * rot_x(r) * rot_fix

        ###

        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            start_each_time = time()

            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

            # Extract end-effector position and orientation from request
            # px,py,pz = end-effector position
            # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])

            ### Your IK code here
            pos_ee = Matrix([px, py, pz])

            # Compensate for rotation discrepancy between DH parameters and Gazebo
            rot_ee = _rot_ee.subs({r: roll, p: pitch, y: yaw})
            pos_wc = pos_ee - dh_params[d7] * rot_ee[:,2]

            #
            # Calculate joint angles using Geometric IK method
            #
            # Project the position to the plane, and thus obtain angle of (x, y) 
            theta1 = atan2(pos_wc[1], pos_wc[0]).evalf()


            # Obtain side lengths of fig3 using pythagorean theorem
            fig3_side_a = pos_wc[0] # x
            fig3_side_b = pos_wc[1] # y
            fig3_side_c = sqrt(fig3_side_a ** 2 + fig3_side_b ** 2)

            # Obtain side lengths of fig2 using pythagorean theorem
            fig2_side_a = pos_wc[2] - dh_params[d1]         # z - height of joint1 
            fig2_side_b = fig3_side_c - dh_params[a1]       # side_b-offset
            fig2_side_c = sqrt(fig2_side_a ** 2 + fig2_side_b ** 2)

            # Obtain side lengths of fig1 using pythagorean theorem
            fig1_side_a = dh_params[d4]
            fig1_side_b = fig2_side_c
            fig1_side_c = dh_params[a2]

            ## Obtain angles of fig1
            #
            # Get triangle angles using sides of triangle
            #
            # ref.low of cosines https://en.wikipedia.org/wiki/Law_of_cosines#Applications
            def angles_from_triangle_sides(a, b, c):
                """:a, b, c: side values of triangle"""
                gamma = acos((a**2 + b**2 - c**2) / (2 * a * b))
                beta = acos((a**2 + c**2 - b**2) / (2 * a * c))
                alpha = acos((b**2 + c**2 - a**2) / (2 * b * c))
                return (alpha, beta, gamma)
            
            fig1_angle_a, fig1_angle_b, _ = angles_from_triangle_sides(
                fig1_side_a, fig1_side_b, fig1_side_c)

            theta2 = pi/2 - fig1_angle_a - atan2(fig2_side_a, fig2_side_b).evalf()
            theta3 = pi/2 - fig1_angle_b + atan2(dh_params[a3], dh_params[d4]).evalf()

            #
            # Extract rotation matrices from the transformation matrices
            #
            # Slice Rotation Matrix, i=1..3
            rot0_3 = (t1 * t2 * t3)[0:3, 0:3].evalf(
                subs={q1: theta1, q2: theta2, q3: theta3})

            # Obtain the Rotation Matrix, i=3..6
            # Memo: 
            #   r: rotation_matrix
            #   inv(r) == r^T
            rot3_6 = rot0_3.T * rot_ee

            #_rot3_6 = rot3_6
            theta4, theta5, theta6 = euler_angles_from_rotation_matrix(rot3_6)
            #
            ###
            rospy.loginfo("  Each time[%d]: %04.4f seconds" % (x, time()-start_each_time))

            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
            joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
            joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("  Total run time: %04.4f seconds" % (time()-start_time))
        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print ("Ready to receive an IK request")
    rospy.spin()

if __name__ == "__main__":
    IK_server()
