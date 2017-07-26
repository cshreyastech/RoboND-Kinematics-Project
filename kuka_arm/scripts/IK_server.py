#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
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
import numpy as np
import datetime

# Defing rotational matrix about x,y ans z
def rot_x(q):
    R_x = Matrix([[1,      0,       0],
                  [0, cos(q), -sin(q)],
                  [0, sin(q),  cos(q)]])

    return R_x


def rot_y(q):
    R_y = Matrix([[ cos(q), 0, sin(q)],
                  [      0, 1,      0],
                  [-sin(q), 0, cos(q)]])

    return R_y


def rot_z(q):
    R_z = Matrix([[cos(q), -sin(q), 0],
                  [sin(q),  cos(q), 0],
                  [     0,       0, 1]])

    return R_z

def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
        # Initialize service response
        joint_trajectory_list = []

        # Define DH param symbols
	d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
	a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
	alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')

        # Joint angle symbols
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8') # joint variables: theta_i
        roll, pitch, yaw = symbols('roll pitch yaw')

        # Modified DH params
	s = {alpha0:0,     a0:0,        d1:0.75,
	     alpha1:-pi/2, a1:0.35,     d2:0,        q2:q2 - pi / 2,
	     alpha2:0,     a2:1.25,     d3:0,
	     alpha3:-pi/2, a3:-0.054,   d4:1.50,
	     alpha4:pi/2,  a4:0,        d5:0,
	     alpha5:-pi/2, a5:0,        d6:0,
	     alpha6:0,     a6:0,        d7:0.303,    q7:0}

        # Define Modified DH Transformation matrix
	T0_1 = Matrix([[             cos(q1),            -sin(q1),            0,              a0 ],
		       [ sin(q1)*cos(alpha0), cos(q1)*cos(alpha0), -sin(alpha0), -sin(alpha0)*d1 ],
	               [ sin(q1)*sin(alpha0), cos(q1)*sin(alpha0),  cos(alpha0),  cos(alpha0)*d1 ],
            	       [                   0,                   0,            0,               1 ]])
	T0_1 = T0_1.subs(s)


	T1_2 = Matrix([[             cos(q2),            -sin(q2),            0,              a1 ],
          	       [ sin(q2)*cos(alpha1), cos(q2)*cos(alpha1), -sin(alpha1), -sin(alpha1)*d2 ],
	               [ sin(q2)*sin(alpha1), cos(q2)*sin(alpha1),  cos(alpha1),  cos(alpha1)*d2 ],
		       [                   0,                   0,            0,               1 ]])
	T1_2 = T1_2.subs(s)


	T2_3 = Matrix([[             cos(q3),            -sin(q3),            0,              a2 ],
        	       [ sin(q3)*cos(alpha2), cos(q3)*cos(alpha2), -sin(alpha2), -sin(alpha2)*d3 ],
	               [ sin(q3)*sin(alpha2), cos(q3)*sin(alpha2),  cos(alpha2),  cos(alpha2)*d3 ],
            	       [                   0,                   0,            0,               1 ]])
	T2_3 = T2_3.subs(s)


	T3_4 = Matrix([[             cos(q4),            -sin(q4),            0,              a3 ],
        	       [ sin(q4)*cos(alpha3), cos(q4)*cos(alpha3), -sin(alpha3), -sin(alpha3)*d4 ],
	               [ sin(q4)*sin(alpha3), cos(q4)*sin(alpha3),  cos(alpha3),  cos(alpha3)*d4 ],
            	       [                   0,                   0,            0,               1 ]])
	T3_4 = T3_4.subs(s)


	T4_5 = Matrix([[             cos(q5),            -sin(q5),            0,              a4 ],
	               [ sin(q5)*cos(alpha4), cos(q5)*cos(alpha4), -sin(alpha4), -sin(alpha4)*d5 ],
        	       [ sin(q5)*sin(alpha4), cos(q5)*sin(alpha4),  cos(alpha4),  cos(alpha4)*d5 ],
	               [                   0,                   0,            0,               1 ]])
	T4_5 = T4_5.subs(s)


	T5_6 = Matrix([[             cos(q6),            -sin(q6),            0,              a5 ],
        	       [ sin(q6)*cos(alpha5), cos(q6)*cos(alpha5), -sin(alpha5), -sin(alpha5)*d6 ],
	               [ sin(q6)*sin(alpha5), cos(q6)*sin(alpha5),  cos(alpha5),  cos(alpha5)*d6 ],
            	       [                   0,                   0,            0,               1 ]])
	T5_6 = T5_6.subs(s)


	T6_G = Matrix([[             cos(q7),            -sin(q7),            0,              a6 ],
        	       [ sin(q7)*cos(alpha6), cos(q7)*cos(alpha6), -sin(alpha6), -sin(alpha1)*d7 ],
	               [ sin(q7)*sin(alpha6), cos(q7)*sin(alpha6),  cos(alpha6),  cos(alpha1)*d7 ],
            	       [                   0,                   0,            0,               1 ]])
	T6_G = T6_G.subs(s)



        # Transform from base link to end effector
	T0_2 = T0_1 * T1_2
	T0_3 = T0_2 * T2_3
	T0_4 = T0_3 * T3_4
	T0_5 = T0_4 * T4_5
	T0_6 = T0_5 * T5_6
	T0_G = T0_6 * T6_G

        # Correcting gripper Orientation to  be same that of base frame
        R_z = rot_z(pi)
        R_y = rot_y(-pi/2)
        R_corr = R_z * R_y

	R0_3 = T0_3[0:3, 0:3]
	R0_3_inv = R0_3**(-1)

        for i in xrange(0, len(req.poses)):
	    main_start = datetime.datetime.now()
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

            # Extract end-effector position and orientation from request
            # px,py,pz = end-effector position
            # roll, pitch, yaw = end-effector orientation
            px = req.poses[i].position.x
            py = req.poses[i].position.y
            pz = req.poses[i].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[i].orientation.x, req.poses[i].orientation.y,
                    req.poses[i].orientation.z, req.poses[i].orientation.w])

            # Calculate joint angles using Geometric IK method
	    WC_start = datetime.datetime.now()
	    R_roll = rot_x(roll)
            R_pitch = rot_y(pitch)
            R_yaw = rot_z(yaw)
	    R0_6 = R_roll * R_pitch * R_yaw * R_corr

            # Calculate WC - Wrist Center
	    # Wrist Center is d7 distance away from End Effector
            P = Matrix([px, py, pz])
	    WC = P - R0_6 * Matrix([0, 0, s[d7]])

	    WC_end = datetime.datetime.now()
	    WC_delta = WC_end - WC_start

	    theta1_3_start = datetime.datetime.now()

	    # theta1, theta2 and theta3 are are calculated using geometry.
            # Calculate theta1
            theta1 = atan2(WC[1], WC[0]).evalf()

	    # Calculate theta2
            s1 = sqrt(WC[0]**2 + WC[1]**2) - s[a1]
            s2 = WC[2] - s[d1]
            s3 = sqrt(s1**2 + s2**2)
            s4 = sqrt(s[a3]**2 + s[d4]**2)

            beeta1 = atan2(s2, s1)

            cos_beeta2 = (s[a2]**2 + s3**2 - s4**2)/(2*s[a2]*s3)
            sin_beeta2 = sqrt(1-cos_beeta2**2)
            beeta2 = atan2(sin_beeta2, cos_beeta2)

            theta2 = (pi/2 - beeta1 - beeta2).evalf()

            # Calculate theta3
            cos_beeta3 = (s[a2]**2 + s4**2 - s3**2)/(2*s[a2]*s4)
            sin_beeta3 = sqrt(1 - cos_beeta3**2)
            beeta3 = atan2(sin_beeta3, cos_beeta3)

            beeta4 = atan2(-s[a3], s[d4])

            theta3 = (pi/2 - beeta4 - beeta3).evalf()
	    theta1_3_end = datetime.datetime.now()
	    theta1_3_delta = theta1_3_end - theta1_3_start

            # Calculate theta4, 5, 6:
	    theta4_6_start = datetime.datetime.now()

	    # Refering to the leassons
	    R3_6 = R0_3_inv * R0_6
	    R3_6_num = R3_6.evalf(subs={q1: theta1, q2: theta2, q3: theta3})

            theta4 = atan2(R3_6_num[2, 2], -R3_6_num[0, 2]).evalf()
	    theta5 = atan2(sqrt(1 - R3_6_num[1, 2]**2), R3_6_num[1, 2]).evalf()
            theta6 = atan2(-R3_6_num[1, 1], R3_6_num[1, 0]).evalf()
	    theta4_6_end = datetime.datetime.now()
	    theta4_6_delta = theta4_6_end - theta4_6_start

	    main_end = datetime.datetime.now()
	    main_delta = main_end - main_start

	    print ('time - main, WC, theta1_3, theta4_6: ', main_delta, WC_delta, theta1_3_delta, theta4_6_delta)
            print ('theta1 to 6: ', theta1, theta2, theta3, theta4, theta5, theta6)


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
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
