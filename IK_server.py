#!/usr/bin/env python

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
import numpy as np

# Define Modified DH Transformation matrix
def TF_Matrix(alpha, a, d, q):

    TF = Matrix([[            cos(q),           -sin(q),           0,             a],
                 [ sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
		 [ sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
		 [                 0,                 0,           0,             1]])
    return TF

#Extract Rotation Matrix - Roll
def ROT_x(r):
    ROT_x = Matrix([[       1,       0,       0],
	            [       0,  cos(r), -sin(r)],
                    [       0,  sin(r),  cos(r)]])  # ROLL
    return ROT_x

#Extract Rotation Matrix - Pitch
def ROT_y(p):
    ROT_y = Matrix([[  cos(p),       0,  sin(p)],
	            [       0,       1,       0],
                    [ -sin(p),       0,  cos(p)]])  # PITCH
    return ROT_y

#Extract Rotation Matrix - Yaw
def ROT_z(y):
    ROT_z = Matrix([[ cos(y),  -sin(y),       0],
	            [ sin(y),   cos(y),       0],
                    [      0,        0,       1]])  # YAW
    return ROT_z

def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    print (len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:

        ##################################
        ### Begin Forward Kinematics
        ##################################

	# Define DH param symbols
	a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')                                   # link lengths
	d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')                                   # link offsets
	alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6  = symbols('alpha0:7')  # joint twist angles
	q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')                                   # joint angles

	# Modified DH parameters
	DH_Table = { alpha0:      0, a0:      0, d1:   0.75, q1: q1,
		     alpha1: -pi/2., a1:   0.35, d2:      0, q2: q2-pi/2.,
                     alpha2:      0, a2:   1.25, d3:      0, q3: q3,
                     alpha3: -pi/2., a3: -0.054, d4:    1.5, q4: q4,
                     alpha4:  pi/2., a4:      0, d5:      0, q5: q5,
                     alpha5: -pi/2., a5:      0, d6:      0, q6: q6,
                     alpha6:      0, a6:      0, d7:  0.303, q7: 0}

	# Create individual transformation matrices
        # Homogeneous Transforms between Individual Neighboring Links
        T0_1  = TF_Matrix(alpha0, a0, d1, q1).subs(DH_Table)
        T1_2  = TF_Matrix(alpha1, a1, d2, q2).subs(DH_Table)	
        T2_3  = TF_Matrix(alpha2, a2, d3, q3).subs(DH_Table)

	# Compensate for rotation discrepancy between DH parameters and Gazebo
	ROT_corr = ROT_z(pi) * ROT_y(-pi/2.)       # 180 degrees yaw, -90 degrees pitch

        ##################################
        ### End Forward Kinematics
        ##################################

        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            print ('Pose Number:' + str(x))

            joint_trajectory_point = JointTrajectoryPoint()

	    # Extract end-effector position and orientation from request
	    # px,py,pz = end-effector position
	    # roll, pitch, yaw = end-effector orientation

            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
		                [req.poses[x].orientation.x, 
		                 req.poses[x].orientation.y,
		                 req.poses[x].orientation.z, 
		                 req.poses[x].orientation.w])

            ##################################
            ### Begin Inverse Kinematics
            ##################################

            # Total Rotational Transform including gripper orientation correction
	    ROT_EE = ROT_z(yaw) * ROT_y(pitch) * ROT_x(roll) * ROT_corr

            # End-Effector Position Px, Py, Pz
	    EE = Matrix([[px],[py],[pz]])
	  
            # Calculate Wrist Center
	    WC = EE - (0.303) * ROT_EE[:,2]

            # Calculate joint angles using Geom,etric IK method
	    theta1 = atan2(WC[1], WC[0])

            # SSS triangle for theta2 and theta3 (Law of Cosines)
	    # sides
	    A = 1.501   # sqrt(1.5^2 + -0.054^2)
            B = sqrt(pow((sqrt(WC[0]*WC[0] + WC[1]*WC[1]) - 0.35), 2) + pow((WC[2] - 0.75), 2))
            C = 1.25
            # angles
            a = acos((B*B + C*C - A*A) / (2*B*C))
            b = acos((A*A + C*C - B*B) / (2*A*C))
            c = acos((A*A + B*B - C*C) / (2*A*B))

            theta2 = pi/2 - a - atan2(WC[2]-0.75, sqrt(WC[0]*WC[0]+WC[1]*WC[1])-0.35)
            theta3 = pi/2 - (b + 0.036)       # 0.036 accounts for sag in link4 of -0.0054m

	    # Extract rotation matrix R0_3 from transformation matrix T0_3 then substitute angles q1-3
	    R0_3 = T0_1[0:3,0:3] * T1_2[0:3,0:3] * T2_3[0:3,0:3]
    	    R0_3 = R0_3.evalf(subs={q1: theta1, q2: theta2, q3:theta3})

	    # Get rotation matrix R3_6 from (inverse of R0_3 * R_EE)
	    R3_6 = R0_3.inv(method="LU") * ROT_EE

	    # Euler angles from rotation matrix
            theta4 = atan2(R3_6[2,2], -R3_6[0,2])
            theta5 = atan2(sqrt(R3_6[0, 2]*R3_6[0, 2] + R3_6[2, 2]*R3_6[2, 2]), R3_6[1, 2])
            theta6 = atan2(-R3_6[1,1],R3_6[1,0])

            ##################################
            ### End Inverse Kinematics
            ##################################
            
            # Populate response for the IK request
	    joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
	    joint_trajectory_list.append(joint_trajectory_point)
            print(theta1.evalf(), theta2.evalf(), theta3.evalf(), theta4.evalf(), theta5.evalf(), theta6.evalf())

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