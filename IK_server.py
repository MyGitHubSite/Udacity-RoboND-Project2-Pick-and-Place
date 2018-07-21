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

#Define Rotation Matrix - Roll
def ROT_x(r):
    ROT_x = Matrix([[       1,       0,       0],
	                [       0,  cos(r), -sin(r)],
                    [       0,  sin(r),  cos(r)]])
    return ROT_x

#Define Rotation Matrix - Pitch
def ROT_y(p):
    ROT_y = Matrix([[  cos(p),       0,  sin(p)],
	                [       0,       1,       0],
                    [ -sin(p),       0,  cos(p)]])
    return ROT_y

#Define Rotation Matrix - Yaw
def ROT_z(y):
    ROT_z = Matrix([[ cos(y),  -sin(y),       0],
	                [ sin(y),   cos(y),       0],
                    [      0,        0,       1]])
    return ROT_z

#Convert from Degrees to Radians
def DegToRad(d):
    return d * pi / 180

def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))

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
	    DH_Table = {alpha0:      0, a0:      0, d1:   0.75, q1: q1,
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
            joint_trajectory_point = JointTrajectoryPoint()

	        # Extract end-effector position (Px,Py,Pz) and orientation (Roll, Pitch, Yaw) from request
            Px = req.poses[x].position.x
            Py = req.poses[x].position.y
            Pz = req.poses[x].position.z

            (Roll, Pitch, Yaw) = tf.transformations.euler_from_quaternion(
		                [req.poses[x].orientation.x, 
		                 req.poses[x].orientation.y,
		                 req.poses[x].orientation.z, 
		                 req.poses[x].orientation.w])

            ##################################
            ### Begin Inverse Kinematics
            ##################################

            # Total Rotational Transform Including Gripper Orientation Correction
	        ROT_EE = ROT_z(Yaw) * ROT_y(Pitch) * ROT_x(Roll) * ROT_corr

            # End-Effector Position Px, Py, Pz
	        EE = Matrix([[Px],[Py],[Pz]])
	  
            # Calculate Wrist Center
	        WC = EE - (0.303) * ROT_EE[:,2]

            # Calculate Joint Angles using Geometric IK method
	        theta1 = np.clip(atan2(WC[1], WC[0]), DegToRad(-185), DegToRad(185))

            # SSS triangle for theta2 and theta3 (Law of Cosines)
	        # sides
	        A = 1.501
            B = sqrt(pow((sqrt(WC[0]*WC[0] + WC[1]*WC[1]) - 0.35), 2) + pow((WC[2] - 0.75), 2))
            C = 1.25
            # angles
            a = acos((B*B + C*C - A*A) / (2*B*C))
            b = acos((A*A + C*C - B*B) / (2*A*C))
            c = acos((A*A + B*B - C*C) / (2*A*B))

            theta2 = np.clip(pi/2 - a - atan2(WC[2]-0.75, sqrt(WC[0]*WC[0]+WC[1]*WC[1])-0.35), DegToRad(-45), DegToRad(85))
            theta3 = np.clip(pi/2 - (b + 0.036), DegToRad(-210), DegToRad(65))   # 0.036 accounts for sag in link4 of -0.054m

	        # Extract rotation matrix R0_3 from transformation matrix T0_3 then substitute angles q1-3
	        R0_3 = T0_1[0:3,0:3] * T1_2[0:3,0:3] * T2_3[0:3,0:3]
    	    R0_3 = R0_3.evalf(subs={q1: theta1, q2: theta2, q3:theta3})

	        # Get rotation matrix R3_6 from (inverse of R0_3 * R_EE)
	        R3_6 = R0_3.inv(method="LU") * ROT_EE

	        # Euler angles from rotation matrix
            theta4 = np.clip(atan2(R3_6[2,2], -R3_6[0,2]), DegToRad(-350), DegToRad(350))
            theta5 = np.clip(atan2(sqrt(R3_6[0, 2]*R3_6[0, 2] + R3_6[2, 2]*R3_6[2, 2]), R3_6[1, 2]), DegToRad(-125), DegToRad(125))
            theta6 = np.clip(atan2(-R3_6[1,1],R3_6[1,0]), DegToRad(-350), DegToRad(350))

            ##################################
            ### End Inverse Kinematics
            ##################################
            
            # Populate response for the IK request
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