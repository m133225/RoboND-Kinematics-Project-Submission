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

def getJointHomogenousTransform(alpha, a, d, angle):
  hTMatrix = Matrix([[cos(angle), -sin(angle), 0, a],
                     [sin(angle)*cos(alpha), cos(angle)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                     [sin(angle)*sin(alpha), cos(angle)*sin(alpha), cos(alpha), cos(alpha)*d],
                     [0, 0, 0, 1]])
  return hTMatrix

def rot_x(q):
    R_x = Matrix([[1, 0, 0],
                  [0, cos(q), -sin(q)],
                  [0, sin(q), cos(q)]])
    
    return R_x
    
def rot_y(q):              
    R_y = Matrix([[cos(q), 0, sin(q)],
                  [0, 1, 0],
                  [-sin(q), 0, cos(q)]])
    
    return R_y

def rot_z(q):    
    R_z = Matrix([[cos(q), -sin(q), 0],
                  [sin(q), cos(q), 0],
                  [0, 0, 1]])
    
    return R_z
              

def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
        ### FORWARD KINEMATICS setup ###
        alpha, angle = symbols('alpha angle')
	a, d = symbols('a d')
        q1, q2, q3, q4, q5, q6 = symbols('q1:7')
	hTMatrix = getJointHomogenousTransform(alpha, a, d, angle)

	alpha_v, a_v, d_v, angle_v = 0*pi/180, 0, 0.33 + 0.42, q1
	resultingMatrix_1 = hTMatrix.subs([(alpha, alpha_v),(a, a_v),(d, d_v),(angle, angle_v)])

	alpha_v, a_v, d_v, angle_v = 90*pi/180, 0.35, 0, q2 + 90*pi/180
	resultingMatrix_2 = hTMatrix.subs([(alpha, alpha_v),(a, a_v),(d, d_v),(angle, angle_v)])

	alpha_v, a_v, d_v, angle_v = 0*pi/180, 1.25, 0, q3 + 180*pi/180
	resultingMatrix_3 = hTMatrix.subs([(alpha, alpha_v),(a, a_v),(d, d_v),(angle, angle_v)])

        T0_3 = resultingMatrix_1 * resultingMatrix_2 * resultingMatrix_3
        R0_3 = T0_3[0:3, 0:3]

        # For computing EE error
        alpha_v, a_v, d_v, angle_v = -90*pi/180, 0.054, 0.96+0.54, q4 + 180*pi/180
        resultingMatrix_4 = hTMatrix.subs([(alpha, alpha_v),(a, a_v),(d, d_v),(angle, angle_v)])

        alpha_v, a_v, d_v, angle_v = -90*pi/180, 0, 0, q5
        resultingMatrix_5 = hTMatrix.subs([(alpha, alpha_v),(a, a_v),(d, d_v),(angle, angle_v)])

        alpha_v, a_v, d_v, angle_v = 90*pi/180, 0, 0, q6
        resultingMatrix_6 = hTMatrix.subs([(alpha, alpha_v),(a, a_v),(d, d_v),(angle, angle_v)])

        alpha_v, a_v, d_v, angle_v = 0*pi/180, 0, 0.193+0.11, 0
        resultingMatrix_7 = hTMatrix.subs([(alpha, alpha_v),(a, a_v),(d, d_v),(angle, angle_v)])
        
        T0_7 = T0_3 * resultingMatrix_4 * resultingMatrix_5 * resultingMatrix_6 * resultingMatrix_7

        dist_error = 0 # To hold cumulative errors
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

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])
     
            ### Start of INVERSE KINEMATICS calculations ###
            overall_rotation = rot_z(yaw) * rot_y(pitch) * rot_x(roll) * rot_y(90*pi/180) * rot_z(180*pi/180)
	    wc_x = px - 0.303 * overall_rotation[0, 2]
	    wc_y = py - 0.303 * overall_rotation[1, 2]
	    wc_z = pz - 0.303 * overall_rotation[2, 2]

	    link_2 = 1.25
	    link_3 = sqrt(1.5**2 + .054**2)
	    s = wc_z - 0.75
	    r = sqrt(wc_x**2 + wc_y**2) - 0.35

	    angle1 = atan2(wc_y, wc_x).evalf() 
	    angle2 = -(90*pi/180 - acos((link_2**2 + s**2 + r**2 - link_3**2)/(2 * link_2 * sqrt(s**2 + r**2))) - atan2(s,r)).evalf()
	    angle3 = -(90*pi/180 - acos((link_2**2 + link_3**2 - s**2 - r**2)/(2 * link_2 * link_3)) - atan2(0.054,1.5)).evalf()
	    
	    R3_6 = R0_3.subs([(q1, angle1),(q2, angle2),(q3, angle3)]).evalf().transpose() * overall_rotation

            ### Uncomment the following to print the symbolic R3_6, which was used to derive the equations
            # print simplify(rot_x(-pi/2) * rot_z(pi + q1) * rot_x(-pi/2) * rot_z(q2) * rot_x(pi/2) * rot_z(q3))
            # Of course, this could also simply be done using the derived transformation matrices

	    angle4 = atan2(R3_6[2,2], -R3_6[0,2]).evalf()
	    angle5 = atan2(sqrt(R3_6[2,2]**2 + R3_6[0,2]**2), R3_6[1,2]).evalf()
	    angle6 = atan2(R3_6[1,1], -R3_6[1,0]).evalf()

            # Calculate EE error
            resulting_EE = T0_7.subs([(q1, angle1), (q2, angle2), (q3, angle3), (q4, angle4), (q5, angle5)]).evalf()[0:3, 3]
            original_EE = Matrix([req.poses[x].position.x, req.poses[x].position.y, req.poses[x].position.z])
            error_EE = resulting_EE - original_EE
            dist_diff = sqrt(error_EE[0]**2 + error_EE[1]**2 + error_EE[2]**2)
            dist_error = dist_error + dist_diff


            theta1, theta2, theta3, theta4, theta5, theta6 = angle1, -angle2, -angle3, angle4, -angle5, angle6# theta2, theta3 and theta5 are negative since the
                                                                                                  	      # chosen z-axis direction has a opposite rotation

	    joint_trajectory_point.positions = [theta1.evalf(), theta2.evalf(), theta3.evalf(), theta4.evalf(), theta5.evalf(), theta6.evalf()] 
	    joint_trajectory_list.append(joint_trajectory_point)

        print "Average error in EE position:", dist_error/len(req.poses)
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
