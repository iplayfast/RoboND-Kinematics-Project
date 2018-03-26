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

def TF_Matrix(twist,length,Offset,q):
                TF = Matrix([[cos(q), -sin(q),    0.0,  length],
                             [sin(q) * cos(twist),cos(q) * cos(twist), -sin(twist), -sin(twist) * Offset],
                             [sin(q) * sin(twist),cos(q) * sin(twist), cos(twist),     cos(twist) * Offset],
                             [0.0,                  0.0, 0.0,              1.0]])
                return TF


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:

            ### Your FK code here
            # Create symbol    # dh parameter symbols
        linkOffset1,linkOffset2,linkOffset3,linkOffset4,linkOffset5,linkOffset6,linkOffset7 = symbols('linkOffset1:8')  #link offset
        linkLength0,linkLength1,linkLength2,linkLength3,linkLength4,linkLength5,linkLength6 = symbols('linkLength0:7')  #link length
        linkTwist0,linkTwist1,linkTwist2,linkTwist3,linkTwist4,linkTwist5,linkTwist6 = symbols('linkTwist0:7') # twist angle
        # joint angle symbols
        joint1, joint2,joint3,joint4,joint5,joint6,joint7 = symbols('joint1:8')

            
            # Create Modified DH parameters
        DH_Table = { linkTwist0:   0.0, linkLength0:   0.0, linkOffset1: 0.75, joint1:  joint1,
                     linkTwist1:-pi/2., linkLength1:  0.35, linkOffset2:  0.0, joint2: -pi/2.0 + joint2,
                     linkTwist2:   0.0, linkLength2:  1.25, linkOffset3:  0.0, joint3:  joint3,
                     linkTwist3:-pi/2.0,linkLength3:-0.054, linkOffset4:  1.5, joint4:  joint4,
                     linkTwist4: pi/2.0,linkLength4:   0.0, linkOffset5:  0.0, joint5:  joint5,
                     linkTwist5:-pi/2.0,linkLength5:   0.0, linkOffset6:  0.0, joint6:  joint6,
                     linkTwist6:    0.0,linkLength6:   0.0, linkOffset7:0.303, joint7:  0.0 }
            #
            #
            # Define Modified DH Transformation matrix
        T0_1 = TF_Matrix(linkTwist0, linkLength0, linkOffset1,joint1).subs(DH_Table)
        T1_2 = TF_Matrix(linkTwist1, linkLength1, linkOffset2,joint2).subs(DH_Table)
        T2_3 = TF_Matrix(linkTwist2, linkLength2, linkOffset3,joint3).subs(DH_Table)
        T3_4 = TF_Matrix(linkTwist3, linkLength3, linkOffset4,joint4).subs(DH_Table)
        T4_5 = TF_Matrix(linkTwist4, linkLength4, linkOffset5,joint5).subs(DH_Table)
        T5_6 = TF_Matrix(linkTwist5, linkLength5, linkOffset6,joint6).subs(DH_Table)
        T6_EE =TF_Matrix(linkTwist6, linkLength6, linkOffset7,joint7).subs(DH_Table)
        T0_EE = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_EE
     #

        #correction to orientation of Gripper in UDRF vs DH
        R_corr = Matrix([[0,0,1.0,0],[0,-1.0,0,0],[1.0,0,0,0],[0,0,0,1.0]])
        # apply correction to result
        T0_EE_corr = (T0_EE * R_corr)

            #
            # Create individual transformation matrices
            #
            #
            # Extract rotation matrices from the transformation matrices
            #
            #
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

            ### Your IK code here
    ###
        # extract end-effector position and orientation from request
        # px,py,pz = end-effector position
        # roll, pitch, yaw = end-effector orientation
        px = req.poses[x].position.x
        py = req.poses[x].position.y
        pz = req.poses[x].position.z

        (roll,pitch,yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x,req.poses[x].orientation.y,
                 req.poses[x].orientation.z, req.poses[x].orientation.w]);

        r,  p,  y = symbols('r p y')
        R_roll = Matrix([[1,0,0],
                        [0,cos(r), -sin(r)],
                        [0,sin(r), cos(r)]])    #roll
        R_pitch = Matrix([[cos(p),    0,  sin(p)  ],
                        [0,         1,          0],
                        [-sin(p),   0,  cos(p)]])   #pitch
        R_yaw = Matrix([[cos(y), -sin(y), 0],
                        [sin(y),    cos(y), 0],
                        [0, 0,  1]])        #yaw

        ROT_EE = R_yaw * R_pitch * R_roll

        Rot_Error = R_yaw.subs(y, radians(180)) * R_pitch.subs(p,radians(-90))

        ROT_EE = ROT_EE * Rot_Error

        ROT_EE = ROT_EE.subs({'r':roll, 'p': pitch, 'y': yaw })
        EE = Matrix([[px],[py],[pz]])
        WC = EE - (0.303) * ROT_EE[:,2]

        # calculate joint angles using Geometrix IK method
        linkLength1 = atan2(WC[1],WC[0])
        #sss triangle for linkLength2 and linkLength3
        side_a = 1.501
        side_b = sqrt(pow((sqrt(WC[0]*WC[0]+WC[1]*WC[1]) - 0.35),2) + pow((WC[2] - 0.75),2))
        side_c = 1.25

        angle_a = acos((side_b * side_b + side_c * side_c - side_a * side_a) / (2 * side_b * side_c))
        angle_b = acos((side_a * side_a + side_c * side_c - side_b * side_b) / (2 * side_a * side_c))
        angle_c = acos((side_a * side_a + side_b * side_b - side_c * side_c) / (2 * side_a * side_b))
        
        linkLength2 = pi / 2 - angle_a - atan2(WC[2] - 0.75, sqrt(WC[0] * WC[0] + WC[1] * WC[1]) - 0.35)
        linkLength3 = pi / 2 - (angle_b + 0.036) # 0.036 accounts for sag in link4 of -0.054
        R0_3 = T0_1[0:3,0:3] * T1_2[0:3,0:3] * T2_3[0:3,0:3]
        R0_3 = R0_3.evalf(subs={joint1: linkLength1, joint2: linkLength2, joint3: linkLength3})

        R3_6 = R0_3.inv("LU") * ROT_EE
        # Euler angles from rotation matrix
        linkLength4 = atan2(R3_6[2,2], -R3_6[0,2])
        linkLength5 = atan2(sqrt(R3_6[0,2]*R3_6[0,2] + R3_6[2,2]*R3_6[2,2]),R3_6[1,2])
        linkLength6 = atan2(-R3_6[1,1],R3_6[1,0])

  	    # Compensate for rotation discrepancy between DH parameters and Gazebo
	    #
	    #
	    # Calculate joint angles using Geometric IK method
	    #
	    #
            ###

            # Populate response for the IK request
            # In the next line replace linkLength1,linkLength2...,linkLength6 by your joint angle variables
	joint_trajectory_point.positions = [linkLength1, linkLength2, linkLength3, linkLength4, linkLength5, linkLength6]
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
