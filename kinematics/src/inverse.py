#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from numpy import linalg
import cmath
from transform_func import *
from forward import *
from kinematics.srv import *
from sensor_msgs.msg import JointState
from kinematics.msg import *



"""
     ****** Inverse Kinematics ******
"""

#Select the optimal solutions among a set of feasible joint value solutions


#def run_inv(req):
#    np_pose =np.asarray(req.pose)
#    nz=np_pose.reshape((4,4))
    #curr_theta = [-91.71, -98.96, -126.22, -46.29, 91.39, -1.78]  # must be taken from simulator
#    joint_states = rospy.wait_for_message("joint_states", JointState)
#    curr_theta = joint_states.position
#    angle_desired = inv_kine(nz, curr_theta)
#    return InverseServResponse(angle_desired)


#def inv_server():
#    rospy.init_node('inv_server')
#    s = rospy.Service('angle_desired', InverseServ, run_inv)
#    print "Inverse kinematics server is ready"
#    rospy.spin()


def callback(data):
    np_pose =np.asarray(data.pose)
    nz=np_pose.reshape((4,4))
    joint_states = rospy.wait_for_message("joint_states", JointState)
    curr_theta = joint_states.position

    angle_desired=inv_kine(nz,curr_theta)
    pub = rospy.Publisher('Angles',matrix, queue_size=10)
    rospy.loginfo(angle_desired)
    pub.publish(angle_desired)
def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("Transform", matrix, callback)
    rospy.spin()




def select(theta_obt, theta_curr, w=[1] * 6):
    
    error = []
    for q in theta_obt:
        error.append(sum([w[i] * (q[i] - theta_curr[i]) ** 2 for i in range(6)]))

    return theta_obt[error.index(min(error))]


def inv_kine(T_06,theta_curr):
    # Initialization of a set of feasible solutions
    theta = np.mat(np.zeros((8, 6)))

    # theta1

    P05 = (T_06 * np.mat([0, 0, -d6, 1]).transpose() - np.mat([0, 0, 0, 1]).transpose())

    # The two solutions for theta1 correspond to the shoulder left or right

    theta[0:4, 0] = (pi / 2) + atan2(P05[(2 - 1), 0], P05[(1 - 1), 0]) + acos(d4 / sqrt(P05[(2 - 1), 0] * P05[(2 - 1), 0] + P05[(1 - 1), 0] * P05[(1 - 1), 0]))
    theta[4:8, 0] = (pi / 2) + atan2(P05[(2 - 1), 0], P05[(1 - 1), 0]) - acos(d4 / sqrt(P05[(2 - 1), 0] * P05[(2 - 1), 0] + P05[(1 - 1), 0] * P05[(1 - 1), 0]))

    #theta5
    # The two solutions for theta5 correspond to wrist up or down for each theta1.Therefore theta5 has 4 solutions
    cl = [0, 4]  
    for i in range(0, len(cl)):
        c = cl[i]
        T10 = linalg.inv(link(1, theta, c))
        T16 = T10 * T_06
        theta[c:c + 2, 4] = + acos((T16[2, 3] - d4) / d6)
        theta[c + 2:c + 4, 4] = - acos((T16[2, 3] - d4) / d6)

    # theta6
    # theta6 is not well-defined when sin(theta5) = 0 or when T16(1,3), T16(2,3) = 0.

    cl = [0, 2, 4, 6]
    for i in range(0, len(cl)):
        c = cl[i]
        T10 = linalg.inv(link(1, theta, c))
        T61 = linalg.inv(T10 * T_06)
        theta[c:c + 2, 5] = atan2((-T61[1, 2] / sin(theta[c, 4])), (T61[0, 2] / sin(theta[c, 4])))

    # theta3
    #theta3 has 2 solutions correspond to elbow right or left for each theta1 and theta5. Therefore theta3 has 8 solutions
    cl = [0, 2, 4, 6]
    for i in range(0, len(cl)):
        c = cl[i]
        T10 = linalg.inv(link(1, theta, c))
        T56 = link(6, theta, c)
        T45 = link(5, theta, c)
        T64= linalg.inv(T45 * T56)
        T14 = (T10 * T_06) * T64
        P13 = T14 * np.mat([0, -d4, 0, 1]).transpose() - np.mat([0, 0, 0, 1]).transpose()
        t3 = cmath.acos((linalg.norm(P13) ** 2 - a2 ** 2 - a3 ** 2) / (2 * a2 * a3))
        theta[c, 2] = t3.real
        theta[c + 1, 2] = -t3.real

    #theta2

    cl = [0, 1, 2, 3, 4, 5, 6, 7]
    for i in range(0, len(cl)):
        c = cl[i]
        T10 = linalg.inv(link(1, theta, c))
        T65 = linalg.inv(link(6, theta, c))
        T54 = linalg.inv(link(5, theta, c))
        T14 = (T10 * T_06) * T65* T54
        P13 = T14 * np.mat([0, -d4, 0, 1]).transpose() - np.mat([0, 0, 0, 1]).transpose()
        theta[c, 1] = -atan2(P13[1], -P13[0]) + asin(a3 * sin(theta[c, 2]) / linalg.norm(P13))

        # theta 4

        T32 = linalg.inv(link(3, theta, c))
        T21 = linalg.inv(link(2, theta, c))
        T34 = T32 * T21 * T14
        theta[c, 3] = atan2(T34[1, 0], T34[0, 0])

    theta = theta.tolist()

    # Select the closest solution
    theta_inc = select(theta, theta_curr)

    return theta_inc

if __name__ == "__main__":
    listener()


