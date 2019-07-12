#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from math import *
import sys
from kinematics.msg import *
from sensor_msgs.msg import JointState

ROBOT= 'UR3'

if ROBOT == 'UR3':

    # robot parameters in mm
    d1 = 151.9
    d4 = 112.35
    d5 = 85.35
    d6 = 81.9
    a2 = -243.65
    a3 = -213.25

elif ROBOT == 'UR5':

   # robot parameters in mm
    d1 = 89.159
    d4 = 109.15
    d5 = 94.65
    d6 = 82.3
    a2 = -425.0
    a3 = -392.25

d = np.mat([d1, 0, 0, d4, d5, d6]) 
a = np.mat([0, a2, a3, 0, 0, 0])  
alpha = np.mat([(pi / 2), 0, 0, (pi / 2), -(pi / 2), 0]) 

"""
    ****** Forward Kinematics ******
"""

def link(i, theta, c):
    R_zt = np.mat([[cos(theta[c, i - 1]), -sin(theta[c, i - 1]), 0, 0],
                   [sin(theta[c, i - 1]), cos(theta[c, i - 1]), 0, 0],
                   [0, 0, 1, 0],
                   [0, 0, 0, 1]])

    T_zd = np.mat(np.identity(4))
    T_zd[2, 3] = d[0, i - 1]

    R_xa = np.mat([[1, 0, 0, 0],
                   [0, cos(alpha[0, i - 1]), -sin(alpha[0, i - 1]), 0],
                   [0, sin(alpha[0, i - 1]), cos(alpha[0, i - 1]), 0],
                   [0, 0, 0, 1]])

    T_xa = np.mat(np.identity(4))
    T_xa[0, 3] = a[0, i - 1]

    A_i = R_zt * T_zd * T_xa * R_xa

    return A_i

pub = rospy.Publisher('Tcurrent',matrix, queue_size=10)
def fwd_kine(data):
    print(data.position)

    curr_theta=np.mat(data.position)

    A_1 = link(1, curr_theta, c=0)
    A_2 = link(2, curr_theta, c=0)
    A_3 = link(3, curr_theta, c=0)
    A_4 = link(4, curr_theta, c=0)
    A_5 = link(5, curr_theta, c=0)
    A_6 = link(6, curr_theta, c=0)

    T_06 = A_1 * A_2 * A_3 * A_4 * A_5 * A_6

    z=np.asarray(T_06)
    x=z.reshape(16)

    np.save("/home/rnm_grp4/matrices/mat_",z)
    rospy.loginfo(x)
    pub.publish(x)
#        rate.sleep()
#    return np.around(T_06, decimals=15)

def listener():
    rospy.init_node("forward", anonymous=True)
    rospy.Subscriber("joint_states", JointState ,fwd_kine)
    rospy.spin()
if __name__ == "__main__":
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
